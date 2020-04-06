#include <math.h>
#include <inttypes.h>
#include "lcd_ili9341.h"
#include "gl.h"

uint16_t gl_buffer[2*(LCD_MAXY+1)];
uint16_t *bgbuffer = gl_buffer;
int16_t zbuf[LCD_MAXY+1];
uint16_t gl_x=1;

const float sin_table[256] = {0.00000,0.02454,0.04907,0.07356,0.09802,0.12241,0.14673,0.17096,
  0.19509,0.21910,0.24298,0.26671,0.29028,0.31368,0.33689,0.35990,0.38268,0.40524,0.42756,0.44961,
  0.47140,0.49290,0.51410,0.53500,0.55557,0.57581,0.59570,0.61523,0.63439,0.65317,0.67156,0.68954,
  0.70711,0.72425,0.74095,0.75721,0.77301,0.78835,0.80321,0.81758,0.83147,0.84485,0.85773,0.87009,
  0.88192,0.89322,0.90399,0.91421,0.92388,0.93299,0.94154,0.94953,0.95694,0.96378,0.97003,0.97570,
  0.98079,0.98528,0.98918,0.99248,0.99518,0.99729,0.99880,0.99970,1.00000,0.99970,0.99880,0.99729,
  0.99518,0.99248,0.98918,0.98528,0.98079,0.97570,0.97003,0.96378,0.95694,0.94953,0.94154,0.93299,
  0.92388,0.91421,0.90399,0.89322,0.88192,0.87009,0.85773,0.84485,0.83147,0.81758,0.80321,0.78835,
  0.77301,0.75721,0.74095,0.72425,0.70711,0.68954,0.67156,0.65317,0.63439,0.61523,0.59570,0.57581,
  0.55557,0.53500,0.51410,0.49290,0.47140,0.44961,0.42756,0.40524,0.38268,0.35990,0.33689,0.31368,
  0.29028,0.26671,0.24298,0.21910,0.19509,0.17096,0.14673,0.12241,0.09802,0.07356,0.04907,0.02454,
  0.00000,-0.02454,-0.04907,-0.07356,-0.09802,-0.12241,-0.14673,-0.17096,-0.19509,-0.21910,-0.24298,
  -0.26671,-0.29028,-0.31368,-0.33689,-0.35990,-0.38268,-0.40524,-0.42756,-0.44961,-0.47140,-0.49290,
  -0.51410,-0.53500,-0.55557,-0.57581,-0.59570,-0.61523,-0.63439,-0.65317,-0.67156,-0.68954,-0.70711,
  -0.72425,-0.74095,-0.75721,-0.77301,-0.78835,-0.80321,-0.81758,-0.83147,-0.84485,-0.85773,-0.87009,
  -0.88192,-0.89322,-0.90399,-0.91421,-0.92388,-0.93299,-0.94154,-0.94953,-0.95694,-0.96378,-0.97003,
  -0.97570,-0.98079,-0.98528,-0.98918,-0.99248,-0.99518,-0.99729,-0.99880,-0.99970,-1.00000,-0.99970,
  -0.99880,-0.99729,-0.99518,-0.99248,-0.98918,-0.98528,-0.98079,-0.97570,-0.97003,-0.96378,-0.95694,
  -0.94953,-0.94154,-0.93299,-0.92388,-0.91421,-0.90399,-0.89322,-0.88192,-0.87009,-0.85773,-0.84485,
  -0.83147,-0.81758,-0.80321,-0.78835,-0.77301,-0.75721,-0.74095,-0.72425,-0.70711,-0.68954,-0.67156,
  -0.65317,-0.63439,-0.61523,-0.59570,-0.57581,-0.55557,-0.53500,-0.51410,-0.49290,-0.47140,-0.44961,
  -0.42756,-0.40524,-0.38268,-0.35990,-0.33689,-0.31368,-0.29028,-0.26671,-0.24298,-0.21910,-0.19509,
  -0.17096,-0.14673,-0.12241,-0.09802,-0.07356,-0.04907,-0.02454
};

#if LCD_MAXX > LCD_MAXY
  #define LCD_MIN2 (LCD_MAXY/2)
#else
  #define LCD_MIN2 (LCD_MAXX/2)
#endif

#define LED_R B,7,1,GPIO_PP50
void gl_sleep(uint32_t time){
  while(time--){asm volatile ("nop");}
}

struct glVector3int{
  int16_t x,y;
  int16_t z;
};
struct glPoint{
  uint16_t x,y;
  uint16_t col;
};
struct glLine{
  uint16_t x1;
  uint16_t x2;
  int32_t k;
  int32_t b;
  uint16_t y;
  uint16_t col;
};
struct glTriangle{
  int16_t x1,x2,x3;
  int32_t k1,k2,k3;
  int16_t b1,b2,b3;
  int32_t kz1,kz2,kz3;
  int16_t bz1,bz2,bz3;
  uint16_t col;
};

float glmat[16];

const struct glVector3 light = {.x=1,.y=0,.z=0,.col=0};

struct glPoint glPoints[GL_POINT_MAX];
uint8_t glPoint_num=0;
struct glLine glLines[GL_LINE_MAX];
uint8_t glLine_num=0;
struct glTriangle glTriangles[GL_TRIANGLE_MAX];
uint16_t glTriangle_num=0;

void glLoadIdentity(){
  glmat[0]=glmat[5]=glmat[10]=glmat[15]=1;
  glmat[1]=glmat[2]=glmat[3]=glmat[4]=glmat[6]=glmat[7]=0;
  glmat[8]=glmat[9]=glmat[11]=glmat[12]=glmat[13]=glmat[14]=0;
}

void glRotateXu(uint8_t alp){
  float si,co,buf[16];
  si = sin_table[alp];
  co = sin_table[(uint8_t)(alp+64)];
  for(alp=0;alp<16;alp++){buf[alp]=glmat[alp]; glmat[alp]=0;}
  for(alp=0;alp<4;alp++){
    glmat[alp]   = buf[alp];
    glmat[alp+4] = buf[alp+4]*co - buf[alp+8]*si;
    glmat[alp+8] = buf[alp+4]*si + buf[alp+8]*co;
    glmat[alp+12]= buf[alp+12];
  }
}
void glRotateYu(uint8_t alp){
  float si,co,buf[16];
  si = sin_table[alp];
  co = sin_table[(uint8_t)(alp+64)];
  for(alp=0;alp<16;alp++){buf[alp]=glmat[alp]; glmat[alp]=0;}
  for(alp=0;alp<4;alp++){
    glmat[alp]   = buf[alp]*co - buf[alp+8]*si;
    glmat[alp+4] = buf[alp+4];
    glmat[alp+8] = buf[alp]*si + buf[alp+8]*co;
    glmat[alp+12]= buf[alp+12];
  }
}
void glRotateZu(uint8_t alp){
  float si,co,buf[16];
  si = sin_table[alp];
  co = sin_table[(uint8_t)(alp+64)];
  for(alp=0;alp<16;alp++){buf[alp]=glmat[alp]; glmat[alp]=0;}
  for(alp=0;alp<4;alp++){
    glmat[alp]   = buf[alp]*co - buf[alp+4]*si;
    glmat[alp+4] = buf[alp]*si + buf[alp+4]*co;
    glmat[alp+8] = buf[alp+8];
    glmat[alp+12]= buf[alp+12];
  }
}
int __errno;
void glRotateu(uint8_t alp, float x, float y, float z){
  float si,co,co1,buf[16];
  float r = sqrtf(x*x+y*y+z*z);
  x /= r; y /= r; z /= r;
  si = sin_table[alp];
  co = sin_table[(uint8_t)(alp+64)];
  co1 = 1-co;
  for(alp=0;alp<16;alp++){buf[alp]=glmat[alp]; glmat[alp]=0;}
  for(alp=0;alp<4;alp++){
    glmat[alp]   = buf[alp]*(co+co1*x*x) + buf[alp+4]*(co1*x*y-si*z) + buf[alp+8]*(co1*x*z+si*y);
    glmat[alp+4] = buf[alp]*(co1*x*y+si*z) + buf[alp+4]*(co+co1*y*y) + buf[alp+8]*(co1*y*z-si*x);
    glmat[alp+8] = buf[alp]*(co1*x*z-si*y) + buf[alp+4]*(co1*y*z+si*x) + buf[alp+8]*(co+co1*z*z);
    glmat[alp+12]= buf[alp+12];
  }
}

void glRotateXf(float alp){
  alp = 128*alp/M_PI;
  glRotateXu(alp);
}
void glRotateYf(float alp){
  alp = 128*alp/M_PI;
  glRotateYu(alp);
}
void glRotateZf(float alp){
  alp = 128*alp/M_PI;
  glRotateZu(alp);
}
void glRotatef(float alp, float x, float y, float z){
  alp = 128*alp/M_PI;
  glRotateu(alp,x,y,z);
}

void glTranslatef(float x, float y, float z){
  glmat[3] +=x;
  glmat[7] +=y;
  glmat[11]+=z;
}

void glIntTransform(const struct glVector3 *src, struct glVector3int *res){
  struct glVector3 temp;
  temp.x = src->x*glmat[0] + src->y*glmat[1] + src->z*glmat[2] + glmat[3];
  temp.y = src->x*glmat[4] + src->y*glmat[5] + src->z*glmat[6] + glmat[7];
  temp.z = src->x*glmat[8] + src->y*glmat[9] + src->z*glmat[10]+ glmat[11];
  
  temp.x = temp.x*LCD_MIN2 + (LCD_MAXX/2);
  temp.y = temp.y*LCD_MIN2 + (LCD_MAXY/2);
  temp.z = temp.z*0x100+0x0FFF;
  res->x = temp.x;
  res->y = temp.y;
  res->z = temp.z;
}

void glIntClear(uint16_t color){
  uint16_t i;
  for(i=0;i<=LCD_MAXY;i++){
    bgbuffer[i]=color;
    zbuf[i]=0;
  }
}
void glDrawPoint(struct glVector3 *p1){
  struct glVector3int p;
  glIntTransform(p1,&p);
  glPoints[glPoint_num].x = p.x;
  glPoints[glPoint_num].y = p.y;
  glPoints[glPoint_num].col = p1->col;
  if(glPoint_num<GL_POINT_MAX)glPoint_num++;
}
void glIntDrawPoint(struct glPoint *p){
  if(gl_x == p->x)bgbuffer[p->y]=p->col;
}
void glDrawLine(struct glVector3 *_p1, struct glVector3 *_p2){
  struct glVector3int p1,p2;
  glIntTransform(_p1,&p1);
  glIntTransform(_p2,&p2);
  int32_t k,b;
  if(p1.x < p2.x){
    glLines[glLine_num].x1 = p1.x;
    glLines[glLine_num].x2 = p2.x;
  }else{
    glLines[glLine_num].x1 = p2.x;
    glLines[glLine_num].x2 = p1.x;
  }
  k = ((int32_t)p2.y - (int32_t)p1.y)<<16;
  k /= ((int32_t)p2.x - (int32_t)p1.x);
  b = (int32_t)p1.y - ((int32_t)(k*p1.x)>>16);
  glLines[glLine_num].k = k;
  glLines[glLine_num].b = b;
  glLines[glLine_num].col = _p1->col;
  if(glLine_num<GL_LINE_MAX)glLine_num++;
}
void glIntDrawLine(struct glLine *l){
  int32_t y,j;
  if(l->x2 < gl_x)return;
  if(l->x1 > gl_x)return;
  y = (int32_t)l->b + (((int32_t)l->k*gl_x)>>16);
  if(y > LCD_MAXY)y=LCD_MAXY;
  if(y<0)y=0;
  if(l->x1 == gl_x)l->y=y;
  if(y>l->y){
    for(j=l->y; j<=y; j++)bgbuffer[j]=l->col;
  }else{
    for(j=y; j<=l->y; j++)bgbuffer[j]=l->col;
  }
  l->y = y;
}
void glDrawTriangle(const struct glVector3 *t1, const struct glVector3 *t2, const struct glVector3 *t3){
  struct glVector3int v1,v2,v3;
  struct glVector3int *p1=&v1,*p2=&v2,*p3=&v3;
  struct glTriangle *t = &glTriangles[glTriangle_num];
  struct glVector3int *a;
  glIntTransform(t1,p1);
  glIntTransform(t2,p2);
  glIntTransform(t3,p3);
  if(p1->x > p2->x){a=p1; p1=p2; p2=a;}
  if(p1->x > p3->x){a=p1; p1=p3; p3=a;}
  if(p2->x > p3->x){a=p2; p2=p3; p3=a;}
  
  t->x1 = p1->x;
  t->x2 = p2->x;
  t->x3 = p3->x;
  
  t->k1 = ((int32_t)p2->y - (int32_t)p1->y)<<16;
  t->k1 /= ((int32_t)p2->x - (int32_t)p1->x);
  t->b1 = (int32_t)p1->y - ((int32_t)(t->k1*p1->x)>>16);
  t->kz1 = ((int32_t)p2->z - (int32_t)p1->z)<<16;
  t->kz1 /= ((int32_t)p2->x - (int32_t)p1->x);
  t->bz1 = (int32_t)p1->z - ((int32_t)(t->kz1*p1->x)>>16);
  
  t->k2 = ((int32_t)p3->y - (int32_t)p1->y)<<16;
  t->k2 /= ((int32_t)p3->x - (int32_t)p1->x);
  t->b2 = (int32_t)p1->y - (((int32_t)(t->k2*p1->x))>>16);
  t->kz2 = ((int32_t)p3->z - (int32_t)p1->z)<<16;
  t->kz2 /= ((int32_t)p3->x - (int32_t)p1->x);
  t->bz2 = (int32_t)p1->z - (((int32_t)(t->kz2*p1->x))>>16);
  
  t->k3 = ((int32_t)p2->y - (int32_t)p3->y)<<16;
  t->k3 /= ((int32_t)p2->x - (int32_t)p3->x);
  t->b3 = (int32_t)p3->y - ((int32_t)(t->k3*p3->x)>>16);
  t->kz3 = ((int32_t)p2->z - (int32_t)p3->z)<<16;
  t->kz3 /= ((int32_t)p2->x - (int32_t)p3->x);
  t->bz3 = (int32_t)p3->z - ((int32_t)(t->kz3*p3->x)>>16);
  
  t->col=t1->col;
  if(glTriangle_num<GL_TRIANGLE_MAX)glTriangle_num++;
}
void glDrawTriangleV(const struct glVector3 *t1, const struct glVector3 *t2, const struct glVector3 *t3, const struct glVector3 *n){
  struct glVector3int v1,v2,v3;
  struct glVector3int *p1=&v1,*p2=&v2,*p3=&v3;
  struct glTriangle *t = &glTriangles[glTriangle_num];
  struct glVector3int *a;
  struct glVector3 temp;
  float cosa=1;
  temp.x = n->x*glmat[0] + n->y*glmat[1] + n->z*glmat[2];
  temp.y = n->x*glmat[4] + n->y*glmat[5] + n->z*glmat[6];
  temp.z = n->x*glmat[8] + n->y*glmat[9] + n->z*glmat[10];
  if(temp.z < 0)return;
  cosa = (temp.x*light.x + temp.y*light.y + temp.z*light.z);
  cosa = fabs(cosa)+0.5;
  if(cosa>1)cosa=1;
  //cosa *= 0xFF;
  v1.x = (t1->col >> 8)&0b11111000;
  v1.y = (t1->col >> 3)&0b11111100;
  v1.z = (t1->col << 3)&0b11111000;
  //v1.x*=cosa; v1.y*=cosa; v1.z*=cosa;
  v1.x = (v1.x*cosa); v1.x &= 0b11111000;
  v1.y = (v1.y*cosa); v1.y &= 0b11111100;
  v1.z = (v1.z*cosa); v1.z &= 0b11111000;
  t->col = rgb2col(v1.x,v1.y,v1.z);
  
  glIntTransform(t1,p1);
  glIntTransform(t2,p2);
  glIntTransform(t3,p3);
  if(p1->x > p2->x){a=p1; p1=p2; p2=a;}
  if(p1->x > p3->x){a=p1; p1=p3; p3=a;}
  if(p2->x > p3->x){a=p2; p2=p3; p3=a;}
  
  t->x1 = p1->x;
  t->x2 = p2->x;
  t->x3 = p3->x;
  
  t->k1 = ((int32_t)p2->y - (int32_t)p1->y)<<16;
  t->k1 /= ((int32_t)p2->x - (int32_t)p1->x);
  t->b1 = (int32_t)p1->y - ((int32_t)(t->k1*p1->x)>>16);
  t->kz1 = ((int32_t)p2->z - (int32_t)p1->z)<<16;
  t->kz1 /= ((int32_t)p2->x - (int32_t)p1->x);
  t->bz1 = (int32_t)p1->z - ((int32_t)(t->kz1*p1->x)>>16);
  
  t->k2 = ((int32_t)p3->y - (int32_t)p1->y)<<16;
  t->k2 /= ((int32_t)p3->x - (int32_t)p1->x);
  t->b2 = (int32_t)p1->y - (((int32_t)(t->k2*p1->x))>>16);
  t->kz2 = ((int32_t)p3->z - (int32_t)p1->z)<<16;
  t->kz2 /= ((int32_t)p3->x - (int32_t)p1->x);
  t->bz2 = (int32_t)p1->z - (((int32_t)(t->kz2*p1->x))>>16);
  
  t->k3 = ((int32_t)p2->y - (int32_t)p3->y)<<16;
  t->k3 /= ((int32_t)p2->x - (int32_t)p3->x);
  t->b3 = (int32_t)p3->y - ((int32_t)(t->k3*p3->x)>>16);
  t->kz3 = ((int32_t)p2->z - (int32_t)p3->z)<<16;
  t->kz3 /= ((int32_t)p2->x - (int32_t)p3->x);
  t->bz3 = (int32_t)p3->z - ((int32_t)(t->kz3*p3->x)>>16);
  
  //t->col=t1->col;
  if(glTriangle_num<GL_TRIANGLE_MAX)glTriangle_num++;
}

void glIntDrawTriangle(struct glTriangle *t){
  int32_t y1,y2,y;
  int32_t z1,z2,z;
  int32_t k,b;
  if(t->x3 < gl_x)return;
  if(t->x1 > gl_x)return;
  if(t->x2 > gl_x){
    y1 = (int32_t)t->b1 + (((int32_t)t->k1*gl_x)>>16);
    y2 = (int32_t)t->b2 + (((int32_t)t->k2*gl_x)>>16);
    z1 = (int32_t)t->bz1 + (((int32_t)t->kz1*gl_x)>>16);
    z2 = (int32_t)t->bz2 + (((int32_t)t->kz2*gl_x)>>16);
    k = ((int32_t)z2 - (int32_t)z1)<<16;
    k /= ((int32_t)y2 - (int32_t)y1);
    b = (int32_t)z1 - (((int32_t)(k*y1))>>16);
  }else{
    y1 = (int32_t)t->b3 + (((int32_t)t->k3*gl_x)>>16);
    y2 = (int32_t)t->b2 + (((int32_t)t->k2*gl_x)>>16);
    z1 = (int32_t)t->bz3 + (((int32_t)t->kz3*gl_x)>>16);
    z2 = (int32_t)t->bz2 + (((int32_t)t->kz2*gl_x)>>16);
    k = ((int32_t)z2 - (int32_t)z1)<<16;
    k /= ((int32_t)y2 - (int32_t)y1);
    b = (int32_t)z1 - (((int32_t)(k*y1))>>16);
  }
  if(y1 > y2){y=y1; y1=y2; y2=y;}
  if(y1<0)y1=0;
  if(y1>LCD_MAXY)y1=LCD_MAXY;
  if(y2<0)y2=0;
  if(y2>LCD_MAXY)y2=LCD_MAXY;
  
  for(y=y1;y<y2;y++){
    z = (int32_t)b + (((int32_t)k*y)>>16);
    if(zbuf[y] < z){
      bgbuffer[y]=t->col;
      zbuf[y] = z;
    }
  }
}
void glIntDraw(){
  uint16_t i;
  glIntClear(0);
  for(i=0;i<glPoint_num;i++)glIntDrawPoint(&glPoints[i]);
  for(i=0;i<glLine_num;i++)glIntDrawLine(&glLines[i]);
  for(i=0;i<glTriangle_num;i++)glIntDrawTriangle(&glTriangles[i]);
}
void glInit(){
  lcd_dma_init(gl_buffer,10);
}

void glSwapBuffers(){
  gl_x=0;
  bgbuffer=gl_buffer;
  glIntDraw();
  bgbuffer=gl_buffer+(LCD_MAXY+1);
  lcd_dma_wait();
  lcd_dma_deinit();
  lcd_begin_area(0,0,LCD_MAXX,LCD_MAXY);
  lcd_dma_init(gl_buffer,(LCD_MAXY+1));
  for(gl_x=1;gl_x<=LCD_MAXX;gl_x++){
    glIntDraw();
    GPO_ON(LED_R);
    lcd_dma_wait();
    GPO_OFF(LED_R);
    lcd_dma_restart(bgbuffer,(LCD_MAXY+1));
    if(bgbuffer == gl_buffer)bgbuffer=gl_buffer+(LCD_MAXY+1);
      else bgbuffer=gl_buffer;
  }
  glPoint_num=0;
  glLine_num=0;
  glTriangle_num=0;
}


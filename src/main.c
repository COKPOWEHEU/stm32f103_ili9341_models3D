#include <stm32f10x.h>
#include "pinmacro.h"
#include "clock.h"
#include "lcd_ili9341.h"
#include "gl.h"
#include <math.h>

char hse_status=0;
#define LED_Y B,5,1,GPIO_PP50
#define LED_G B,6,1,GPIO_PP50
#define LED_R B,7,1,GPIO_PP50
#define BTN1  A,8,0,GPIO_HIZ
#define BTN2  A,9,0,GPIO_HIZ //используется для программирования по UART

void SystemInit(void){
  SysClockMax();
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIO_config(LED_R);
  GPIO_config(LED_G);
  GPIO_config(LED_Y);
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIO_mode(A, 8, GPIO_HIZ);
}

#include "../model/fly.h"
#include "../model/lur.h"

const struct glVector3 v[]={
  {.x=-1,.y=-1,.z=0,.col=rgb2col(0xFF,0,0)},
  {.x=1,.y=-1,.z=0,.col=rgb2col(0xFF,0,0)},
  {.x=-1,.y=1,.z=0,.col=rgb2col(0xFF,0,0)},
  {.x=1,.y=1,.z=0,.col=rgb2col(0xFF,0,0)},
  
  {.x=-1,.y=0,.z=-1,.col=rgb2col(0,0xFF,0)},
  {.x=1,.y=0,.z=-1,.col=rgb2col(0,0xFF,0)},
  {.x=-1,.y=0,.z=1,.col=rgb2col(0,0xFF,0)},
  {.x=1,.y=0,.z=1,.col=rgb2col(0,0xFF,0)},
  
  {.x=0,.y=-1,.z=-1,.col=rgb2col(0,0,0xFF)},
  {.x=0,.y=1,.z=-1,.col=rgb2col(0,0,0xFF)},
  {.x=0,.y=-1,.z=1,.col=rgb2col(0,0,0xFF)},
  {.x=0,.y=1,.z=1,.col=rgb2col(0,0,0xFF)}
};

int main(void){
  uint16_t i=0,j=0,k=0;
  uint8_t mode=0;
  uint8_t old_state=0;
  SystemInit();
  lcd_init();
  glInit();
  while(1){
    glLoadIdentity();
    
    glRotateZu(i>>8);
    glRotateXu(j>>8);
    glRotateYu(k>>8);
    
    i+=0xFF;
    j+=0x010F;
    k+=0x0250;
    
    if(mode == 0){
      glDrawTriangle(&v[0],&v[1],&v[2]);
      glDrawTriangle(&v[1],&v[2],&v[3]);
      glDrawTriangle(&v[4],&v[5],&v[6]);
      glDrawTriangle(&v[5],&v[6],&v[7]);
      glDrawTriangle(&v[8],&v[9],&v[10]);
      glDrawTriangle(&v[9],&v[10],&v[11]);
    }else if(mode == 1){
      Draw_lur();
    }else if(mode == 2){
      Draw_fly();
    }else{
      mode=0;
    }
    
    if(GPI_ON(BTN1)){
      old_state=1;
    }else{
      if(old_state == 1){
        mode++;
        if(mode > 2)mode=0;
      }
      old_state = 0;
    }
    
    glSwapBuffers();
    GPO_T(LED_G);
  }
}

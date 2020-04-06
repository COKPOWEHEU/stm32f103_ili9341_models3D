#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#define FNAME "lur2"
#define INPFIL "model/"FNAME".obj"
#define OUTFIL "res/"FNAME".h"

struct Vector3{
  float x,y,z;
  //float nx,ny,nz;
  //uint16_t col;
};
struct Triangle{
  unsigned int v1,v2,v3;
  unsigned int n1,n2,n3;
};

struct Vector3 vec[2000],norm[2000];
struct Triangle tr[2000];
uint16_t vec_num=0;
uint16_t tr_num=0;
uint16_t n_num=0;
float max=0;
uint16_t col=0;

int main(int argc, char **argv){
  FILE *fin = fopen(INPFIL,"rt");
  FILE *fout= fopen(OUTFIL,"wt");
  char buffer[256],*cur;
  int line=1;
  
  if(fin == NULL || fout == NULL){
    if(fin != NULL)fclose(fin);
    if(fout!= NULL)fclose(fout);
    printf("can not open file [%s] or [%s]\n",INPFIL,OUTFIL);
    return 0;
  }
  
  while(!feof(fin)){
    if(!fgets(buffer,sizeof(buffer),fin))break;
    line++;
    if(buffer[0] == '#')continue;
    if(buffer[0] == 'v'){
      if(buffer[1] == ' '){
        //"v "
        sscanf(buffer+2,"%f%f%f",&vec[vec_num].x,&vec[vec_num].y,&vec[vec_num].z);
        
        if(fabs(vec[vec_num].x)>max)max=fabs(vec[vec_num].x);
        if(fabs(vec[vec_num].y)>max)max=fabs(vec[vec_num].y);
        if(fabs(vec[vec_num].z)>max)max=fabs(vec[vec_num].z);
        
        if(vec_num<2000)vec_num++;
        continue;
      }else if(buffer[1]=='n' && buffer[2]==' '){
        //"vn "
        sscanf(buffer+3,"%f%f%f",&norm[n_num].x,&norm[n_num].y,&norm[n_num].z);
        
        if(n_num<2000)n_num++;
        continue;
      }
    }else if(buffer[0] == 'f'){
      if(buffer[1] == ' '){
        char c1,c2;
        uint16_t val;
        //"f "
        sscanf(buffer+2,"%u%*c%*u%*c%u %u%*c%*u%*c%u %u%*c%*u%*c%u",&tr[tr_num].v1,&tr[tr_num].n1,
          &tr[tr_num].v2,&tr[tr_num].n2,&tr[tr_num].v3,&tr[tr_num].n3);
        
        tr[tr_num].v1--;
        tr[tr_num].n1--;
        tr[tr_num].v2--;
        tr[tr_num].n2--;
        tr[tr_num].v3--;
        tr[tr_num].n3--;

        if(tr_num<2000)tr_num++;
        continue;
      }
    }
  }
  fclose(fin);
  

  fprintf(fout,"const struct glVector3  %s_vec[%i]={\n",FNAME,vec_num);
  for(line=0;line<vec_num;line++){
    fprintf(fout,"  {.x=%f,.y=%f,.z=%f,.col=0x9583},\n",vec[line].x/max,vec[line].y/max,vec[line].z/max);
  }
  fprintf(fout,"};\n\n");
  
  fprintf(fout,"const struct glVector3  %s_norm[%i]={\n",FNAME,n_num);
  for(line=0;line<n_num;line++){
    fprintf(fout,"  {.x=%f,.y=%f,.z=%f,.col=0},\n",norm[line].x,norm[line].y,norm[line].z);
  }
  fprintf(fout,"};\n\n");
  
  
  fprintf(fout,"void Draw_%s(){\n",FNAME);
  fprintf(fout,"//%u triangles\n",tr_num);
  for(line=0;line<tr_num;line++){
    fprintf(fout,"  glDrawTriangleV(&%s_vec[%u],&%s_vec[%u],&%s_vec[%u],&%s_norm[%u]);\n",FNAME,tr[line].v1,FNAME,tr[line].v2,FNAME,tr[line].v3,FNAME,tr[line].n1);
  }
  fprintf(fout,"};\n\n");
  
  fclose(fout);
}

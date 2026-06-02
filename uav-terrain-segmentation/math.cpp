#include<math.h>
#include<iostream>

void sub(double*res,double*a,double*b){
    for(int i=0; i<3;++i)
      res[i]=a[i]-b[i];
}

void cross(double*t,double*a,double*b){
    double res[3];
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];

    t[0]=res[0];
    t[1]=res[1];
    t[2]=res[2];
}

double len(double*a){
  return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
}

void mag(double*res,double *a){
  double temp[3];
  double l=len(a);
  for(int i=0; i<3;++i)
    temp[i]=a[i]/l;

  for(int i=0;i<3;++i)
    res[i]=temp[i];
}

void copy(double*a,double*b){
  for(int i=0;i<3;++i)
    a[i]=b[i];
}

double dot(double*a,double*b){
  double prod=0;
  for(int i=0; i<3; ++i)
    prod=prod+a[i]*b[i];
  return prod;
}

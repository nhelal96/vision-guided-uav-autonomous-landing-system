
#include "SafeLanding/helper.h"

#include<math.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>


namespace SafeLanding{

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

  Direction compute_movement_distance(ResultPoint&res_p,Direction&c_pos){

      double pcy=(res_p.p2[1]-res_p.p0[1])/2+res_p.p0[1];
      double pcx=(res_p.p2[0]-res_p.p0[0])/2+res_p.p0[0];
      double cy=YS/2.;
      double cx=XS/2.;

      double dy=(cy-pcy)/YS;
      double dx=(cx-pcx)/XS;


      Direction dir;
      dir.x=(dx+c_pos.x);
      dir.y=(dy+c_pos.y);

      return dir;
  }

  bool check_for_alignment(Direction&c_pos,Direction&dir){

      double dy=(c_pos.y-dir.y);
      double dx=(c_pos.x-dir.x);

      if(abs(dy)<=EPS)dir.y=0; // correct position
      if(abs(dx)<=EPS)dir.x=0;


      if(!dir.x && !dir.y)return true;
      return false;

  }

  Direction random_planner(Direction&c_pos){
    Direction dir;
    dir.x=choice[rand()%2]+c_pos.x;
    dir.y=choice[rand()%2]+c_pos.y;
    return dir;
  }


}

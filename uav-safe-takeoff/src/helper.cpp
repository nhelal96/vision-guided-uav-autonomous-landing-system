#include "safe_takeoff/helper.h"

#include <stdlib.h>
#include <time.h>

#include <iostream>

namespace SafeTakeoff{

  double size(ResultPoint&rect){
      double x=(rect.p2[0]-rect.p0[0]);
      double y=(rect.p2[1]-rect.p0[1]);
      double s=x+y;
      std::cout<<"size of rect= "<<s<<std::endl;
      return s;
  }

  Direction compute_movement_distance(ResultPoint&res_p,Direction&c_pos){

      double pcy=(res_p.p2[1]-res_p.p0[1])/2+res_p.p0[1];
      double pcx=(res_p.p2[0]-res_p.p0[0])/2+res_p.p0[0];
      double cy=YS/2.;
      double cx=XS/2.;

      double dy=(cy-pcy)/YS;
      double dx=(cx-pcx)/XS;

      Direction dir;
      dir.x=dx+c_pos.x;
      dir.y=dy+c_pos.y;

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

  bool is_big_enough(ResultPoint&res_p){
      int sz=size(res_p);
      return sz>=RECT_X+RECT_Y;
  }

  Direction random_planner(Direction&c_pos){
    Direction dir;
    dir.x=choice[rand()%2]+c_pos.x;
    dir.y=choice[rand()%2]+c_pos.y;
    return dir;
  }



}

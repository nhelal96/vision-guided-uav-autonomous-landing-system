
#include "qr_landing/helper.h"

#include<math.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>


namespace QRLanding{

  Direction planner(ResultPoint&res_p,Direction&initial_pos,Direction&c_pos){
    double pcy=(res_p.p2[1]-res_p.p0[1])/2+res_p.p0[1];
    double pcx=(res_p.p2[0]-res_p.p0[0])/2+res_p.p0[0];
    double cy=YSS/2.;
    double cx=XSS/2.;

    double dy=(cy-pcy)/YSS;
    double dx=(cx-pcx)/XSS;

    Direction dir;
    dir.x=dx;
    dir.y=dy;
    if(abs(dy)<=EPS && dy>=0)dir.y=0; // correct y position
    if(abs(dx)<=EPS && dx>=0)dir.x=0;

    // move the position of rectangle to memic the movement of the drone
    Direction diff;
    diff.x=c_pos.x-initial_pos.x;
    diff.y=c_pos.y-initial_pos.y;

    res_p.p0[0]+=diff.x*-1;
    res_p.p2[0]+=diff.x*-1;
    res_p.p0[1]+=diff.y*-1;
    res_p.p2[1]+=diff.y*-1;

    return dir;
  }

  Direction random_planner(){
    Direction dir;
    dir.x=choice[rand()%2];
    dir.y=choice[rand()%2];
    return dir;
  }


}

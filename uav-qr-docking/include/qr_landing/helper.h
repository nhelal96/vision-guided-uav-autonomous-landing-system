#ifndef HELPER_H
#define HELPER_H

#include<math.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <stdlib.h>
#include <time.h>
#include <vector>
#include <map>

namespace QRLanding{

  const int YSS=720;
  const int XSS=1280;
  const double EPS=0.05;
  const double choice[2]={-0.1,0.1};


  class ResultPoint{
  public:
    double p0[2],p2[2];
    bool valid;
    ResultPoint():valid(false){
      p0[0]=-100;
      p2[0]=-100;
      p0[1]=-100;
      p2[1]=-100;
    };
    ResultPoint* operator =(const ResultPoint *a){*this=*a; return this;}
  };

  class Direction{
  public:
    double x,y;
    Direction(){};
    Direction(double _dx, double _dy):x(_dx),y(_dy){};
    Direction* operator =(const Direction*a){*this=*a; return this;}

  };

  class QRObject{
  public:
      int type;
      std::vector<cv::Point>location;
      std::map<std::string,std::string>dict_data;
  };

  /**
    * @brief        convert solution to goal position
    * @param[in]    res_p, solution position
    * @param[in]    initial_pos, initial position of the drone
    * @param[in]    c_pos, current position of the drone
    * @return       goal position
  **/
  Direction planner(ResultPoint&,Direction&,Direction&);

  /**
    * @brief        generate random goal position in 1M diameter
    * @return       goal position
  **/
  Direction random_planner();
}

#endif

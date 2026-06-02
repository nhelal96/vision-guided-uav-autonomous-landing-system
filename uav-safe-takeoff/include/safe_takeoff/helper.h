#ifndef HELPER_H
#define HELPER_H

#include "safe_takeoff/helper.h"

#include <stdlib.h>
#include <time.h>

namespace SafeTakeoff{

  const int YS=720;
  const int XS=1280;
  const int RECT_Y=250;
  const int RECT_X=250;
  const double EPS=0.1;
  const int MIN_WINDOW=100;
  const int MAX_ERROR=30;
  const double drone_size=1.1;
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

  /**
    * @breif        compute the differnce distance betweem start
                    and end position of the drone
    * @param[in]    res_p, initial position of the drone
    * @param[in]    c_pos, final position of the drone
    * @return       2D distance differnce
  **/
  Direction compute_movement_distance(ResultPoint&,Direction&);

  /**
    * @brief        checks if drone arrived to the targer position
    * params[in]    c_pos, current position of the drone
    * @param[in]    dir, target position
  **/
  bool check_for_alignment(Direction&,Direction&);

  /**
    * @brief        generate random goal position in 1M diameter
    * @return       goal position
  **/
  Direction random_planner(Direction&);

  /**
    * @brief        checks if the free area if big enough
                    for the drone to take off safely
  **/
  bool is_big_enough(ResultPoint&);

  /**
    * @brief        computes the size of the solution rect
  **/
  double size(ResultPoint&rect);
}

#endif

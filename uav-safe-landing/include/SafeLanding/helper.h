#ifndef HELPER_H
#define HELPER_H

#include "SafeLanding/helper.h"

#include<math.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <stdlib.h>
#include <time.h>

namespace SafeLanding{

  const int YS=720;
  const int XS=1280;
  const int RECT_Y=400;
  const int RECT_X=570;
  const double EPS=0.1;
  const double choice[2]={-0.1,0.1};

  class DepthPoint{
  public:
      double pos[3];
      double nor[3];

      DepthPoint(){};
      DepthPoint* operator = (const DepthPoint *a) { *this=*a; return this; }
  };

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
    * @brief        helper function to write image on desk
    * @param[in]    img, to write on desk
    * @param[in]    i, index name
  **/
  void write(cv::Mat&img,int i);

  /**
    * @brief        subtract two 1D vector
    * @param[out]   res, pointer to an empty array to hold solution
    * @param[in]    a, pointer to the first vector
    * @param[in]    b, pointer to the second vector
  **/
  void sub(double*res,double*a,double*b);

  /**
    * @brief        computes cross product
    * @param[out]   t, pointer to an empty array to hold solution
    * @param[in]    a, pointer to the first vector
    * @param[in]    b, pointer to the second vector
  **/
  void cross(double*t,double*a,double*b);

  /**
    * @brief        computes the magnitute of a vector
    * @param[in]    a, pointer to the vector
    * @return       lenght of the vector
  **/
  double len(double*a);

  /**
    * @brief        computes the normalize magnitute of the vector
    * @param[out]   res, pointer to an empty array to hold the solution
    * @param[in]    a, pointer to the first vector
  **/
  void mag(double*res,double *a);

  /**
    * @brief        helper function to copy vector to another one
    * @param[out]   a, pointer to an empty array to hold the solution
    * @param[in]    b, pointer to the vector
  **/
  void copy(double*a,double*b);

  /**
    * @brief        computes dot product between two 1D vectors
    * @param[in]    a, pointer to the first vector
    * @param[in]    b, pointer to the second vector
    * @return       dot product
  **/
  double dot(double*a,double*b);

  /**
    * @brief        helper function to visualize an image
  **/
  void show(cv::Mat&img);


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
}

#endif

/**
  * algorithm.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  defining the algorithms of finding landing area
**/


#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "SafeLanding/helper.h"
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include<vector>



namespace SafeLanding{


  const int _DepthImage_invert_z      =0x00000001;    // brighter colros are closer
  const int _DepthImage_cos_correct_z =0x00000002;    // depth is perpendicular distance instead of euclid
  const int _DepthImage_smooth        =0x00000004;    // average position to reduce noise
  const double deg=M_PI/180.0;
  const double WX=0.6;
  const double WY=0.6;
  const double MAX_VARIANCE=0.005;


  class Algorithm{
      int xs,ys;
      int x0,y0,x1,y1; // solution index
      double *p0,*p1,*p2,*p3; // solution pointers
      std::vector<std::vector<DepthPoint> >cloud;
      cv::Mat depth;
      double depthFOVx,depthFOVy,
             depthx0,depthy0,
             depthz0,depthz1;
      int    depthcfg;
      double error;

  public:
      Algorithm();
      Algorithm(Algorithm& a)   { *this=a; }
      ~Algorithm()   { std::vector<std::vector<DepthPoint> >().swap(cloud); }
      Algorithm* operator = (const Algorithm *a) { *this=*a; return this; }


      /**
        * @brief        maps the coordinates of the QR in image
                        to local coordinates matches controller
                        coordinate system
        * @param[in]    indx, index of the solution in the objects
                        vector
        * @param[out]   res_p, holder of the position of solution
      **/
      void project(ResultPoint&);

      /**
        * @brief        convert image from 2D space to 3D space
      **/
      void load(cv::Mat&);

      /**
        * @brief        search for the solution in 3D space mesh
        * @return       true if there is solution, false otherwise
      **/
      bool find_flat_rect();

      /**
        * @brief        main function of the class
        * @param[in]    img, an input imgage with QR code in it
        * @param[out]   res_p, holder of the position of QR in the image
        * @return       true if there is an QR in the image, false otherwise
      **/
      bool process(cv::Mat&,ResultPoint&);
      
      /**
        * @brief        check if the area under the drone is free or not
      **/
      bool is_clear();

      /**
        * @brief        contruct mesh data and checks for free area
                        exactly under the drone
        * @return       true if the area under the drone is clear,
                        false otherwise
      **/
      bool is_clear_area(cv::Mat&);
  };


}


#endif // ALGORITHM_H

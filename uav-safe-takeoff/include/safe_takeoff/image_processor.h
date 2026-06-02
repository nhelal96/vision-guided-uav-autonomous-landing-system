/**
  * image_processor.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  grabs data from camera
**/


#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include "safe_takeoff/helper.h"

#include <librealsense2/rs.hpp>

#include <vector>

namespace SafeTakeoff{

  class ImageProcessor{
    std::string serial_no="043422251682";
    int i=0;
    int w,h;
    rs2::pipeline pipe;
    rs2::config cfg;

    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thr_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter hole;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    std::vector<rs2::filter> filters;

  public:
    ImageProcessor();

    /**
      * @return         2D vector array contains the distance data from camera
    **/
    std::vector<std::vector<double> > process();

    /**
      * @param[in]      realsense camera frame
      * @param[in]      x coordinate of the pixel
      * @param[in]      y coordinate of the pixel
      * @return         returns distance from the camera lense to a pixel
    **/
    float dist_3d(const rs2::depth_frame&,int,int);
  };

}

#endif //IMAGE_PROCESSOR_H

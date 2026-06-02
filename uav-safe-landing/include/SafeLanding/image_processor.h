/**
  * image_processor.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  grabs data from camera
**/


#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <librealsense2/rs.hpp>

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <vector>

namespace SafeLanding{

  class ImageProcessor{
    std::string serial_no="043422251112";
    int i=0;
    int w,h;
    cv::Mat color;
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
    cv::Mat process();
    cv::Mat get_color();
  };

}

#endif //IMAGE_PROCESSOR_H

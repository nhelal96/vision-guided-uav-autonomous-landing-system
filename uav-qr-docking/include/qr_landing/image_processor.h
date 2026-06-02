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

namespace QRLanding{

  class ImageProcessor{
    std::string serial_no="043422251112";
    rs2::pipeline pipe;
    rs2::config cfg;

  public:
    ImageProcessor();
    cv::Mat process();
  };

}

#endif //IMAGE_PROCESSOR_H

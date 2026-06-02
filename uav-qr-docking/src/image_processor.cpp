#include "qr_landing/image_processor.h"

#include <librealsense2/rs.hpp>

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

namespace QRLanding{


  ImageProcessor::ImageProcessor(){

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_device(serial_no);
    pipe.start(cfg);
  }

  cv::Mat ImageProcessor::process(){
    rs2::frameset data = pipe.wait_for_frames();
    rs2::frame color_frame = data.get_color_frame();
    if(!color_frame)return cv::Mat();

    int w = color_frame.as<rs2::video_frame>().get_width();
    int h = color_frame.as<rs2::video_frame>().get_height();

    cv::Mat img(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    return img;

  }


}

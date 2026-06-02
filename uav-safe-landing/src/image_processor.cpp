#include "SafeLanding/image_processor.h"
#include "SafeLanding/helper.h"

#include <librealsense2/rs.hpp>

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

namespace SafeLanding{


  ImageProcessor::ImageProcessor(){
    hole=rs2::hole_filling_filter(1);
    depth_to_disparity=rs2::disparity_transform(true);
    disparity_to_depth=rs2::disparity_transform(false);

    cfg.enable_stream(RS2_STREAM_DEPTH, XS, YS, RS2_FORMAT_Z16, 30);
    cfg.enable_device(serial_no);
    pipe.start(cfg);

    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    //filters.push_back(dec_filter);
    filters.push_back(thr_filter);
    filters.push_back(depth_to_disparity);
    filters.push_back(spat_filter);
    filters.push_back(temp_filter);
    filters.push_back(hole);
    filters.push_back(disparity_to_depth);

  }

  cv::Mat ImageProcessor::process(){
    rs2::frameset data = pipe.wait_for_frames();
    rs2::frame depth_frame = data.get_depth_frame();
    if (!depth_frame)
        return cv::Mat();

    rs2::frame filtered = depth_frame;

    for (auto& filter : filters)
        filtered = filter.process(filtered);

    w = filtered.as<rs2::video_frame>().get_width();
    h = filtered.as<rs2::video_frame>().get_height();
    cv::Mat img(cv::Size(w, h), CV_16UC1, (void*)filtered.get_data(), cv::Mat::AUTO_STEP);
    img.convertTo(img, CV_8UC1, 40 / 256.0);

    cv::imwrite("/home/isoft/safe_landing_test/d_imgs/"+std::to_string(i)+".jpg",img);
    i++;
    return img;
  }

  cv::Mat ImageProcessor::get_color(){return color;}


}

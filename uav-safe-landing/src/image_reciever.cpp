#include "SafeLanding/image_reciever.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>

namespace SafeLanding{


  ImageReciever::ImageReciever(ros::NodeHandle&nh_)
    :it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera_bot/depth/image_raw", 1,
      &ImageReciever::callback, this);
  }

  void ImageReciever::callback(const sensor_msgs::ImageConstPtr&msg){
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
         // 32-bit encoding is the default for depth maps (use it for deep learning and kinect)
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // fro gray scale images represent the depth map (realsens)
        // just use it if u will use realsense-ros-wrapper
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // for deep learning generated depth map
      /*
      double minVal, maxVal;
      cv::minMaxLoc(cv_ptr->image, &minVal, &maxVal,NULL,NULL);
      cv_ptr->image.convertTo(cv_ptr->image,CV_32FC1,256.0/(maxVal - minVal), -minVal * 256.0/(maxVal - minVal));
      cv::normalize(cv_ptr->image, cv_ptr->image, 0, 255, cv::NORM_MINMAX);
      */

     // for realsense, do not use any other preprocessing

      cv_ptr->image.convertTo(cv_ptr->image, CV_8UC1,40/256.);      
      cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(1280,720),cv::INTER_AREA);
      image=cv_ptr->image;
  }

  cv::Mat ImageReciever::process(){
    return image;
  }

}

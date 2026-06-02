#include "qr_landing/image_reciever.h"

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>

namespace QRLanding{


  ImageReciever::ImageReciever(ros::NodeHandle&nh_)
    :it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageReciever::callback, this);
  }

  void ImageReciever::callback(const sensor_msgs::ImageConstPtr&msg){
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv_ptr->image.convertTo(cv_ptr->image, CV_8UC1, 40 / 256.0);
      cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(1280,720),cv::INTER_AREA);
      image=cv_ptr->image;
      //cv::imwrite("/home/isoft-jetson/d_imgs/"+std::to_string(i)+".jpg",image);
      //i++;
  }

  cv::Mat ImageReciever::process(){
    return image;
  }

}

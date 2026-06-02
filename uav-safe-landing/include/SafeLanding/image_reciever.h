#ifndef IMAGE_RECIVER_H
#define IMAGE_RECIVER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace SafeLanding{

  class ImageReciever{
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat image;
    int i=0;

  public:
    ImageReciever(ros::NodeHandle&nh_);
    void callback(const sensor_msgs::ImageConstPtr&);
    cv::Mat process();
  };

}

#endif //IMAGE_RECIEVER_H

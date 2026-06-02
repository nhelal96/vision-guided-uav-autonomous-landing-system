#include "SafeLanding/algorithm.h"
//#include "SafeLanding/image_reciever.h"
#include "SafeLanding/image_processor.h"
#include "SafeLanding/helper.h"
#include "SafeLanding/communicator.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>

int main(int argc, char ** argv){

  ros::init(argc,argv,"SafeLanding_node",ros::init_options::NoSigintHandler);

  ros::NodeHandle nh; // main nodehandle

  SafeLanding::Algorithm algorithm_;
  SafeLanding::ImageProcessor image_processor_;
  //SafeLanding::ImageReciever image_reciever_(nh);
  SafeLanding::Communicator communicator_(nh);

  ROS_INFO("SafeLanding: SafeLanding node started.");

  bool start=false,search=true;
  SafeLanding::ResultPoint res_p;
  SafeLanding::Direction initial_pos,c_pos,dir;
  bool ret,align;
  ros::Duration dummy(1);
  ros::Rate rate(10);

  srand ( time(NULL) ); // initiallizing random gerator in dummy_planner
  while(ros::ok()){
      if(!communicator_.execute)continue; // work is finish

      if(!start){
        ROS_INFO("SafeLanding: listening...");
        communicator_.listen();
        start=true;
        ROS_INFO("SafeLanding: landing signal received, start SafeLanding...");
      }

      if(search){
          ROS_INFO("SafeLanding: searching...");
          initial_pos=communicator_.get_pos();
          align=true;

          while(search){
              auto img=image_processor_.process();
              ret=algorithm_.process(img,res_p);
              if(ret){
                  initial_pos=communicator_.get_pos();
                  dir=SafeLanding::compute_movement_distance(res_p,initial_pos);
                  search=false;
                  break;
              }

              c_pos=communicator_.get_pos();

              if(align){
                  dir=SafeLanding::random_planner(c_pos);
                  //dummy.sleep(); // for being sure that the drone has moved
              }

              align=SafeLanding::check_for_alignment(c_pos,dir);
              communicator_.process(dir,true);
              dummy.sleep();
              rate.sleep();
            }

            ROS_INFO("SafeLanding: found landing position...");
      }

      c_pos=communicator_.get_pos();
      align=SafeLanding::check_for_alignment(c_pos,dir);
      communicator_.process(dir,false);

      if(align){ // reached goal point
          // if this place is occuppied, continue searching
          auto img=image_processor_.process();
          ret=algorithm_.is_clear_area(img);
          if(!ret){
              ROS_INFO("SafeLanding: landing position occuppied...");
              search=true;
          }
          else communicator_.land();
      }

      ros::spinOnce();
      rate.sleep();
  }
}

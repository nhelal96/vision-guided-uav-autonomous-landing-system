#include "safe_takeoff/algorithm.h"
#include "safe_takeoff/image_processor.h"
#include "safe_takeoff/helper.h"
#include "safe_takeoff/communicator.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>

int main(int argc, char ** argv){

  ros::init(argc,argv,"safetakeoff_node",ros::init_options::NoSigintHandler);

  ros::NodeHandle nh; // main nodehandle

  SafeTakeoff::Algorithm algorithm_;
  SafeTakeoff::ImageProcessor image_processor_;
  SafeTakeoff::Communicator communicator_(nh);

  ROS_INFO("safe_takeoff: safetakeoff_node node started.");

  SafeTakeoff::ResultPoint res_p;
  SafeTakeoff::Direction initial_pos,c_pos,dir;
  auto rate=ros::Rate(10);

  bool ret,align;
  bool start=false,search=true;
  ros::Duration dummy(1);

  srand ( time(NULL) ); // initiallizing random gerator in dummy_planner
  while(ros::ok()){
      if(!communicator_.execute)continue; // work is finish

      if(!start){
        ROS_INFO("SafeTakeoff: listening...");
        communicator_.listen();
        start=true;
        ROS_INFO("SafeTakeoff: takeoff signal received, start SafeTakeoff...");
      }

      if(search){
          ROS_INFO("SafeTakeoff: searching...");
          initial_pos=communicator_.get_pos();
          align=true;

          while(search){
              auto grid=image_processor_.process();
              ret=algorithm_.process(grid,res_p);
              if(ret){
                  initial_pos=communicator_.get_pos();
                  dir=SafeTakeoff::compute_movement_distance(res_p,initial_pos);
                  search=false;
                  break;
              }

              c_pos=communicator_.get_pos();

              if(align){
                  dir=SafeTakeoff::random_planner(c_pos);
                  dummy.sleep(); // for being sure that the drone has moved
              }

              align=SafeTakeoff::check_for_alignment(c_pos,dir);
              communicator_.process(dir,true);
              dummy.sleep();
              ros::spinOnce();
              rate.sleep();
            }

            ROS_INFO("SafeTakeoff: found takeoff position...");
      }

      c_pos=communicator_.get_pos();
      align=SafeTakeoff::check_for_alignment(c_pos,dir);
      communicator_.process(dir,false);

      if(align){ // reached goal point
          // if this place is occuppied, continue searching
          auto grid=image_processor_.process();
          ret=algorithm_.is_clear_area(grid);
          ret&=SafeTakeoff::is_big_enough(res_p);

          if(!ret){
              ROS_INFO("SafeTakeoff: takeoff position occuppied...");
              search=true;
          }
          else communicator_.takeoff();
      }

      ros::spinOnce();
      rate.sleep();
  }

}

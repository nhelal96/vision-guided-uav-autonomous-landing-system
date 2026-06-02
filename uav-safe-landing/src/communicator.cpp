#include "SafeLanding/communicator.h"
#include "SafeLanding/helper.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

namespace SafeLanding{

  Communicator::Communicator(ros::NodeHandle&nh)
    :nh_(nh)
    {
    current_pos_=nh_.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10,
                &Communicator::pose_cb,this);
    safe_=nh_.subscribe<std_msgs::Bool>
                ("safelanding/fire",10,
                &Communicator::safe_cb,this);
    next_pos_=nh_.advertise<visualization_msgs::MarkerArray>
                ("input/goal_position", 10);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
    state_sub_ = nh_.subscribe<mavros_msgs::State>
                ("mavros/state", 10,
                &Communicator::state_cb,this);

    execute=true; // allow computations

  }

  void Communicator::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
      current_pose = *msg;
  }

  void Communicator::state_cb(const mavros_msgs::State::ConstPtr& msg){
      current_state = *msg;
  }

  void Communicator::safe_cb(const std_msgs::Bool::ConstPtr& msg){
      safe_flag=*msg;
  }

  void Communicator::land(){
    mavros_msgs::SetMode landing;
    landing.request.custom_mode = "AUTO.LAND";
    if( current_state.mode != "AUTO.LAND"){
        if( set_mode_client_.call(landing) &&
            landing.response.mode_sent)
              ROS_INFO("SafeLanding: successful landing");
        else ROS_INFO("SafeLanding: landing failed.");
      }

  }

  void Communicator::listen(){
     ros::Rate rate(2);
     while(1){
         if(safe_flag.data==true)
             return;
         ros::spinOnce();
         rate.sleep();
     }
 }

 void Communicator::publish(const double x, const double y, const double z,bool dummy){
     visualization_msgs::Marker pose;
     pose.pose.position.x=x;
     pose.pose.position.y=y;
     pose.pose.position.z=z;

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(pose);

     next_pos_.publish(arr);

     if(dummy)std::cout<<"\n--------------> dummy position <--------------\n";
     else std::cout<<"\n--------------> next position <--------------\n";
     std::cout<<"           ";
     std::cout<<"x= "<<pose.pose.position.x<<" , y= "<<pose.pose.position.y<<" , z= "<<pose.pose.position.z;
     std::cout<<std::endl<<std::endl;
 }

 Direction Communicator::get_pos(){
    Direction dir;
    dir.x=current_pose.pose.position.x;
    dir.y=current_pose.pose.position.y;

    std::cout<<"\n\n\n\n--------------> current position <--------------\n";
    std::cout<<"           ";
    std::cout<<"x= "<<dir.x<<" , y= "<<dir.y<<" , z= "<<current_pose.pose.position.z;
    std::cout<<std::endl<<std::endl;

    return dir;
 }

 void Communicator::process(Direction&dir,bool dummy){
    // reached goal point
    if(!dir.x && !dir.y)
        execute=false;

    if(execute)publish(dir.x,dir.y,1.,dummy);

}

}

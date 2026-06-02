#include "qr_landing/algorithm.h"
#include "qr_landing/communicator.h"
#include "qr_landing/helper.h"
#include "qr_landing/image_processor.h"

#include <ros/ros.h>


int main(int argc, char ** argv){
    ros::init(argc,argv,"qrlanding_node",ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    QRLanding::Algorithm algorithm_;
    QRLanding::ImageProcessor image_processor_;
    QRLanding::Communicator communicator_(nh);

    ROS_INFO("qrlanding_node: qrlanding_node node started.");

    bool start=false,search=true;
    QRLanding::ResultPoint res_p;
    QRLanding::Direction initial_pos,c_pos,dir;
    auto rate=ros::Rate(10);
    bool ret;
    ros::Duration dummy(0.5);

    while(ros::ok()){
        if(!communicator_.execute)continue;

        if(!start){
          ROS_INFO("qrlanding_node: listening...");
          communicator_.listen();
          start=true;
          ROS_INFO("qrlanding_node: landing signal received, start SafeLanding...");
        }

        if(search){
            ROS_INFO("qrlanding_node: searching...");
            while(search){
                auto img=image_processor_.process();
                ret=algorithm_.process(img,res_p);
                initial_pos=communicator_.get_pos();
                if(ret){
                  search=false;
                  break;
                }

                dir=QRLanding::random_planner();
                communicator_.process(dir,true);
                dummy.sleep();
                ros::spinOnce();
                rate.sleep();
              }
              ROS_INFO("qrlanding_node: found landing position...");
        }

        c_pos=communicator_.get_pos();
        dir=QRLanding::planner(res_p,initial_pos,c_pos);
        communicator_.process(dir,false);


        ros::spinOnce();
        rate.sleep();
    }
}

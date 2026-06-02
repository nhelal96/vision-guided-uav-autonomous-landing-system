/**
  * Communicator.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  communcating with controller via ros topics
**/

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include "SafeLanding/helper.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>


namespace SafeLanding{



  class Communicator{
    ros::NodeHandle nh_;
    ros::Publisher next_pos_;
    ros::Subscriber current_pos_;
    ros::Subscriber state_sub_;
    ros::Subscriber safe_;
    ros::ServiceClient set_mode_client_;
    geometry_msgs::PoseStamped current_pose;
    mavros_msgs::State current_state;
    std_msgs::Bool safe_flag;

    bool execute; // flag to allow or denie commputations

    public:

    /**
      * @param        main node handler of the node
    **/
    Communicator(ros::NodeHandle&);

    /**
      * @brief        set of callbacks for ros topics
    **/
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void state_cb(const mavros_msgs::State::ConstPtr&);
    void safe_cb(const std_msgs::Bool::ConstPtr&);


    /**
      * @brief        service function to make drone takeoff
    **/
    void land();

    /**
      * @brief        sleep function waits for start command
    **/
    void listen();

    /**
      * @brief        publishe goal position to controller
      * param[in]     x coordinate of the goal position
      * param[in]     y coordinate of the goal position
      * param[in]     z coordinate of the goal position
    **/
    void publish(const double , const double , const double, bool);

    /**
      * @brief        returns current position of the drone
    **/
    Direction get_pos();

    /**
      * @brief        main function for the class
      * @param[in]    dir, solution position
      * @param[in]    flag to indicate if it is a dummy position or not
    **/
    void process(Direction&,bool);

  };
}

#endif

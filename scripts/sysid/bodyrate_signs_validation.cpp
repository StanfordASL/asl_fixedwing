/**
    @file bodyrate_signs_validation.cpp
    Commands positive roll, pitch, or yaw rates to allow for a check on the 
    directions of the control surfaces.
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/CommandBool.h>
#include <iostream>
#include <Eigen/Dense>

#include <utils/plane.hpp>


int main(int argc, char **argv) {
    ros::init(argc, argv, "bodyrate_signcheck_node");
    ros::NodeHandle nh;
    
    ros::ServiceClient arm_clt = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
    ros::ServiceClient param_clt = nh.serviceClient<mavros_msgs::ParamSet>
                ("mavros/param/set");
    
    // Get from user which set of servos to check
    char input = 'z';
    std::string controls = "rpy";
    while (controls.find(input) == std::string::npos) {
        std::cout << "Select input: roll (r), pitch (p), yaw (y): ";
        std::cin >> input;
    }
    int c = controls.find(input);
    
    std::string CTRL_PARAM_PATH; // model name, gazebo or skywalker
    if (!nh.getParam("/bodyrate_signs_val/CTRL_PARAM_PATH", CTRL_PARAM_PATH)) {
        ROS_INFO("Need to define controller parameters directory path");
        exit(1);
    }
    
    Plane plane(nh, Plane::BODY_RATE, CTRL_PARAM_PATH);

    // Define rate for the node
    ros::Rate rate(10.0);

    // Turn off auto-disarm before takeoff so sysid can be performed on the ground
    mavros_msgs::ParamSet dsrm_off;
    dsrm_off.request.param_id = "COM_DISARM_PRFLT";
    dsrm_off.request.value.real = -1.;
    param_clt.call(dsrm_off);
    if (dsrm_off.response.success) ROS_INFO("Disabled auto-disarm before takeoff");

    // Automatically arm the vehicle
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arm_clt.call(arm_cmd);
    if (arm_cmd.response.success) ROS_INFO("Vehicle armed");

    while(ros::ok() && !plane.px4_connected()) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Set PX4 to offboard mode to apply command");
    Eigen::Vector4d u; // u = [T, p, q, r]
    u.setZero();
    u(c+1) = 1.0; 
    while(ros::ok()) {
        ros::spinOnce();
        
        plane.send_control(u);
        if (plane.px4_mode() == "OFFBOARD") {
            ROS_INFO("Sending commands for a positive rate");
        }

        rate.sleep(); // sleep for remaining time
    }
}

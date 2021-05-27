/**
	@file rompc_node.cpp
	ROS node for running ROMPC controller.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <rompc/rompc_utils.hpp>
#include <utils/plane.hpp>
#include <utils/rotation.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv) {
	ros::init(argc, argv, "rompc_node");
	ros::NodeHandle nh;
    
    // Access parameters
    double T_RESET = 5.0; // seconds between resetting controller before activated
    if (!nh.getParam("/rompc_node/T_RESET", T_RESET)) {
        ROS_INFO("Using default reset time of 5 seconds");
    }

    double CTRL_RATE = 200.0; // Hz controller frequency
    if (!nh.getParam("/rompc_node/CTRL_RATE", CTRL_RATE)) {
        ROS_INFO("Using default control rate of 200 Hz");
    }

    std::string CTRL_PATH; // model name, gazebo or skywalker
    if (!nh.getParam("/rompc_node/CTRL_PATH", CTRL_PATH)) {
        ROS_INFO("Need to define controller parameters directory path");
        exit(1);
    }

    std::string ctrl_type; // control type, body_rate or ctrl_surf
    if (!nh.getParam("/rompc_node/CTRL_TYPE", ctrl_type)) {
        ROS_INFO("Need to define controller type {body_rate or ctrl_surf}");
        exit(1);
    }
    unsigned CTRL_TYPE;
    if (ctrl_type.compare("body_rate") == 0) {
        CTRL_TYPE = Plane::BODY_RATE;
        ROS_INFO("Control type set to BODY_RATE");
    }
    else if (ctrl_type.compare("ctrl_surf") == 0) {
        CTRL_TYPE = Plane::CTRL_SURF;
        ROS_INFO("Control type set to CTRL_SURF");
    }
    else {
        ROS_INFO("CTRL_TYPE must be body_rate or ctrl_surf, got %s", ctrl_type.c_str());
    }

    std::string MODEL; // model name, gazebo or skywalker
    if (!nh.getParam("/rompc_node/MODEL", MODEL)) {
        ROS_INFO("Need to define model name {gazebo or skywalker}");
        exit(1);
    }
    std::string filepath = CTRL_PATH + MODEL;
    ROS_INFO("Loading controller parameters from %s", filepath.c_str());

    // Define Plane object
    Plane plane(nh, CTRL_TYPE, filepath, true);


	// Define publishers
	ros::Publisher att_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>
				("mavros/setpoint_raw/attitude", 10);
	ros::Publisher act_cmd_pub = nh.advertise<mavros_msgs::ActuatorControl>
				("mavros/actuator_control", 10);


	// Initialize controller
	//ROMPC ctrl(filepath);
	// ctrl.init();

	// Define rate for the node
	ros::Rate rate(CTRL_RATE);

	mavros_msgs::ActuatorControl act_cmd;
	act_cmd.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;

	// Wait for FCU connection
	while(ros::ok() && !plane.px4_connected()) {
		ros::spinOnce();
		rate.sleep();
	}

	// Main loop, until ROS shutdown
	double t0 = ros::Time::now().toSec();
	double t = t0;
	double t_last_reset = t0;
	double dt;
	Eigen::VectorXd u_nrmlzd;
	
    // Temporary publishing messages
	while(ros::ok()) {
		// Each call to spinOnce will result in subscriber callbacks
		ros::spinOnce();

		// Need to do some time checking on subscribed data to make sure its relevant?
		dt = ros::Time::now().toSec() - t;
		t = ros::Time::now().toSec();

		// Run diagnostics to ensure ROMPC still good
		// TODO

		// Get control
		//ctrl.update(y);
		
		// Convert control into normalized units
		//u_nrmlzd = Plane::normalize_control(ctrl.get_u());
		
		//if (px4_mode != "OFFBOARD" && t - t_last_reset > T_RESET) {
			//ctrl.init(x0);
			//ROS_INFO("Reinitializing ROMPC controller");
			//t_last_reset = t;
		//}

		// Add a check on ROMPC controller failure and
		// have backup be a loiter or something using a change mode
		act_cmd_pub.publish(act_cmd);
		rate.sleep(); // sleep for remaining time
	}
}

/*
	@file rompc_node.cpp
	ROS node for running ROMPC controller.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <utils/plane.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "rompc_node");
	ros::NodeHandle nh;
    
    // Access parameters
    double T_RESET = 5.0; // seconds between resetting controller before activated
    if (!nh.getParam("/rompc_node/T_RESET", T_RESET)) {
        ROS_INFO("Using default reset time of 5 seconds");
    }

    double CTRL_RATE = 100.0; // Hz controller frequency
    if (!nh.getParam("/rompc_node/CTRL_RATE", CTRL_RATE)) {
        ROS_INFO("Using default control rate of 100 Hz");
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
    }
    else if (ctrl_type.compare("ctrl_surf") == 0) {
        CTRL_TYPE = Plane::CTRL_SURF;
    }
    else {
        ROS_INFO("CTRL_TYPE must be body_rate or ctrl_surf, got %s", ctrl_type.c_str());
    }

    std::string target_type; // target type, SGF, SLF or STF
    if (!nh.getParam("/rompc_node/TARGET_TYPE", target_type)) {
        ROS_INFO("Need to define target type {sgf, slf, or stf}");
        exit(1);
    }
    unsigned TARGET_TYPE;
    if (target_type.compare("sgf") == 0) {
        TARGET_TYPE = ROMPC::SGF;
    }
    else if (target_type.compare("slf") == 0) {
        TARGET_TYPE = ROMPC::SLF;
    }
    else if (target_type.compare("stf") == 0) {
        TARGET_TYPE = ROMPC::STF;
    }
    else {
        ROS_INFO("TARGET_TYPE must be sgf, slf, or stf, got %s", target_type.c_str());
    }

    std::string model_type; // model type
    if (!nh.getParam("/rompc_node/MODEL_TYPE", model_type)) {
        ROS_INFO("Need to define model type {standard or cfd}");
        exit(1);
    }
    unsigned MODEL_TYPE;
    if (model_type.compare("standard") == 0) {
        MODEL_TYPE = ROMPC::STANDARD;
    }
    else if (model_type.compare("cfd") == 0) {
        MODEL_TYPE = ROMPC::CFD;
    }
    else {
        ROS_INFO("MODEL_TYPE must be standard or cfd, got %s", model_type.c_str());
    }

    std::string MODEL; // model name, gazebo or skywalker
    if (!nh.getParam("/rompc_node/MODEL", MODEL)) {
        ROS_INFO("Need to define model name {gazebo or skywalker}");
        exit(1);
    }
    std::string filepath = CTRL_PATH + MODEL + "/" + target_type;
    ROS_INFO("Loading controller parameters from %s", filepath.c_str());

    // Initialize Plane object
    Plane plane(nh, CTRL_TYPE, filepath);

	// Initialize controller
	ROMPC ctrl(nh, CTRL_TYPE, TARGET_TYPE, MODEL_TYPE, filepath);

	// Define rate for the node
	ros::Rate rate(CTRL_RATE);

	// Wait for FCU connection
	while(ros::ok() && !plane.px4_connected()) {
		ros::spinOnce();
		rate.sleep();
	}
    ROS_INFO("PX4 connection established");

	// Main loop, until ROS shutdown
	double t0 = ros::Time::now().toSec();
	double t = t0;
	double t_last_reset = t0;
	double dt;
	

	while(ros::ok()) {
		// Each call to spinOnce will result in subscriber callbacks
		ros::spinOnce();
		t = ros::Time::now().toSec();

        // Get current measurements
        Eigen::Vector3d pos = plane.get_pos();
        Eigen::Vector3d vel = plane.get_vel();
        Eigen::Vector3d euler = plane.get_euler();
        Eigen::Vector3d om = plane.get_bodyrate();
        double thrust = plane.get_thrust();
		Eigen::Vector3d ctrl_srf = plane.get_ctrl_srf();

        // If offboard mode not yet set, periodically reset controller
		if (plane.px4_mode() != "OFFBOARD" && t - t_last_reset > T_RESET) {
			ctrl.init(t, pos, euler(2));
			ROS_INFO("Reinitializing ROMPC controller");
			t_last_reset = t;
		}
		
        // Update controller wih new measurement information
        ctrl.update(t, pos, vel, euler, om, thrust, ctrl_srf);
        
        // Send control
        plane.send_control(ctrl.get_ctrl());

		// Add a check on ROMPC controller failure and
		// have backup be a loiter or something using a change mode
		rate.sleep(); // sleep for remaining time
	}
}

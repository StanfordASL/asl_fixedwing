/**
	@file rompc_node.cpp
	ROS node for running ROMPC controller.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <rompc/rompc_utils.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>

// Callback function for the MAVROS State message
bool px4_connected; // PX4 connection exists
bool px4_armed; // PX4 armed
std::string px4_mode; // PX4 flight mode

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	px4_connected = msg->connected;
	px4_armed = msg->armed;
	px4_mode = msg->mode;
}

void ctrl_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg) {}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rompc_node");
	ros::NodeHandle nh;

	// Define subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
				("mavros/state", 10, state_cb);
	ros::Subscriber ctrl_sub = nh.subscribe<mavros_msgs::ActuatorControl>
				("mavros/target_actuator_control", 10, ctrl_cb); // pre-mixer actuator commands
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
				("mavros/local_position/pose", 10, pose_cb);
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
				("mavros/local_position/velocity", 10, twist_cb);

	// Define publishers
	ros::Publisher att_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>
				("mavros/setpoint_raw/attitude", 10);
	ros::Publisher act_cmd_pub = nh.advertise<mavros_msgs::ActuatorControl>
				("mavros/actuator_control", 10);

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

	std::string MODEL; // model name, gazebo or skywalker
	if (!nh.getParam("/rompc_node/MODEL", MODEL)) {
		ROS_INFO("Need to define model name");
		exit(1);
	}
	std::string filepath = CTRL_PATH + MODEL;
	ROS_INFO("Loading controller parameters from %s", filepath.c_str());

	// Initialize controller
	//ROMPC ctrl(filepath);
	// ctrl.init();

	// Define rate for the node
	ros::Rate rate(CTRL_RATE);

	mavros_msgs::ActuatorControl act_cmd;
	act_cmd.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;

	// Wait for FCU connection
	while(ros::ok() && !px4_connected) {
		ros::spinOnce();
		rate.sleep();
	}

	// Main loop, until ROS shutdown
	double t0 = ros::Time::now().toSec();
	double t = t0;
	double t_last_reset = t0;
	double dt;
	Eigen::VectorXd u_nrmlzd;
	
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
		
		if (px4_mode != "OFFBOARD" && t - t_last_reset > T_RESET) {
			//ctrl.init(x0);
			ROS_INFO("Reinitializing ROMPC controller");
			t_last_reset = t;
		}


		// Add a check on ROMPC controller failure and
		// have backup be a loiter or something using a change mode
		act_cmd_pub.publish(act_cmd);
		rate.sleep(); // sleep for remaining time
	}
}

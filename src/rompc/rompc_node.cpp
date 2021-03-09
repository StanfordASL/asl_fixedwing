/**
	@file node.cpp
	ROS node for running ROMPC controller.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <utils.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <string>

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

int main(int argc, char **argv) {
	ros::init(argc, argv, "rompc_node");
	ros::NodeHandle nh;

	// Define subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
				("mavros/state", 10, state_cb);

	// Subscribe to actuator messages from PX4
	ros::Subscriber ctrl_sub = nh.subscribe<mavros_msgs::ActuatorControl>
				("mavros/target_actuator_control", 10, ctrl_cb);

	// Define publishers
	ros::Publisher att_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>
				("mavros/setpoint_raw/attitude", 10);

	// Access parameters
	double T_RESET = 5.0; // seconds between resetting controller before activated
	if (!nh.getParam("/rompc_node/T_RESET", T_RESET)) {
		ROS_INFO("Using default reset time of 5 seconds");
	}

	double CTRL_RATE = 200.0; // Hz controller frequency
	if (!nh.getParam("/rompc_node/CTRL_RATE", CTRL_RATE)) {
		ROS_INFO("Using default control rate of 200 Hz");
	}

	// Define rate for the node
	ros::Rate rate(CTRL_RATE);

	mavros_msgs::AttitudeTarget att_cmd;
	att_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
	att_cmd.body_rate.x = 0.0;
	att_cmd.body_rate.y = 0.0;
	att_cmd.body_rate.z = 0.0;
	att_cmd.thrust = 0.0;

	// Test
	Eigen::MatrixXd test;
	test.setIdentity(3,3);
	Utils::save_matrix("test.csv", test);
	
	Eigen::MatrixXd test2;
	test2 = Utils::load_matrix("test.csv");
	Utils::save_matrix("test2.csv", test2);

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
	
	// Initialize controller
	//ROMPC ctrl(filename);
	//ctrl.init();

	
	while(ros::ok()) {
		// Each call to spinOnce will result in subscriber callbacks
		ros::spinOnce();

		// Need to do some time checking on subscribed data to make sure its relevant?
		dt = ros::Time::now().toSec() - t;
		t = ros::Time::now().toSec();

		// Run diagnostics to ensure ROMPC still good
		// TODO

		// Get control
		//u = ctrl.control(y);

		if (px4_mode != "OFFBOARD" && t - t_last_reset > T_RESET) {
			//ctrl.init(x0);
			ROS_INFO("Reinitializing ROMPC controller");
			t_last_reset = t;
		}


		// Add a check on ROMPC controller failure and
		// have backup be a loiter or something using a change mode
		att_cmd_pub.publish(att_cmd);
		rate.sleep(); // sleep for remaining time
	}
}

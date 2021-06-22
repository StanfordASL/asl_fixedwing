/**
    @file sysid_actuator_skywalker.cpp
    ROS node for collecting data for sysid to determine
    an appropriate scaling for thrust and body angular
    acceleration commands over ActuatorControl messages.
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/RCOut.h>
#include <iostream>

bool px4_connected; // PX4 connection exists
std::string px4_mode; // PX4 mode
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    px4_connected = msg->connected;
    px4_mode = msg->mode;
}

unsigned pwm;
void rcout_cb(const mavros_msgs::RCOut::ConstPtr& msg) {
    pwm = msg->channels[0];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "thrust_sysid_node");
    ros::NodeHandle nh;

    // Get parameters
    double STEP_SIZE = 0.1;
    if (!nh.getParam("/sysid_actuator_ctrl/STEP_SIZE", STEP_SIZE)) {
        ROS_INFO("Using default step size of 0.1");
    }

    double HOLD_TIME = 10.0;
    if (!nh.getParam("/sysid_actuator_ctrl/HOLD_TIME", HOLD_TIME)) {
        ROS_INFO("Using default hold time of 10 seconds");
    }
    HOLD_TIME = std::max(HOLD_TIME, 0.1);

    // Define subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, state_cb);
    ros::Subscriber rcout_sub = nh.subscribe<mavros_msgs::RCOut>
                ("mavros/rc/out", 10, rcout_cb);

    // Define publishers
    ros::Publisher act_cmd_pub = nh.advertise<mavros_msgs::ActuatorControl>
                ("mavros/actuator_control", 10);

    // Define services
    ros::ServiceClient arm_clt = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
    ros::ServiceClient param_clt = nh.serviceClient<mavros_msgs::ParamSet>
                ("mavros/param/set");
    
    // Get from user which set of servos to check
    char input = 'z';
    std::string controls = "rpyt";
    while (controls.find(input) == std::string::npos) {
        std::cout << "Select input: roll (r), pitch (p), yaw (y), thrust (t): ";
        std::cin >> input;
    }
    int c = controls.find(input);
    
    char sign = '.';
    std::string signs = "+-";
    while (signs.find(sign) == std::string::npos) {
        std::cout << "Select direction: positive (+), negative (-): ";
        std::cin >> sign;
    }
    double s;
    if (sign == '+') s = 1.0;
    else s = -1.0;

    // Define rate for the node
    ros::Rate rate(50.0);

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

    // Define the actuator control message
    mavros_msgs::ActuatorControl act_cmd;
    act_cmd.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;

    while(ros::ok() && !px4_connected) {
        ros::spinOnce();
        rate.sleep();
    }

    int hold_steps = HOLD_TIME/0.1;
    int inc_steps = 1.0/STEP_SIZE;
    int h = 0;
    int i = 0;
    std::cout << "Set PX4 to offboard mode to start sequence" << std::endl;
    while(ros::ok() && i <= inc_steps) {
        ros::spinOnce();

        act_cmd_pub.publish(act_cmd);

        if (px4_mode == "OFFBOARD") {
        
            // Switch the level every hold_steps number of steps
            if (h == hold_steps) {
                act_cmd.controls[c] += s*STEP_SIZE;
                h = 0;
                i++;

                // Print the command
                if (i <= inc_steps) {
                    std::cout << "act_cmd.controls[" << input << "] = "
                                << act_cmd.controls[c] << std::endl;
                }
            }

            h++;

        }

        rate.sleep(); // sleep for remaining time
    }
    act_cmd.controls[c] = 0.0;
    act_cmd_pub.publish(act_cmd);
    ROS_INFO("Done with open-loop inputs, shutting down node");
    ros::shutdown();
}

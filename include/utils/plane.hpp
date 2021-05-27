#ifndef PLANE_HPP
#define PLANE_HPP

/**
    @file plane.hpp
    Defines a Plane class that contains some useful utilities.

	@brief Define a plane class that provides some useful tools
	for controller design, such as mappings from commands to
	actuator inputs.
*/

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>

using uvec = Eigen::Vector4d;

class Plane {
private:
    double nrmlz_ctrl_srf_cmd(const double delta, const int i);
    double nrmlz_thrust_cmd(const double T, const double V);

	// Parameters for scaling control surface deflection commands
	Eigen::Vector3d _delta_0; // in order of [a, e, r]
	Eigen::Vector3d _c_delta; // in order of [a, e, r]

    // Parameters for thrust command normalization
	double _c_T0;
	double _c_TVom;
    
    // Specifies either BODY_RATE or CTRL_SURF
    unsigned _ctrl_type;

    // ROS stuff
    ros::Subscriber _state_sub;
    ros::Subscriber _pose_sub;
    ros::Subscriber _twist_sub;
    ros::Publisher _euler_pub;

    // Aircraft state information
    Eigen::Vector3d _p_b_in_i; // position of body frame in inertial coordinates
    Eigen::Vector4d _q_i_to_b; // quaternion describing rotation from inertial NED to body FRD frame
    Eigen::Vector3d _euler; // euler angles
    const Eigen::Matrix3d _R_ENU_TO_NED = (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                                                         1.0, 0.0, 0.0,
                                                         0.0, 0.0, -1.0).finished();
    const Eigen::Vector4d _Q_NED_TO_ENU = (Eigen::Vector4d() << 0.0, -0.5*sqrt(2.0), -0.5*sqrt(2.0), 0.0).finished();
    const Eigen::Vector4d _Q_FLU_TO_FRD = (Eigen::Vector4d() << 0.0, 1.0, 0.0, 0.0).finished();
    
    // PX4 state information
    bool _px4_connected = false; // PX4 connection exists
    bool _px4_armed = false; // PX4 armed
    std::string _px4_mode; // PX4 flight mode


public:
    Plane(ros::NodeHandle& nh, const unsigned ctrl_type, const std::string& filepath);
    void normalize_control(const uvec& u, uvec& u_nrmlzd, const double V);
    
    // MAVROS related functions
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    
    // Accessing useful information
    Eigen::Vector3d get_pos();
    Eigen::Vector3d get_euler();
    bool px4_connected();
    bool px4_armed();

    // Control modes
    static const unsigned BODY_RATE = 0;
    static const unsigned CTRL_SURF = 1;
};

#endif // PLANE_HPP

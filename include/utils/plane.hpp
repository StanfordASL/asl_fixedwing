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
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>

using Vec4 = Eigen::Vector4d;
using Vec3 = Eigen::Vector3d;

class Plane {
public:
    // Constructor
    Plane(ros::NodeHandle& nh, const unsigned ctrl_type, 
          const std::string& filepath, bool debug = false);
    
    // Send control commands
    void send_control(const Vec4& u);
    
    // MAVROS related functions
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void act_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg);

    // Accessing useful information
    Vec3 get_pos();
    Vec3 get_euler();
    Vec3 get_vel();
    Vec3 get_bodyrate();
    double get_thrust();
    Vec3 get_ctrl_srf();
    bool px4_connected();
    bool px4_armed();
    std::string px4_mode();

    // Control modes
    static const unsigned BODY_RATE = 0;
    static const unsigned CTRL_SURF = 1;

private:
    // Control input helpers
    double nrmlz_ctrl_srf_cmd(const double delta, const int i);
    double nrmlz_thrust_cmd(const double T, const double V);
    void normalize_control(const Vec4& u, Vec4& u_nrmlzd, const double V);
    
    double dim_ctrl_srf_cmd(const double u_d, const int i);
    double dim_thrust_cmd(const double u_T, const double V);

	// Use debug to publish some internal ROS topics
    bool _debug;
    
    // Parameters for scaling control surface deflection commands
	Vec3 _delta_0; // in order of [a, e, r]
	Vec3 _c_delta; // in order of [a, e, r]

    // Parameters for thrust command normalization
	double _c_T0;
	double _c_TVom;
    
    // Specifies either BODY_RATE or CTRL_SURF
    unsigned _ctrl_type;

    // ROS stuff
    ros::Subscriber _state_sub;
    ros::Subscriber _pose_sub;
    ros::Subscriber _twist_sub;
    ros::Subscriber _act_sub;
    ros::Publisher _pos_pub;
    ros::Publisher _vel_pub;
    ros::Publisher _euler_pub;
    ros::Publisher _bodyrate_pub;
    ros::Publisher _ctrl_pub; // sends commands to PX4
    ros::Publisher _act_pub; // publishes dimensionalized actuator values

    // Aircraft state information
    Vec3 _p_b_i_I; // body pos relative to inertial in I coord.
    Vec3 _v_b_I_B; // body vel as seen by I in B coord.
    Vec4 _q_I_to_B; // quat rotation from inertial NED to body FRD frame
    Vec3 _euler; // euler angles (roll, pitch, yaw) (phi, theta, psi)
    Vec3 _om_B_I_B; // ang vel (p, q, r) of B w.r.t I frame, in B coordinates
    const Vec4 _Q_NED_TO_ENU = (Vec4() << 0.0, -0.5*sqrt(2.0), -0.5*sqrt(2.0), 0.0).finished();
    const Vec4 _Q_FLU_TO_FRD = (Vec4() << 0.0, 1.0, 0.0, 0.0).finished();
    double _thrust;
    Vec3 _ctrl_srf;

    // PX4 state information
    bool _px4_connected = false; // PX4 connection exists
    bool _px4_armed = false; // PX4 armed
    std::string _px4_mode; // PX4 flight mode
};

#endif // PLANE_HPP

/** 
    @file plane.cpp
    Definition of Plane class to provide useful utilities. This plane
    class assumes the control inputs are the thrust commands and
    the either the control surface deflection commands or body rate commands.
*/

#include <utils/plane.hpp>
#include <utils/data.hpp>
#include <utils/rotation.hpp>
#include <utils/utils.hpp>

/** 
	@brief Constructor which loads parameters from csv file.

    @param[in] ctrl_type   control type, either Plane::BODY_RATE 
                           or Plane::CTRL_SURF
	@param[in] filepath    to ctrl_params.csv file, which should 
                           store in the order: [c_T0, c_TVom, a_0, 
                           c_a, e_0, c_e, r_0, c_r]

*/
Plane::Plane(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const std::string& filepath, bool debug)
             : _ctrl_type(ctrl_type), _debug(debug) {
    // Check ctrl_type is valid
    if (_ctrl_type != BODY_RATE and _ctrl_type != CTRL_SURF) {
        throw "Control type must be either BODY_RATE or CTRL_SURF";
    }

	// Load relevant control parameters from csv file
	Eigen::VectorXd ctrl_params = Data::load_vector(filepath + "/ctrl_params.csv");
    _c_T0 = ctrl_params(0);
    _c_TVom = ctrl_params(1);
    if (_ctrl_type == CTRL_SURF) {
	    _delta_0(0) = ctrl_params(2); // a_0
	    _delta_0(1) = ctrl_params(4); // e_0
	    _delta_0(2) = ctrl_params(6); // r_0
	    _c_delta(0) = ctrl_params(3); // c_a
	    _c_delta(1) = ctrl_params(5); // c_e
	    _c_delta(2) = ctrl_params(7); // c_r
    }

    // Initialize variables
    _p_b_i_I.setZero();
    _v_b_I_B.setZero();
    _q_I_to_B.setZero();
    _euler.setZero();
    _om_B_I_B.setZero();
    
    // Define MAVROS subscribers
    _state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 1, &Plane::state_cb, this);
    _pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 1, &Plane::pose_cb, this);
    _twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_body", 1, &Plane::twist_cb, this);
    
    // Define MAVROS publisher
    if (_ctrl_type == CTRL_SURF) {
        _ctrl_pub = nh.advertise<mavros_msgs::ActuatorControl>
                ("mavros/actuator_control", 1);
        ROS_INFO("Control type set to CTRL_SURF");
    }
    else {
        _ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                ("mavros/setpoint_raw/attitude", 1);
        ROS_INFO("Control type set to BODY_RATE");
    }

    if (_debug) {
        _pos_pub = nh.advertise<geometry_msgs::Point>
                    ("plane/position", 10);
        _vel_pub = nh.advertise<geometry_msgs::Point>
                    ("plane/velocity", 10);
        _euler_pub = nh.advertise<geometry_msgs::Point>
                    ("plane/euler", 10);
        _bodyrate_pub = nh.advertise<geometry_msgs::Point>
                    ("plane/bodyrate", 10);
    }
}

/**
    @brief Sends control u to PX4 using MAVROS. The control u is defined:
    
    u = [T, a, e, r] in [N, rad, rad, rad] if _ctrl_type = CTRL_SURF  
    u = [T, p, q, r] in [N, rad/s, rad/s, rad/s] if _ctrl_type = BODY_RATE
    
    @param[in] u   dimensional control vector
*/
void Plane::send_control(const Vec4& u) {
    Vec4 u_nrmlzd;
    Plane::normalize_control(u, u_nrmlzd, _v_b_I_B(0));

    if (_ctrl_type == CTRL_SURF) {
        mavros_msgs::ActuatorControl cmd;
        cmd.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
        cmd.controls[0] = u_nrmlzd(1);
        cmd.controls[1] = u_nrmlzd(2);
        cmd.controls[2] = u_nrmlzd(3);
        cmd.controls[3] = u_nrmlzd(0);
        _ctrl_pub.publish(cmd);
    }
    else {
        mavros_msgs::AttitudeTarget cmd;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        cmd.body_rate.x = u_nrmlzd(1);
        cmd.body_rate.y = u_nrmlzd(2);
        cmd.body_rate.z = u_nrmlzd(3);
        cmd.thrust = u_nrmlzd(0);
        _ctrl_pub.publish(cmd);
    }
}

/**
	@brief Converts the full control u = [T, a, e, r] or u = [T, p, q, r] 
    given in unit [N, rad, rad, rad] or [N, rad/s, rad/s, rad/s] to 
    normalized quantities that are commanded through MAVROS.

	@param[in] u         dimensional control vector
	@param[in] u_nrmlzd  normalized commands
	@param[in] V         rel. airspeed parallel to body x axis [m/s]
*/
void Plane::normalize_control(const Vec4& u, Vec4& u_nrmlzd, const double V) {
    u_nrmlzd(0) = Plane::nrmlz_thrust_cmd(u(0), V);
    if (_ctrl_type == CTRL_SURF) { 
	    u_nrmlzd(1) = Plane::nrmlz_ctrl_srf_cmd(u(1), 0);
	    u_nrmlzd(2) = Plane::nrmlz_ctrl_srf_cmd(u(2), 1);
	    u_nrmlzd(3) = Plane::nrmlz_ctrl_srf_cmd(u(3), 2);
    }
    else {
        u_nrmlzd(1) = u(1);
        u_nrmlzd(2) = u(2);
        u_nrmlzd(3) = u(3);
    }
}

/**
	@brief Converts a commanded control surface deflection angle
	into a normalized [-1,1] command that can be sent over
	actuator_control MAVROS topics. Assumes a linear mapping.

	@param[in]  cmd  control surface deflection cmd [radians]
	@param[in]  i    index int 0=a, 1=e, 2=r
	@param[out] normalized command in range [-1, 1]
*/
double Plane::nrmlz_ctrl_srf_cmd(const double delta, const int i) {
	return std::min(1.0, std::max(-1.0, (delta - _delta_0(i))/_c_delta(i) ));
}

/**
    @brief Converts a commanded thrust value into a normalized 
	[0,1] command that can be sent over actuator_control MAVROS 
	topics. Assumes a quadratic mapping from cmd to T.

    @param[in]  cmd  thrust command [N]
	@param[in]  V    rel. windspeed parallel to body x axis [m/s]
    @param[out] normalized command in range [0, 1]
*/
double Plane::nrmlz_thrust_cmd(const double T, const double V) {
    return std::min(1.0, std::max(0.0, 0.5*(_c_TVom*V + 
				sqrt( (_c_TVom*V)*(_c_TVom*V) + 4.0*T/_c_T0 )) ));
}

/**
    @brief Callback for extracting information from MAVROS
    local_position/pose messages. These messages provide
    the filtered local position in ENU coordinates where the origin
    of the frame is where the autopilot was booted up. Also provides
    the quaternion of the aircraft body frame w.r.t ENU frame

    @param[in] msg  message from /mavros/local_position/pose
*/
void Plane::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    _p_b_i_I(0) = msg->pose.position.y;
    _p_b_i_I(1) = msg->pose.position.x;
    _p_b_i_I(2) = -msg->pose.position.z;

    // Transform the quaternion from ENU to ENU to NED to FRD
    Vec4 q_ENU_to_FLU;
    Vec4 q_NED_to_FLU;
    Utils::quat_to_eigen4d(msg->pose.orientation, q_ENU_to_FLU);
    Rot::compose_quats(_Q_NED_TO_ENU, q_ENU_to_FLU, q_NED_to_FLU);
    Rot::compose_quats(q_NED_to_FLU, _Q_FLU_TO_FRD, _q_I_to_B);

    // Compute Euler angles from quaternion
    Rot::quat_to_euler(_q_I_to_B, _euler);

    if (_debug) {
        // Publish position
        geometry_msgs::Point pos;
        Utils::eigen3d_to_point(_p_b_i_I, pos);
        _pos_pub.publish(pos);

        // Publish euler angles
        geometry_msgs::Point euler;
        euler.x = Rot::rad_to_deg(_euler(0)); 
        euler.y = Rot::rad_to_deg(_euler(1)); 
        euler.z = Rot::rad_to_deg(_euler(2));
        _euler_pub.publish(euler);
    }
}

/**
    @brief Callback for extracting information from MAVROS
    local_position/velocity messages. These messages provide velocity
    of body w.r.t inertial in body FLU frame and body angular velocity in FLU.

    @param[in] msg  message from /mavros/local_position/velocity_body
*/
void Plane::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // Convert both from FLU to FRD
    _v_b_I_B(0) = msg->twist.linear.x;
    _v_b_I_B(1) = -msg->twist.linear.y;
    _v_b_I_B(2) = -msg->twist.linear.z;

    _om_B_I_B(0) = msg->twist.angular.x;
    _om_B_I_B(1) = -msg->twist.angular.y;
    _om_B_I_B(2) = -msg->twist.angular.z;

    if (_debug) {
        // Publish velocity
        geometry_msgs::Point vel;
        Utils::eigen3d_to_point(_v_b_I_B, vel);
        _vel_pub.publish(vel);

        // Publish body rates
        geometry_msgs::Point om;
        om.x = Rot::rad_to_deg(_om_B_I_B(0));
        om.y = Rot::rad_to_deg(_om_B_I_B(1));
        om.z = Rot::rad_to_deg(_om_B_I_B(2));
        _bodyrate_pub.publish(om);
    }
}

/**
    @brief Callback for extracting PX4 status from MAVROS.

    @param[in] msg  message from /mavros/state
*/
void Plane::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    _px4_connected = msg->connected;
    _px4_armed = msg->armed;
    _px4_mode = msg->mode;
}

/**
    @brief Returns aircraft position in meters in local inertial NED frame coordinates.
*/
Vec3 Plane::get_pos() {
    return _p_b_i_I;
}

/**
    @brief Returns aircraft euler angles (roll, pitch, yaw) in rad with respect
    to local inertial NED frame.
*/
Vec3 Plane::get_euler() {
    return _euler;
}

/**
    @brief Returns aircraft velocity in m/s w.r.t local inertial frame in body frame coordinates.
*/
Vec3 Plane::get_vel() {
    return _v_b_I_B;
}

/**
    @brief Returns aircraft body rates (p, q, r) in rad/s.
*/
Vec3 Plane::get_bodyrate() {
    return _om_B_I_B;
}


/**
    @brief Returns true if PX4 is connected
*/
bool Plane::px4_connected() {
    return _px4_connected;
}

/**
    @brief Returns true if PX4 is armed
*/
bool Plane::px4_armed() {
    return _px4_armed;
}

/**
    @brief Returns PX4 mode string, i.e. OFFBOARD
*/
std::string Plane::px4_mode() {
    return _px4_mode;
}


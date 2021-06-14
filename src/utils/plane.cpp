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

#include <geometry_msgs/PointStamped.h>

/** 
	@brief Constructor which loads parameters from csv file.
    
    @param[in] nh          ROS nodehandle
    @param[in] ctrl_type   control type, either Plane::BODY_RATE 
                           or Plane::CTRL_SURF
	@param[in] filepath    to ctrl_params.csv file, which should 
                           store in the order: [c_T0, c_TVom, a_0, 
                           c_a, e_0, c_e, r_0, c_r]
*/
Plane::Plane(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const std::string& filepath)
             : _ctrl_type(ctrl_type) {
    // Check ctrl_type is valid
    if (_ctrl_type != BODY_RATE and _ctrl_type != CTRL_SURF) {
        throw std::runtime_error("Control type must be either BODY_RATE or CTRL_SURF");
    }

	// Load relevant control parameters from csv file
	Eigen::VectorXd ctrl_params = Data::load_vector(filepath + "/ctrl_params.csv");
    if (ctrl_params.rows() < 2) {
        throw std::runtime_error("Missing some control params for thrust");
    }
    else if (ctrl_params.rows() < 8) {
        ROS_INFO("Missing control params for ctrl srf model, not publishing...");
        _delta_0.setZero();
        _c_delta.setZero();
    }
    else {
	    _delta_0(0) = ctrl_params(2); // a_0
	    _delta_0(1) = ctrl_params(4); // e_0
	    _delta_0(2) = ctrl_params(6); // r_0
	    _c_delta(0) = ctrl_params(3); // c_a
	    _c_delta(1) = ctrl_params(5); // c_e
	    _c_delta(2) = ctrl_params(7); // c_r
    }
    _c_T0 = ctrl_params(0);
    _c_TVom = ctrl_params(1);

    // Initialize variables
    _p_b_i_I.setZero();
    _v_b_I_B.setZero();
    _q_I_to_B.setZero();
    _euler.setZero();
    _om_B_I_B.setZero();
    _thrust = 0.0;
    _ctrl_srf.setZero();
    
    // Define MAVROS subscribers
    _state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 1, &Plane::state_cb, this);
    _pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 1, &Plane::pose_cb, this);
    _twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_body", 1, &Plane::twist_cb, this);
    _act_sub = nh.subscribe<mavros_msgs::ActuatorControl>
                ("mavros/target_actuator_control", 1, &Plane::act_cb, this);

    // Define MAVROS publisher
    if (_ctrl_type == CTRL_SURF) {
        _ctrl_pub = nh.advertise<mavros_msgs::ActuatorControl>
                ("mavros/actuator_control", 1);
        ROS_INFO("Control type set to CTRL_SURF");
    }
    else if (_ctrl_type == BODY_RATE) {
        _ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                ("mavros/setpoint_raw/attitude", 1);
        ROS_INFO("Control type set to BODY_RATE");
    }

    _pos_pub = nh.advertise<geometry_msgs::PointStamped>
                ("plane/position", 10);
    _vel_pub = nh.advertise<geometry_msgs::PointStamped>
                ("plane/velocity", 10);
    _euler_pub = nh.advertise<geometry_msgs::PointStamped>
                ("plane/euler", 10);
    _bodyrate_pub = nh.advertise<geometry_msgs::PointStamped>
                ("plane/bodyrate", 10);
    _act_pub = nh.advertise<mavros_msgs::ActuatorControl>
                ("plane/actuators", 10);
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
    else if (_ctrl_type == BODY_RATE) {
        mavros_msgs::AttitudeTarget cmd;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

        // Convert desired body rates back into FLU from FRD
        cmd.body_rate.x = u_nrmlzd(1);
        cmd.body_rate.y = -u_nrmlzd(2);
        cmd.body_rate.z = -u_nrmlzd(3);
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

	@param[in]  delta  control surface deflection cmd [radians]
	@param[in]  i     index int 0=a, 1=e, 2=r
	@param[out] normalized command in range [-1, 1]
*/
double Plane::nrmlz_ctrl_srf_cmd(const double delta, const int i) {
	return std::min(1.0, std::max(-1.0, (delta - _delta_0(i))/_c_delta(i) ));
}

/**
    @brief Converts a commanded thrust value into a normalized 
	[0,1] command that can be sent over actuator_control MAVROS 
	topics. Assumes a quadratic mapping from cmd to T.

    @param[in]  T    thrust command [N]
	@param[in]  V    rel. windspeed parallel to body x axis [m/s]
    @param[out] normalized command in range [0, 1]
*/
double Plane::nrmlz_thrust_cmd(const double T, const double V) {
    return std::min(1.0, std::max(0.0, 0.5*(_c_TVom*V + 
				sqrt( (_c_TVom*V)*(_c_TVom*V) + 4.0*T/_c_T0 )) ));
}

/**
    @brief Converts a control surface deflection angle setpoint
    from a normalized [-1,1] command to dimensional angle.

    @param[in]  u_d  normalized control surface deflection setpoint
    @param[in]  i    index int 0=a, 1=e, 2=r
    @param[out] control surface deflection setpoint [rad]
*/
double Plane::dim_ctrl_srf_cmd(const double u_d, const int i) {
    return _c_delta(i)*u_d + _delta_0(i);
}

/**
    @brief Converts a thrust value setpoint from a normalized
    [0,1] command to value in N. 

    @param[in]  u_T  normalized thrust command
    @param[in]  V    rel. windspeed parallel to body x axis [m/s]
    @param[out] thrust setpoint [N]
*/
double Plane::dim_thrust_cmd(const double u_T, const double V) {
    if (u_T < .000001) return 0.0;
    else {
        return _c_T0*(1.0 - _c_TVom*V/u_T)*pow(u_T, 2.0);
    }
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

    // Publish position
    geometry_msgs::PointStamped pos;
    pos.header.stamp = msg->header.stamp;
    Utils::eigen3d_to_point(_p_b_i_I, pos);
    _pos_pub.publish(pos);

    // Publish euler angles
    geometry_msgs::PointStamped euler;
    euler.header.stamp = msg->header.stamp;
    Utils::eigen3d_to_point(_euler, euler);
    _euler_pub.publish(euler);
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

    // Publish velocity
    geometry_msgs::PointStamped vel;
    vel.header.stamp = msg->header.stamp;
    Utils::eigen3d_to_point(_v_b_I_B, vel);
    _vel_pub.publish(vel);

    // Publish body rates
    geometry_msgs::PointStamped om;
    om.header.stamp = msg->header.stamp;
    Utils::eigen3d_to_point(_om_B_I_B, om);
    _bodyrate_pub.publish(om);
}

/**
    @brief Callback for extracting current thrust and control surface
    deflection angle setpoints.

    @param[in] msg  message from /mavros/target_actuator_control
*/
void Plane::act_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg) {
    if (msg->group_mix == mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL) {
        _ctrl_srf(0) = dim_ctrl_srf_cmd(msg->controls[0], 0);
        _ctrl_srf(1) = dim_ctrl_srf_cmd(msg->controls[1], 1);
        _ctrl_srf(2) = dim_ctrl_srf_cmd(msg->controls[2], 2);
        _thrust = dim_thrust_cmd(msg->controls[3], _v_b_I_B(0));
    }
    else throw "Group mix of /mavros/target_actuator_control not recognized";
    
    // Publish thrust and deflection angles
    mavros_msgs::ActuatorControl actuators;
    actuators.header.stamp = msg->header.stamp;
    actuators.group_mix = msg->group_mix;
    actuators.controls[0] = _ctrl_srf(0);
    actuators.controls[1] = _ctrl_srf(1);
    actuators.controls[2] = _ctrl_srf(2);
    actuators.controls[3] = _thrust;
    _act_pub.publish(actuators);
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
    @brief Returns current commanded thrust in N.
*/
double Plane::get_thrust() {
    return _thrust;
}

/**
    @brief Returns current commanded control surface deflection angles in rad.
*/
Vec3 Plane::get_ctrl_srf() {
    return _ctrl_srf;
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


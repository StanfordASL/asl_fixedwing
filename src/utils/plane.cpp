/** 
    @file plane.cpp
    Definition of Plane class to provide useful utilities. This plane
    class assumes the control inputs are the thrust commands and
    the either the control surface deflection commands or body rate commands.
*/

#include <utils/plane.hpp>
#include <utils/data.hpp>
#include <utils/rotation.hpp>

/** 
	@brief Constructor which loads parameters from csv file.

    @param[in] ctrl_type   control type, either Plane::BODY_RATE 
                           or Plane::CTRL_SURF
	@param[in] filepath    to ctrl_params.csv file, which should 
                           store in the order: [c_T0, c_TVom, a_0, 
                           c_a, e_0, c_e, r_0, c_r]

*/
Plane::Plane(ros::NodeHandle& nh, const unsigned ctrl_type, const std::string& filepath)
                    : _ctrl_type(ctrl_type) {
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
    _p_b_in_i.setZero();
    _q_i_to_b.setZero();
    _euler.setZero();
    
    // Define MAVROS subscribers
    _state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, &Plane::state_cb, this);
    _pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10, &Plane::pose_cb, this);
    _twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity", 10, &Plane::twist_cb, this);
    _euler_pub = nh.advertise<geometry_msgs::Point>
                ("plane/euler", 10);
}

/**
	@brief Converts the full control u = [T, a, e, r] or u = [T, p, q, r] 
    given in unit [N, rad, rad, rad] or [N, rad/s, rad/s, rad/s] to 
    normalized quantities that are commanded through MAVROS.

	@param[in] u         dimensional control vector
	@param[in] u_nrmlzd  normalized commands
	@param[in] V         rel. airspeed parallel to body x axis [m/s]
*/
void Plane::normalize_control(const uvec& u, uvec& u_nrmlzd, const double V) {
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
    Eigen::Vector3d p_b_in_i_ENU;
    p_b_in_i_ENU(0) = msg->pose.position.x;
    p_b_in_i_ENU(1) = msg->pose.position.y;
    p_b_in_i_ENU(2) = msg->pose.position.z;

    // Since the position is written w.r.t ENU frame, do a rotation
    // to map to NED frame.
    _p_b_in_i = _R_ENU_TO_NED * p_b_in_i_ENU;

    // Transform the quaternion from ENU to ENU to NED to FRD
    Eigen::Vector4d q_enu_to_flu;
    Eigen::Vector4d q_ned_to_flu;
    q_enu_to_flu(0) = msg->pose.orientation.w;
    q_enu_to_flu(1) = msg->pose.orientation.x;
    q_enu_to_flu(2) = msg->pose.orientation.y;
    q_enu_to_flu(3) = msg->pose.orientation.z;
    Rot::compose_quats(_Q_NED_TO_ENU, q_enu_to_flu, q_ned_to_flu);
    Rot::compose_quats(q_ned_to_flu, _Q_FLU_TO_FRD, _q_i_to_b);

    // Compute Euler angles from quaternion
    Rot::quat_to_euler(_q_i_to_b, _euler);

    geometry_msgs::Point e;
    e.x = _euler(0);
    e.y = _euler(1);
    e.z = _euler(2);
    _euler_pub.publish(e);
}

/**
    @brief Callback for extracting information from MAVROS
    local_position/velocity messages. These messages provide

    @param[in] msg  message from /mavros/local_position/velocity
*/
void Plane::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {

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
    @brief Returns aircraft position in local inertial NED frame coordinates.
*/
Eigen::Vector3d Plane::get_pos() {
    return _p_b_in_i;
}

/**
    @brief Returns aircraft euler angles (roll, pitch, yaw) with respect
    to local inertial NED frame.
*/
Eigen::Vector3d Plane::get_euler() {
    return _euler;
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


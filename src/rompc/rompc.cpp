/**
	@file rompc.cpp
	Implementation of the ROMPC control scheme.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <utils/data.hpp>
#include <utils/rotation.hpp>
#include <utils/utils.hpp>

#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>

/**
    @brief Constructor which loads model and control parameters from file
    
    @param[in] nh        ROS nodehandle
    @param[in] filepath  to directory containing model definition
*/
ROMPC::ROMPC(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const unsigned target_type, const unsigned att_type, 
             const std::string filepath)
             : _ctrl_type(ctrl_type), _target_type(target_type), 
             _att_type(att_type) {
    
    // Define control type
    if (_ctrl_type == CTRL_SURF) {
        ROS_INFO("ROMPC control type set to CTRL_SURF");
    }
    else if (_ctrl_type == BODY_RATE) {
        ROS_INFO("ROMPC control type set to BODY_RATE");
    }             
    else {
        throw std::runtime_error("Control type must be either BODY_RATE or CTRL_SURF");
    }

    // Set up target
    if (_target_type == SGF) {
        // Load parameters p = [S, gamma, th]
        Eigen::VectorXd p = Data::load_vector(filepath + "/target.csv");
        _target = std::unique_ptr<ROMPC_UTILS::Target>(new ROMPC_UTILS::SGF(p(0), p(1), p(2)));
        ROS_INFO("Target is STEADY GLIDESLOPE FLIGHT");
    }
    else if (_target_type == SLF) {
        // Load paramters p = [S, th]
        Eigen::VectorXd p = Data::load_vector(filepath + "/target.csv");
        _target = std::unique_ptr<ROMPC_UTILS::Target>(new ROMPC_UTILS::SGF(p(0), 0.0, p(1)));
        ROS_INFO("Target is STEADY LEVEL FLIGHT");
    }
    else if (_target_type == STF) {
        // Load parameters p = [v, om, phi, th, R]
        Eigen::VectorXd p = Data::load_vector(filepath + "/target.csv");
        Eigen::Vector3d v(p(0), p(1), p(2));
        Eigen::Vector3d om(p(3), p(4), p(5));
        _target = std::unique_ptr<ROMPC_UTILS::Target>(new ROMPC_UTILS::STF(v, om, p(6), p(7), p(8)));
        ROS_INFO("Target is STEADY TURNING FLIGHT");
    }
    else {
        throw std::runtime_error("Control type must be {SGF, SLF, STF}");
    }
    
    // Check attitude parameterization type
    if (_att_type == EULER) {
        ROS_INFO("Attitude parameterization is EULER");
    }
    else if (_att_type == AA) {
        ROS_INFO("Attitude param is AA");
    }
    else {
        throw std::runtime_error("Attitude parameterization must be {EULER, AA}");
    }
    
    // Load controller parameters from file
	_A = Data::load_matrix(filepath + "/A.csv");
	_B = Data::load_matrix(filepath + "/B.csv");
	_C = Data::load_matrix(filepath + "/C.csv");
	_H = Data::load_matrix(filepath + "/H.csv");
	_K = Data::load_matrix(filepath + "/K.csv");
	_L = Data::load_matrix(filepath + "/L.csv");
    _u_eq = Data::load_vector(filepath + "/u_eq.csv");
	
    // ROS publishers
    _e_pos_pub = nh.advertise<geometry_msgs::Point>
                    ("rompc/pos_error", 1);
    _e_vel_pub = nh.advertise<geometry_msgs::Point>
                    ("rompc/vel_error", 1);
    _e_euler_pub = nh.advertise<geometry_msgs::Point>
                    ("rompc/euler_error", 1);
    _ubar_pub = nh.advertise<std_msgs::Float32MultiArray>
                    ("rompc/ubar", 1);
    _u_pub = nh.advertise<std_msgs::Float32MultiArray>
                    ("rompc/u", 1);
    _zbar_pub = nh.advertise<std_msgs::Float32MultiArray>
                    ("rompc/zbar", 1);
    _zhat_pub = nh.advertise<std_msgs::Float32MultiArray>
                    ("rompc/zhat", 1);

    // Initialize other variables to zero
    int n = _A.rows();
    int m = _B.cols();
    int p = _C.rows();
    _xbar.resize(n);
    _ubar.resize(m);
    _xhat.resize(n);
    _u.resize(m);
    _y.resize(p);
    _xbar.setZero();
    _ubar.setZero();
    _xhat.setZero();
    _u.setZero();
    _y.setZero();
    _zhat.setZero();
    _zbar.setZero();
}

/**
    @brief Initialize controller internal state estimate and nominal
    state to zero every time the target position is reset to current
    position
*/
void ROMPC::init(const double t0, const Eigen::Vector3d p, const double psi) {	
    // Initialize target with current position and heading
    _t0 = t0;
    _target->initialize(p, psi);
    
    // Set ROMPC state estimate and reference system to zero
    _xhat.setZero();
	_xbar.setZero();
    
    _init = true;
    _t = t0;
}

/**
    @brief update the ROMPC controller based on the most recent measurements
    from the rigid body states of the aircraft.

    @param[in] t          current time [s]
    @param[in] p_b_i_I    inertial position in inertial NED coordinates [m]
    @param[in] v_b_I_B    inertial velocity in FRD body frame coordinates [m/s]
    @param[in] euler      aircraft euler angles (roll, pitch, yaw) in [rad]
    @param[in] om_B_I_B   aircraft body rate (p, q, r) in [rad/s]
    @param[in] T          current thrust setpoint [N]
    @param[in] ctrl_srf   current control surface deflection setpoints [rad]
*/
void ROMPC::update(const double t, const Vec3 p_b_i_I, const Vec3 v_b_I_B, 
                const Vec3 euler, const Vec3 om_B_I_B, const double T,
                const Vec3 ctrl_srf) {
    
    if (!_init) return;

    // Compute position relative to target in target reference frame coord.
    // TODO need to do a rotation from I coordinates to R coordinates
    Vec3 e_pos = p_b_i_I - _target->get_pos(t - _t0);

    // Compute velocity relative to target
    Vec3 e_vel = v_b_I_B - _target->get_vel(t - _t0);

    // Compute attitude relative to target
    Vec3 e_att = euler - _target->get_att_euler(t - _t0);

    // Publish errors
    geometry_msgs::Point pos_error;
    Utils::eigen3d_to_point(e_pos, pos_error);
    _e_pos_pub.publish(pos_error);

    geometry_msgs::Point vel_error;
    Utils::eigen3d_to_point(e_vel, vel_error);
    _e_vel_pub.publish(vel_error);

    geometry_msgs::Point euler_error;
    euler_error.x = Rot::rad_to_deg(e_att(0));
    euler_error.y = Rot::rad_to_deg(e_att(1));
    euler_error.z = Rot::rad_to_deg(e_att(2));
    _e_euler_pub.publish(euler_error);
    
    if (_att_type == AA) {
        // Change e_att to axis angle param
        Eigen::Vector4d q_I_to_B;
        Eigen::Vector3d aa_I_to_B;
        Rot::euler_to_quat(euler, q_I_to_B);
        Rot::quat_to_axis(q_I_to_B, aa_I_to_B);
        e_att = aa_I_to_B - _target->get_att_aa(t - _t0);
    }
    
    // Compute current perturbation from equilibrium control
    Eigen::Vector4d u_prev;
    if (_ctrl_type == CTRL_SURF) {
        u_prev << T, ctrl_srf;
    }
    else if (_ctrl_type == BODY_RATE) {
        u_prev << T, om_B_I_B;
    }
    u_prev -= _u_eq;

    // Update ROMPC controller with error measurements
    Eigen::Matrix<double, 9, 1>  y;
    y << e_vel, e_pos, e_att;
    ROMPC::update_ctrl(t, y, u_prev);    
}

/**
    @brief Update the control based on new mesurements
*/
void ROMPC::update_ctrl(const double t, const Eigen::VectorXd y,
                        const Eigen::Vector4d u_prev) {
    double dt = t - _t; // time step since last update
    _t = t;

    // Update state estimate from previous step
    _xhat += dt*(_A*_xhat + _B*u_prev + _L*(y - _C*_xhat));
    _zhat = _H*_xhat;
    
    // Update simulated ROM TODO
    _xbar = _xbar;
    _zbar = _H*_xbar;
    
    // Get simulated ROM state and control at time t
    ROMPC::update_sim_rom(t);
    
    // Control law
    _u = _ubar + _K*(_xhat - _xbar);
    
    // Save current measurement
    _y = y;

    // Broadcast control value on ROS topic
    std_msgs::Float32MultiArray nom_ctrl, ctrl;
    Utils::eigenxd_to_multarr(_ubar, nom_ctrl);
    Utils::eigenxd_to_multarr(_u, ctrl);
    _ubar_pub.publish(nom_ctrl);
    _u_pub.publish(ctrl);

    // Broadcast performance output value on ROS topic
    std_msgs::Float32MultiArray zbar, zhat;
    Utils::eigenxd_to_multarr(_zhat, zhat);
    Utils::eigenxd_to_multarr(_zbar, zbar);
    _zbar_pub.publish(zbar);
    _zhat_pub.publish(zhat);
}

/**
    @brief Update the simulated ROM state and control TODO
*/
void ROMPC::update_sim_rom(const double t) {
	_ubar.setZero();
	_xbar.setZero();
}

/**
    @brief Get the current control with addition of equilibrium
    control term
*/
Eigen::VectorXd ROMPC::get_ctrl() {
	return _u + _u_eq; // include equilibrium feedforward term
}

Eigen::VectorXd ROMPC::get_xhat() {
    return _xhat;
}

Eigen::VectorXd ROMPC::get_xbar() {
    return _xbar;
}


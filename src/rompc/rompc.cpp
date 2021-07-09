/**
	@file rompc.cpp
	Implementation of the ROMPC control scheme.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <utils/data.hpp>
#include <utils/rotation.hpp>
#include <utils/utils.hpp>
#include <asl_fixedwing/FloatVecStamped.h>

#include <string>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>

/**
    @brief Constructor which loads model and control parameters from file
    
    @param[in] nh           ROS nodehandle
    @param[in] ctrl_type    ROMPC::BODY_RATE or ROMPC::CTRL_SURF
    @param[in] target_type  ROMPC::SGF,SLF,STF
    @param[in] model_type   ROMPC::STANDARD or ROMPC::CFD
    @param[in] dt           timestep for controller, seconds
    @param[in] filepath     to directory containing model definition
    @param[in] tmax         max time for OCP, seconds
    @param[in] debug        boolean
*/
ROMPC::ROMPC(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const unsigned target_type, const unsigned model_type, double dt, 
             const std::string filepath, const double tmax, const bool debug)
             : _ctrl_type(ctrl_type), _target_type(target_type), 
             _model_type(model_type), _debug(debug), _ocp(filepath, tmax) {
    
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
        VecX p = Data::load_vector(filepath + "/target.csv");
        _target = TargetPtr(new ROMPC_UTILS::SGF(p(0), p(1), p(2)));
        ROS_INFO("Target is STEADY GLIDESLOPE FLIGHT");
    }
    else if (_target_type == SLF) {
        // Load paramters p = [S, th]
        VecX p = Data::load_vector(filepath + "/target.csv");
        _target = TargetPtr(new ROMPC_UTILS::SGF(p(0), 0.0, p(1)));
        ROS_INFO("Target is STEADY LEVEL FLIGHT");
    }
    else if (_target_type == STF) {
        // Load parameters p = [v, om, phi, th, R]
        VecX p = Data::load_vector(filepath + "/target.csv");
        Vec3 v(p(0), p(1), p(2));
        Vec3 om(p(3), p(4), p(5));
        _target = TargetPtr(new ROMPC_UTILS::STF(v, om, p(6), p(7), p(8)));
        ROS_INFO("Target is STEADY TURNING FLIGHT");
    }
    else {
        throw std::runtime_error("Control type must be {SGF, SLF, STF}");
    }
    
    // Check model type
    if (_model_type == STANDARD) {
        ROS_INFO("Using STANDARD model inputs and measurements");
    }
    else if (_model_type == CFD) {
        ROS_INFO("Using CFD model inputs and measurements");
    }
    else {
        throw std::runtime_error("Model must be {STANDARD, CFD}");
    }

    // Check OCP initialization success
    _qp_dt = _ocp.get_dt();
    ROS_INFO("QP solver time limit set to %.1f ms", 1000.0*tmax);
    ROS_INFO("MPC problem has dt = %.1f ms and N = %d", 1000.0*_qp_dt, _ocp.get_N());
    if (_ocp.success()) {
        ROS_INFO("QP initial solve SUCCESS in %.1f ms", 1000.0*_ocp.solve_time());
    }
    else {
        ROS_INFO("QP initial solve FAILED"); 
    }
    
    // Load controller parameters from file
	_A = Data::load_matrix(filepath + "/A.csv");
	_B = Data::load_matrix(filepath + "/B.csv");
	_C = Data::load_matrix(filepath + "/C.csv");
	_H = Data::load_matrix(filepath + "/H.csv");
	_K = Data::load_matrix(filepath + "/K.csv");
	_L = Data::load_matrix(filepath + "/L.csv");
    _u_eq = Data::load_vector(filepath + "/u_eq.csv");
    _n = _A.rows();
    int m = _B.cols();
    int p = _C.rows();

    // Compute QR decompositions for implicit Euler
    _M_est.compute(MatX::Identity(_n, _n) - dt*(_A - _L*_C));
    _M_rom.compute(MatX::Identity(_n, _n) - dt*_A);

    // ROS publishers
    _e_pos_pub = nh.advertise<geometry_msgs::PointStamped>
                    ("rompc/pos_error", 1);
    _e_vel_pub = nh.advertise<geometry_msgs::PointStamped>
                    ("rompc/vel_error", 1);
    _e_att_pub = nh.advertise<geometry_msgs::PointStamped>
                    ("rompc/att_error", 1);
    _e_attrate_pub = nh.advertise<geometry_msgs::PointStamped>
                    ("rompc/attrate_error", 1);
    _ubar_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                    ("rompc/ubar", 1);
    _u_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                    ("rompc/u", 1);
    _zbar_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                    ("rompc/zbar", 1);
    _zhat_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                    ("rompc/zhat", 1);
    if (_debug) {
        _y_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                        ("rompc/y", 1);
        _uprev_pub = nh.advertise<asl_fixedwing::FloatVecStamped>
                        ("rompc/u_prev", 1);
        _qptime_pub = nh.advertise<std_msgs::Float32>
                        ("rompc/qp_solve_time", 1);
    }

    // Initialize other variables to zero
    _xbar.resize(_n);
    _ubar.resize(m);
    _xhat.resize(_n);
    _u.resize(m);
    _y.resize(p);
    _xbar.setZero();
    _ubar.setZero();
    _xhat.setZero();
    _u.setZero();
    _y.setZero();
    _zhat.setZero();
    _zbar.setZero();
    _q_R_to_B.setZero();
    _t0 = _t = _t_qp = 0.0;
}

/**
    @brief Initialize controller internal state estimate and nominal
    state to zero every time the target position is reset to current
    position
*/
void ROMPC::init(const double t0, const Vec3 p, const double psi) {	
    // Initialize target with current position and heading
    _t0 = t0;
    _target->initialize(p, psi);
    
    // Set ROMPC state estimate and reference system to zero
    _xhat.setZero();
	_xbar.setZero();
    
    _init = true;
    _t = t0;
    _t_qp = t0 - _qp_dt;
}

/**
    @brief Start the ROMPC scheme
*/
void ROMPC::start() {
    _started = true;
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
    Vec4 q_I_to_R = _target->get_att_quat(t - _t0);
    Mat3 R_R_to_I;
    Rot::quat_to_R(q_I_to_R, R_R_to_I);
    Vec3 p_b_r_R = R_R_to_I.transpose() * (p_b_i_I - _target->get_pos(t - _t0));
    
    // Compute quaternion of rotation from R to B
    Vec4 q_I_to_B;
    Rot::euler_to_quat(euler, q_I_to_B);
    Rot::invert_quat(q_I_to_R); // rotates in place, probs bad design
    Rot::compose_quats(q_I_to_R, q_I_to_B, _q_R_to_B);
    
    // Depending on the model inputs and outputs, define y and u_prev
    Vec3 e_vel, e_att, e_attrate;
    Vec4 u_prev;
    if (_model_type == STANDARD) {
        
        // Compute body frame velocity relative to target body frame velocity
        e_vel = v_b_I_B - _target->get_vel(t - _t0);
        
        // Compute attitude relative to target in Euler angles
        e_att = euler - _target->get_att_euler(t - _t0);
        e_att(2) = Rot::wrap_to_pi(e_att(2)); // wrap yaw to [-pi, pi]
        
        // Compute body rate error 
        e_attrate = om_B_I_B - _target->get_om(t - _t0);
        
        if (_ctrl_type == CTRL_SURF) {
            _y << e_vel, e_attrate, e_att, p_b_r_R;
            u_prev << T, ctrl_srf;
        }
        else if (_ctrl_type == BODY_RATE) {
            _y << e_vel, e_att, p_b_r_R;
            u_prev << T, om_B_I_B;
        }
    }
    else if (_model_type == CFD) {
        
        // Compute rate of change of position error in target frame coord.
        Mat3 R_B_to_R; // rotates vector from B coord to R coord
        Rot::quat_to_R(_q_R_to_B, R_B_to_R);
        e_vel = R_B_to_R * v_b_I_B - 
                    _target->get_vel(t - _t0) - 
                    _target->get_om(t - _t0).cross(p_b_r_R);

        // Compute attitude relative to target in axis/angle param
        Rot::quat_to_axis(_q_R_to_B, e_att);

        // Compute rate of change of axis/angle params
        // First the angular vel of B w.r.t R is computed based on measured
        // ang vel of B w.r.t I and ang vel of R w.r.t I
        Vec3 om_B_R_B = om_B_I_B - R_B_to_R.transpose()*_target->get_om(t - _t0);
        ROMPC_UTILS::om_to_aadot(e_att, om_B_R_B, e_attrate);

        if (_ctrl_type == CTRL_SURF) {
            _y << e_vel, p_b_r_R, e_attrate, e_att;
            u_prev << T, ctrl_srf;
        }
        else if (_ctrl_type == BODY_RATE) {
            _y << e_vel, p_b_r_R, e_att;
            u_prev << T, e_attrate;
        }
    }
    
    // Compute current perturbation from equilibrium control
    u_prev -= _u_eq;
    
    // Update ROMPC conroller
    ROMPC::update_ctrl(t, u_prev);
    
    // Publish errors
    ros::Time time(t);
    geometry_msgs::PointStamped pos_error;
    pos_error.header.stamp = time;
    Utils::eigen3d_to_point(p_b_r_R, pos_error);
    _e_pos_pub.publish(pos_error);

    geometry_msgs::PointStamped vel_error;
    vel_error.header.stamp = time;
    Utils::eigen3d_to_point(e_vel, vel_error);
    _e_vel_pub.publish(vel_error);

    geometry_msgs::PointStamped att_error;
    att_error.header.stamp = time;
    Utils::eigen3d_to_point(e_att, att_error);
    _e_att_pub.publish(att_error);
   
    geometry_msgs::PointStamped attrate_error;
    attrate_error.header.stamp = time;
    Utils::eigen3d_to_point(e_attrate, attrate_error);
    _e_attrate_pub.publish(attrate_error);

    if (_debug) {
        asl_fixedwing::FloatVecStamped up, y;
        up.header.stamp = y.header.stamp = time;
        Utils::eigenxd_to_floatvec(u_prev, up);
        Utils::eigenxd_to_floatvec(_y, y);
        _uprev_pub.publish(up);
        _y_pub.publish(y);
    }
}

/**
    @brief Update the control based on new mesurements
*/
void ROMPC::update_ctrl(const double t, const Vec4 u_prev) {
    double dt = t - _t; // time step since last update
    _t = t;

    // Update state estimate from previous step using backward Euler
    _xhat = _M_est.solve(_xhat + dt*_B*u_prev + dt*_L*_y); // backward Euler
    _zhat = _H*_xhat;
    
    // Update simulated ROM
    if (!_started) {
        _ubar = u_prev - _K*(_xhat - _xbar);
    }
    else if (t >= _t_qp) {
        _t_qp = t + _qp_dt;
        _ocp.solve(_xbar, _ubar);
        if (!_ocp.success()) ROS_INFO("QP solver failed or ran out of time");
        
        if (_debug) {
            std_msgs::Float32 time;
            time.data = 1000.0*_ocp.solve_time();
            _qptime_pub.publish(time);
        }
    }
    
    _xbar = _M_rom.solve(_xbar + dt*_B*_ubar); // backward Euler
    _zbar = _H*_xbar;
    
    // Control law
    if (!_started) _u = _K*_xhat;
    else _u = _ubar + _K*(_xhat - _xbar);
    
    // Broadcast control value on ROS topic
    ros::Time time(t);
    asl_fixedwing::FloatVecStamped nom_ctrl, ctrl;
    nom_ctrl.header.stamp = ctrl.header.stamp = time;
    Utils::eigenxd_to_floatvec(_ubar, nom_ctrl);
    Utils::eigenxd_to_floatvec(_u, ctrl);
    _ubar_pub.publish(nom_ctrl);
    _u_pub.publish(ctrl);

    // Broadcast performance output value on ROS topic
    asl_fixedwing::FloatVecStamped zbar, zhat;
    zbar.header.stamp = zhat.header.stamp = time;
    Utils::eigenxd_to_floatvec(_zhat, zhat);
    Utils::eigenxd_to_floatvec(_zbar, zbar);
    _zbar_pub.publish(zbar);
    _zhat_pub.publish(zhat);
}

/**
    @brief Get the current control with addition of equilibrium
    control term
*/
VecX ROMPC::get_ctrl(double t) {
    
    // Need to convert from axis angle parameter rates to ang. vel.
    if (_model_type == CFD && _ctrl_type == BODY_RATE) {
        Vec4 u = _u + _u_eq;

        // Get axis angle parameter rates
        Vec3 e_attrate;
        e_attrate << u(1), u(2), u(3);

        // Get current axis angle for rotation from R to B
        Vec3 e_att;
        Rot::quat_to_axis(_q_R_to_B, e_att);

        // Convert axis angle rates to angular velocity
        Vec3 om_B_R_B;
        ROMPC_UTILS::aadot_to_om(e_att, e_attrate, om_B_R_B);

        // Compute ang. vel. of body w.r.t inertial
        Mat3 R_B_to_R; // rotates vector from B coord to R coord
        Rot::quat_to_R(_q_R_to_B, R_B_to_R);
        Vec3 om_B_I_B = om_B_R_B + R_B_to_R.transpose()*_target->get_om(t - _t0);
    
        // Control is [T, p, q, r]
        u(1) = om_B_I_B(0);
        u(2) = om_B_I_B(1);
        u(3) = om_B_I_B(2);
        return u;
    }
    
    // Otherwise the control is already in the correct form!
    else {
        return _u + _u_eq;
    }    
}

VecX ROMPC::get_xhat() {
    return _xhat;
}

VecX ROMPC::get_xbar() {
    return _xbar;
}

bool ROMPC::started() {
    return _started;
}

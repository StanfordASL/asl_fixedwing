#ifndef ROMPC_CTRL_HPP
#define ROMPC_CTRL_HPP

/**
	@file rompc.hpp
	Header file defining the ROMPC controller class.

	@brief ROMPC controller class.
*/
#include <rompc/rompc_utils.hpp>

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;

class ROMPC {
public:
	ROMPC(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const unsigned target_type, const unsigned att_type, 
             const std::string filepath);
	void init(const double t0, const Eigen::Vector3d p, const double psi);
	void update(const double t, const Vec3 p_b_i_I, const Vec3 v_b_I_B, 
                const Vec3 euler, const Vec3 om_B_I_B, const double T,
                const Vec3 ctrl_srf); 
	Eigen::VectorXd get_ctrl();
	Eigen::VectorXd get_xhat();
	Eigen::VectorXd get_xbar();

    // Control types
    static const unsigned BODY_RATE = 0;
    static const unsigned CTRL_SURF = 1;

    // Target types
    static const unsigned SGF = 0; // steady glidelsope flight
    static const unsigned SLF = 1; // steady level flight
    static const unsigned STF = 2; // steady turning flight

    // Attitude types
    static const unsigned EULER = 0; // using euler angles
    static const unsigned AA = 1; // using axis/angle

private:
	Eigen::MatrixXd _K; // controller gain matrix
	Eigen::MatrixXd _L; // estimator gain matrix
	Eigen::MatrixXd _A; // system dynamics ...
	Eigen::MatrixXd _B; // ...
	Eigen::MatrixXd _C; // ...
	Eigen::MatrixXd _H; // ... matrices

	Eigen::VectorXd _xbar; // nominal ROM state
	Eigen::VectorXd _ubar; // nominal ROM control
	Eigen::VectorXd _xhat; // ROM state estimate
    Eigen::VectorXd _u_eq; // equilibrium feed forward control
	Eigen::VectorXd _u; // control to be applied
	Eigen::VectorXd _y; // measurement
    Eigen::VectorXd _zhat; // estimated output variables
    Eigen::VectorXd _zbar; // nominal output variables

	double _t0; // initialization time (s)
    double _t; // last update time (s)
    bool _debug;
    bool _init = false;

    unsigned _ctrl_type; // Control type
    unsigned _target_type; // Target type
    unsigned _att_type; // Attitude parameterization

    std::unique_ptr<ROMPC_UTILS::Target> _target; // pointer to target object
    
    ros::Publisher _e_pos_pub; // tracking position error [m]
    ros::Publisher _e_vel_pub; // tracking velocity error [m/s]
    ros::Publisher _e_euler_pub; // Euler angles error [rad]
    ros::Publisher _ubar_pub;
    ros::Publisher _u_pub;
    ros::Publisher _zbar_pub;
    ros::Publisher _zhat_pub;

    void update_ctrl(const double t, const Eigen::VectorXd y,
                     const Eigen::Vector4d u_prev);
	void update_sim_rom(const double t);
};

#endif // ROMPC_CTRL_HPP

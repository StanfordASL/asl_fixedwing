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
using Vec4 = Eigen::Vector4d;
using VecX = Eigen::VectorXd;
using Mat3 = Eigen::Matrix3d;
using MatX = Eigen::MatrixXd;

using TargetPtr = std::unique_ptr<ROMPC_UTILS::Target>; 

class ROMPC {
public:
	ROMPC(ros::NodeHandle& nh, const unsigned ctrl_type, 
             const unsigned target_type, const unsigned model_type, 
             const std::string filepath, const double tmax, const bool debug = false);
	void init(const double t0, const Vec3 p, const double psi);
	void update(const double t, const Vec3 p_b_i_I, const Vec3 v_b_I_B, 
                const Vec3 euler, const Vec3 om_B_I_B, const double T,
                const Vec3 ctrl_srf); 
	VecX get_ctrl(double t);
	VecX get_xhat();
	VecX get_xbar();

    // Control types
    static const unsigned BODY_RATE = 0;
    static const unsigned CTRL_SURF = 1;

    // Model types
    // Standard model is u = [T, p, q, r]
    //                   y = [u, v, w, phi, th, psi, x_r, y_r, z_r]
    // or                u = [T, a, e, r]
    //                   y = [u, v, w, p, q, r, phi, th, psi, x_r, y_r, z_r]
    static const unsigned STANDARD = 0;
    
    // CFD measurement is u = [T, th1d, th2d, th3d]
    //                    y = [xd_r, yd_r, zd_r, x_r, y_r, z_r, th1, th2, th3]
    // or                 u = [T, a, e, r]
    //                    y = [xd_r, yd_r, zd_r, x_r, y_r, z_r, th1d, th2d
    //                         th3d, th1, th2, th3] 
    // where th = (th1, th2, th3) is axis/angle paramterization of orientation
    static const unsigned CFD = 1;

    // Target types
    static const unsigned SGF = 0; // steady glidelsope flight
    static const unsigned SLF = 1; // steady level flight
    static const unsigned STF = 2; // steady turning flight

private:
    MatX _K; // controller gain matrix
    MatX _L; // estimator gain matrix
    MatX _A; // system dynamics ...
    MatX _B; // ...
    MatX _C; // ...
    MatX _H; // ... matrices
    int _n; // state dimensior
    MatX _AL; // A - LC

    ROMPC_UTILS::OCP _ocp; // optimal control problem

    VecX _xbar; // nominal ROM state
    VecX _ubar; // nominal ROM control
    VecX _xhat; // ROM state estimate
    VecX _u_eq; // equilibrium feed forward control
    VecX _u; // control to be applied
    VecX _y; // measurement
    VecX _zhat; // estimated output variables
    VecX _zbar; // nominal output variables

    double _t0; // initialization time (s)
    double _t; // last update time (s)
    bool _init = false;
    Vec4 _q_R_to_B;

    unsigned _ctrl_type; // Control type
    unsigned _target_type; // Target type
    unsigned _model_type; // Measurement type
    bool _debug;

    TargetPtr _target; // pointer to target object
    
    ros::Publisher _e_pos_pub; // tracking position error [m]
    ros::Publisher _e_vel_pub; // tracking velocity error [m/s]
    ros::Publisher _e_att_pub; // attitude error (euler or axis/angle) [rad]
    ros::Publisher _e_attrate_pub; // attitude rate error (omega or axis/angle)
    ros::Publisher _ubar_pub;
    ros::Publisher _u_pub;
    ros::Publisher _zbar_pub;
    ros::Publisher _zhat_pub;
    ros::Publisher _uprev_pub;
    ros::Publisher _y_pub;

    void update_ctrl(const double t, const Vec4 u_prev);
    void update_sim_rom(const double t);
};

#endif // ROMPC_CTRL_HPP

#ifndef ROMPC_UTILS_HPP
#define ROMPC_UTILS_HPP

/**
	@file rompc_utils.hpp
	Header file defining useful utilities for ROMPC controller.
*/

#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace ROMPC_UTILS {

/**
    @class SGF
   
    @brief Steady glideslope flight target. Velocity (u,v,w) is constant,
    body rates (p,q,r) are constant and zero, no side motion (v=0), roll angle
    is zero (phi=0), and constant descent rate defined by the glideslope
    angle gamma, which is negative when velocity is below local horizon.
*/
class SGF {
protected:
    Vec3 _p_r_i_I; // position relative to inertial in inertial coord
    Vec3 _v_r_I_I; // velocity as seen from I in inertial coord.
    double _S_xy; // speed in the inertial x-y plane 
    Vec3 _v_r_I_R; // velocity as seen from I in target frame coord.
    Vec3 _euler; // euler angles
    Vec3 _th; // constant pitch angle
    Vec4 _q_I_to_R; // quat rotation from inertial NED to target FRD frame
    Vec3 _om_R_I_R; // ang vel (p, q, r) of R w.r.t I frame, in R coordinates

public:
    // Constructor
    SGF(double S, double gamma, double th);

    // Initialize position
    void initialize(Vec3 p, double psi);

    // Query target values at time t
    Vec3 get_pos(double t);
    Vec3 get_vel(double t);
    Vec4 get_attitude(double t);
    Vec3 get_bodyrate(double t);    
};

/**
    @class SLF

    @brief Steady level flight target, same as SGF but with gamma = 0;
*/
class SLF: public SGF {

};

/**
    @class STF
   
    @brief Steady turning flight target. Velocity (u,v,w) is constant,
    body rates (p,q,r) are constant (but not zero), constant yaw rate 
    \dot{\psi} (implicitly defined by constant speed S and turning radius
    R), constant altitude. 
*/
class STF {

};

}

#endif // ROMPC_UTILS_HPP

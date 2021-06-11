#ifndef ROMPC_UTILS_HPP
#define ROMPC_UTILS_HPP

/**
	@file rompc_utils.hpp
	Header file defining useful utilities for ROMPC controller.
*/

#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;

namespace ROMPC_UTILS {

class Target {
public:
    Target();
    virtual void initialize(Vec3 p, double psi) {};
    virtual Vec3 get_pos(double t);
    virtual Vec3 get_vel(double t);
    virtual Vec4 get_att_quat(double t);
    virtual Vec3 get_att_euler(double t);
    virtual Vec3 get_att_aa(double t);
    virtual Vec3 get_om(double t);
    virtual ~Target() = default;
protected: 
    Vec3 _p_r_i_I; // position relative to inertial in inertial coord
    Vec3 _v_r_I_R; // inertial velocity target frame coord.
    Vec4 _q_I_to_R; // quat rotation from inertial NED to target FRD frame
    Vec3 _om_R_I_R; // ang vel (p, q, r) of R w.r.t I frame, in R coordinates
    Vec3 _euler; // euler angles (roll, pitch, yaw)
    Vec3 _aa_I_to_R; // axis angle rotation from inertial NED to target FRD frame
};

/**
    @class SGF
   
    @brief Steady glideslope flight target. Velocity (u,v,w) is constant,
    body rates (p,q,r) are constant and zero, no side motion (v=0), roll angle
    is zero (phi=0), and constant descent rate defined by the glideslope
    angle gamma, which is negative when velocity is below local horizon.
    When gamma = 0 this is equivalent to steady level flight.
*/
class SGF : public Target {
public:
    SGF(double S, double gamma, double th);

    // Initialize position and yaw angle
    void initialize(Vec3 p, double psi) override;

    Vec3 get_pos(double t) override;

protected:
    Vec3 _v_r_I_I; // velocity as seen from I in inertial coord.
    double _S_xy; // speed in the inertial x-y plane 
    double _th; // constant pitch angle
};

/**
    @class STF
   
    @brief Steady turning flight target. Velocity (u,v,w) is constant,
    body rates (p,q,r) are constant (but not zero), constant yaw rate 
    \dot{\psi} (implicitly defined by constant speed S and turning radius
    R), constant altitude. 
*/
class STF: public Target {
public:
    STF(Vec3 v, Vec3 om, double phi,  double th, double R);
    void compute_euler(double t);

    // Initialize position and yaw angle
    void initialize(Vec3 p, double psi) override;

    Vec3 get_pos(double t) override;
    Vec4 get_att_quat(double t) override;
    Vec3 get_att_euler(double t) override;
    Vec3 get_att_aa(double t) override;

protected:
    double _phi; // roll angle constant
    double _th; // pitch angle constant
    double _psi; // yaw angle/heading
    double _psi_dot; // yaw rate
    double _R; // turning radius
    Vec3 _p_c_i_I; // center of circle w.r.t inertial frame in I coord
};

void tangential_transf(const Vec3& aa, Mat3& T);

void om_to_aadot(const Vec3& aa, const Vec3& om, Vec3& aadot);

void aadot_to_om(const Vec3& aa, const Vec3& aadot, Vec3& om);

}

#endif // ROMPC_UTILS_HPP

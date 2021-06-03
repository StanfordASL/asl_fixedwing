/**
	@file rotation.cpp
	Utilities for expressing and converting rotations of reference frames

	@brief All of the rotation parameterizations here represent rotations
	of an initial fixed frame to a new frame. Therefore using these
	parameterizations to operate on a vector is the same as transforming
	the new frame coordinates into the original fixed frame.

	Examples if rotation describes relation of body w.r.t. inertial
	(how to rotate inertial to body):
	R: coverts body frame coordinates into inertial frame coordinates
	euler: usual roll-pitch-yaw of body frame w.r.t inertial frame

*/

#include <utils/rotation.hpp>

#include <Eigen/Dense>
#include <cmath>

/** 
	@brief Converts the rotation matrix R to the Euler angles (ZYX)
	(Yaw-Pitch-Roll) (psi, theta, phi) used for aircraft conventions.
	Note to compute the Euler angles for the aircraft this should be
	the R matrix that converts a vector from body frame coordinates
	to a vector in inertial frame coordinates.

	@param[in] R  rotation matrix
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)
*/
void Rot::R_to_euler(const Eigen::Matrix3d R, Eigen::Vector3d& e) {
	// Check for singularity that occurs when pitch = 90 deg
	if (sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)) >= 1e-6) {
		e(0) = atan2(R(2,1), R(2,2));
	    e(1) = asin(-R(2,0));
	    e(2) = atan2(R(1,0), R(0,0));
	}
	else {
		e(0) = atan2(-R(1,2), R(1,1));
	    e(1) = asin(-R(2,0));
	    e(2) = 0.0;
	}	
}

/**
    @brief Converts Euler angles (ZYX) (Yaw-Pitch-Roll) (psi, theta, phi) 
    into the inertial to body rotation matrix
	
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)
	@param[in] R  rotation matrix
*/
void Rot::euler_to_R(const Eigen::Vector3d e, Eigen::Matrix3d& R) {
	R(0,0) = cos(e(1))*cos(e(2));
	R(0,1) = sin(e(0))*sin(e(1))*cos(e(2)) - cos(e(0))*sin(e(2));
	R(0,2) = sin(e(0))*sin(e(2)) + cos(e(0))*sin(e(1))*cos(e(2));
	R(1,0) = cos(e(1))*sin(e(2));
	R(1,1) = cos(e(0))*cos(e(2)) + sin(e(0))*sin(e(1))*sin(e(2));
	R(1,2) = cos(e(0))*sin(e(1))*sin(e(2)) - sin(e(0))*cos(e(2));
	R(2,0) = -sin(e(1));
	R(2,1) = sin(e(0))*cos(e(1));
	R(2,2) = cos(e(0))*cos(e(1));
}

/**
	@brief Converts a unit quaternion into a rotation matrix

	@param[in] q  unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] R  rotation matrix
*/
void Rot::quat_to_R(const Eigen::Vector4d q, Eigen::Matrix3d& R) {
	R(0,0) = 1.0 - 2.0*q(2)*q(2) - 2.0*q(3)*q(3);
	R(0,1) = 2.0*q(1)*q(2) - 2.0*q(3)*q(0);
	R(0,2) = 2.0*q(1)*q(3) + 2.0*q(2)*q(0);
	R(1,0) = 2.0*q(1)*q(2) + 2.0*q(3)*q(0);
	R(1,1) = 1.0 - 2.0*q(1)*q(1) - 2.0*q(3)*q(3);
	R(1,2) = 2.0*q(2)*q(3) - 2.0*q(1)*q(0);
	R(2,0) = 2.0*q(1)*q(3) - 2.0*q(2)*q(0);
	R(2,1) = 2.0*q(2)*q(3) + 2.0*q(1)*q(0);
	R(2,2) = 1.0 - 2.0*q(1)*q(1) - 2.0*q(2)*q(2);
}

/**
	@brief Converts a unit quaternion into the aircraft Euler angles
	(ZYX) (Yaw-Pitch-Roll) (psi, theta, phi)

	@param[in] q  unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)

*/
void Rot::quat_to_euler(const Eigen::Vector4d q, Eigen::Vector3d& e) {
	double R00 = 1.0 - 2.0*q(2)*q(2) - 2.0*q(3)*q(3);
    double R10 = 2.0*q(1)*q(2) + 2*q(3)*q(0);
    if (sqrt(R00*R00 + R10*R10 >= 1e-6)) {
	    e(0) = atan2(2.0*q(2)*q(3) + 2.0*q(1)*q(0), 1.0 - 2.0*q(1)*q(1) - 2.0*q(2)*q(2));
	    e(1) = asin(-2.0*q(1)*q(3) + 2.0*q(2)*q(0));
	    e(2) = atan2(R10, R00);
    }
    else {
    	e(0) = atan2(-2.0*q(2)*q(3) - 2.0*q(1)*q(0), 1.0 - 2.0*q(1)*q(1) - 2.0*q(3)*q(3));
        e(1) = asin(-2.0*q(1)*q(3) + 2.0*q(2)*q(0));
        e(2) = 0.0;
    }
}

/**
    @brief Converts aircraft Euler angles (ZYX) (Yaw-Pitch-Roll) (psi, theta, phi)
    into unit quaternion

    @param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)
    @param[in] q  unit quaternion q = (w, x, y, z) = w + (x i, y j, z k)
*/
void Rot::euler_to_quat(const Eigen::Vector3d e, Eigen::Vector4d& q) {
    double phi = 0.5 * e(0);
    double th = 0.5 * e(1);
    double psi = 0.5 * e(2);
    q(0) = cos(phi) * cos(th) * cos(psi) + sin(phi) * sin(th) * sin(psi);
    q(1) = sin(phi) * cos(th) * cos(psi) - cos(phi) * sin(th) * sin(psi);
    q(2) = cos(phi) * sin(th) * cos(psi) + sin(phi) * cos(th) * sin(psi);
    q(3) = cos(phi) * cos(th) * sin(psi) - sin(phi) * sin(th) * cos(psi);
}

/**
	@brief Converts an axis and angle into a unit quaternion

	@param[in] aa  axis/angle aa = (x, y, z), th = ||aa||, e = aa/th 
	@param[in] q   unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
*/
void Rot::axis_to_quat(const Eigen::Vector3d aa, Eigen::Vector4d& q) {
    double th = aa.norm();
    if (th < 0.0000001) {
        q(0) = 1.0;
        q(1) = 0.0;
        q(2) = 0.0;
        q(3) = 0.0;
    }
    else {
        Eigen::Vector3d e = aa.normalized();
	    q(0) = cos(th/2.0);
	    q(1) = e(0)*sin(th/2.0);
	    q(2) = e(1)*sin(th/2.0);
	    q(3) = e(2)*sin(th/2.0);
    }
}    

/**
	@brief Converts a unit quaternion into a normalized axis and angle.

	@param[in] q   unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] aa  axis/angle aa = (x, y, z), th = ||aa||, e = aa/th 
*/
void Rot::quat_to_axis(const Eigen::Vector4d q, Eigen::Vector3d& aa) {
	if (q(0) > 0.9999999) { // no rotation
		aa(0) = 0.0;
		aa(1) = 0.0;
		aa(2) = 0.0;
	}
	else {
    	double m = 2.0*acos(q(0))/sqrt(1.0 - q(0)*q(0));
    	aa(0) = m*q(1);
    	aa(1) = m*q(2);
    	aa(2) = m*q(3);
	}
}

/**
	@brief Composes two rotations parameterized by unit quaternions
    and outputs a single quaternion representing the composed rotation.
    This performs rotation p first and then rotation q after.

	@param[in] p   unit quaternion p = (pw, px, py, pz) = pw + (px i, py j, pz k) 
	@param[in] q   unit quaternion q = (qw, qx, qy, qz) = qw + (qx i, qy j, qz k) 
	@param[in] o   composed unit quaternion o = (ow, ox, oy, oz) = ow + (ox i, oy j, oz k) 
*/
void Rot::compose_quats(const Eigen::Vector4d p, const Eigen::Vector4d q, Eigen::Vector4d& o) {
	o(0) = p(0)*q(0) - (p(1)*q(1) + p(2)*q(2) + p(3)*q(3));
	o(1) = p(0)*q(1) + p(1)*q(0) + p(2)*q(3) - p(3)*q(2);
	o(2) = p(0)*q(2) - p(1)*q(3) + p(2)*q(0) + p(3)*q(1);
	o(3) = p(0)*q(3) + p(1)*q(2) - p(2)*q(1) + p(3)*q(0);
}

/**
	@brief Inverts a unit quaternion, gives the opposite rotation

	@param[in] q   unit quaternion q = (qw, qx, qy, qz) = qw + (qx i, qy j, qz k) 
*/
void Rot::invert_quat(Eigen::Vector4d& q) {
    q(1) *= -1.0;
    q(2) *= -1.0;
    q(3) *= -1.0;
}

double Rot::rad_to_deg(double th) {
    return th*(180.0/M_PI);
}

double Rot::deg_to_rad(double th) {
    return th*(M_PI/180.0);
}

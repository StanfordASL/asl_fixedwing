#ifndef ROTATION_HPP
#define ROTATION_HPP

/**
	@file rotation.hpp
	Utilities for reference frame rotations

	@brief Common tools for expressing and converting rotations.
*/

#include <Eigen/Dense>

namespace Rot {

void R_to_euler(const Eigen::Matrix3d R, Eigen::Vector3d& e);

void euler_to_R(const Eigen::Vector3d e, Eigen::Matrix3d& R);

void quat_to_R(const Eigen::Vector4d q, Eigen::Matrix3d& R);

void quat_to_euler(const Eigen::Vector4d q, Eigen::Vector3d& e);

void euler_to_quat(const Eigen::Vector3d e, Eigen::Vector4d& q);

void axis_to_quat(const Eigen::Vector3d aa, Eigen::Vector4d& q);

void quat_to_axis(const Eigen::Vector4d q, Eigen::Vector3d& aa);

void compose_quats(const Eigen::Vector4d p, const Eigen::Vector4d q, Eigen::Vector4d& o);

void invert_quat(Eigen::Vector4d& q);

double rad_to_deg(double th);

double deg_to_rad(double th);

double wrap_angle(double th);
}

#endif // ROTATION_HPP

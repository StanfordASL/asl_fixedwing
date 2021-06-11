#ifndef UTILS_HPP
#define UTILS_HPP

/**
    @file utils.hpp
    Defines useful utilities.

    @brief Defines useful utility functions.
*/
#include <asl_fixedwing/FloatVecStamped.h>

#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Utils {

void point_to_eigen3d(const geometry_msgs::PointStamped& p, Eigen::Vector3d& x);

void quat_to_eigen4d(const geometry_msgs::Quaternion& q, Eigen::Vector4d& x);

void vec3_to_eigen3d(const geometry_msgs::Vector3& v, Eigen::Vector3d& x);

void eigen3d_to_point(const Eigen::Vector3d x, geometry_msgs::PointStamped& p);

void eigen4d_to_quat(const Eigen::Vector4d& x, geometry_msgs::Quaternion& q);

void eigen3d_to_vec3(const Eigen::Vector3d& x, geometry_msgs::Vector3& v);

void eigenxd_to_floatvec(const Eigen::VectorXd& x, asl_fixedwing::FloatVecStamped& v);

}

#endif // PLANE_HPP

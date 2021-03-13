#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

namespace Utils {

// Data loading utilities
void save_matrix(const std::string& file_name, Eigen::MatrixXd matrix);

Eigen::MatrixXd load_matrix(const std::string& file_name);

Eigen::VectorXd load_vector(const std::string& file_name);
}

// Kinematic rotations utilities
void Utils::R_to_euler(const Eigen::Matrix3d R, Eigen::Vector3d& e);

void Utils::euler_to_R(const Eigen::Vector3d e, Eigen::Matrix3d& R);

void Utils::quat_to_R(const Eigen::Vector4d q, Eigen::Matrix3d& R);

void Utils::quat_to_euler(const Eigen::Vector4d q, Eigen::Vector3d& e);

void Utils::axis_to_quat(const double th, const Eigen::Vector3d a, Eigen::Vector3d& q);

void Utils::quat_to_axis(const Eigen::Vector3d q, double& th, Eigen::Vector3d& a);

void Utils::compose_quats(const Eigen::Vector3d p, const Eigen::Vector3d q, Eigen::Vector3d& o);

void Utils::invert_quat(const Eigen::Vector3d p, Eigen::Vector3d& q);

#endif // UTILS_HPP

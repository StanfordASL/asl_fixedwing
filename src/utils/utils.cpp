/**
    @file utils.cpp
    Defines useful utilities.
*/

#include <utils/utils.hpp>

void Utils::point_to_eigen3d(const geometry_msgs::PointStamped& p, Eigen::Vector3d& x) {
    x(0) = p.point.x; x(1) = p.point.y; x(2) = p.point.z;
}

void Utils::quat_to_eigen4d(const geometry_msgs::Quaternion& q, Eigen::Vector4d& x) {
    x(0) = q.w; x(1) = q.x; x(2) = q.y; x(3) = q.z;
}

void Utils::vec3_to_eigen3d(const geometry_msgs::Vector3& v, Eigen::Vector3d& x) {
    x(0) = v.x; x(1) = v.y; x(2) = v.z;
}

void Utils::eigen3d_to_point(const Eigen::Vector3d x, geometry_msgs::PointStamped& p) {
    p.point.x = x(0); p.point.y = x(1); p.point.z = x(2);
}

void Utils::eigen4d_to_quat(const Eigen::Vector4d& x, geometry_msgs::Quaternion& q) {
    q.w = x(0); q.x = x(1); q.y = x(2); q.z = x(3);
}

void Utils::eigen3d_to_vec3(const Eigen::Vector3d& x, geometry_msgs::Vector3& v) {
    v.x = x(0); v.y = x(1); v.z = x(2);
}

void Utils::eigenxd_to_floatvec(const Eigen::VectorXd& x, asl_fixedwing::FloatVecStamped& v) {
    v.data = std::vector<float>(x.data(), x.data() + x.size());
}

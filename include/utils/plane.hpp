#ifndef PLANE_HPP
#define PLANE_HPP

/**
    @file plane.hpp
    Defines a Plane class that contains some useful utilities.

	@brief Define a plane class that provides some useful tools
	for controller design, such as mappings from commands to
	actuator inputs.
*/

#include <Eigen/Dense>
#include <vector>

using vec = Eigen::Vector4d;

class Plane {
private:
	// Parameters for scaling commands
	Eigen::Vector3d _delta_0; // in order of [a, e, r]
	Eigen::Vector3d _c_delta; // in order of [a, e, r]
	double _c_T0;
	double _c_TVom;

public:
    Plane(const std::string& filename);
    void normalize_control(const vec& u, vec& u_nrmlzd);
};

#endif // PLANE_HPP

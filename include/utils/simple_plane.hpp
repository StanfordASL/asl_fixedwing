#ifndef SIMPLE_PLANE_HPP
#define SIMPLE_PLANE_HPP

/**
    @file simple_plane.hpp
    Defines a Plane class that contains some useful utilities.

	@brief Define a plane class that provides some useful tools
	for controller design, such as mappings from commands to
	actuator inputs. This is a simplified version of the Plane
    class that assumes body rate commands instead of control surface
    deflection.
*/

#include <Eigen/Dense>
#include <vector>

using vec = Eigen::Vector4d;

class SimplePlane {
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

/** 
    @file simple_plane.cpp
    Definition of Plane class to provide useful utilities. This plane
    class assumes the control inputs are the thrust commands and
    the body rate commands.
*/

#include <utils/simple_plane.hpp>
#include <utils/data.hpp>

#include <cmath>

/** 
	@brief Constructor which loads parameters from csv file.

	@param[in] filepath  path to directory containing relevant .csv files, which
	should store in the order: [c_T0, c_TVom]

*/
void SimplePlane::SimplePlane(const std::string& filepath) {

	// Load nominal state and control 
	_x_0 = Data::load_vector(filepath + "/x_eq.csv");
    _u_0 = Data::load_vector(filepath + "/u_eq.csv");

	// Load relevant control parameters from csv file
	Eigen::Vector8d ctrl_params = Data::load_vector(filepath + "/ctrl_params.csv");
	_c_T0 = ctrl_params(0);
	_c_TVom = ctrl_params(1);
	
}

/**
	@brief Converts the full control u = [T, p, q, r] given in units
	[N, rad/s, rad/s, rad/s] to appropriate quantities that are commanded
	through MAVROS.

	@param[in] u         dimensional control vector [T, p, q, r]
	@param[in] u_nrmlzd  normalized commands
	@param[in] V         rel. airspeed parallel to body x axis [m/s]
*/
void Plane::normalize_control(const vec& u, vec& u_nrmlzd, const double V) {
    u_nrmlzd(0) = Plane::nrmlz_thrust_cmd(u(0), V);
	u_nrmlzd(1) = Plane::nrmlz_ctrl_srf_cmd(u(1), 0);
	u_nrmlzd(2) = Plane::nrmlz_ctrl_srf_cmd(u(2), 1);
	u_nrmlzd(3) = Plane::nrmlz_ctrl_srf_cmd(u(3), 2);
}

/**
	@brief Converts a commanded control surface deflection angle
	into a normalized [-1,1] command that can be sent over
	actuator_control MAVROS topics. Assumes a linear mapping.

	@param[in]  cmd  control surface deflection cmd [radians]
	@param[in]  i    index int 0=a, 1=e, 2=r
	@param[out] normalized command in range [-1, 1]
*/
double Plane::nrmlz_ctrl_srf_cmd(const double delta, const int i) {
	return min(1.0, max(-1.0, (delta - _delta_0(i))/_c_delta(i) ));
}

/**
    @brief Converts a commanded thrust value into a normalized 
	[0,1] command that can be sent over actuator_control MAVROS 
	topics. Assumes a quadratic mapping from cmd to T.

    @param[in]  cmd  thrust command [N]
	@param[in]  V    rel. windspeed parallel to body x axis [m/s]
    @param[out] normalized command in range [0, 1]
*/
double Plane::nrmlz_thrust_cmd(const double T, const double V) {
    return min(1.0, max(0.0, 0.5*(_c_TVom*V + 
				sqrt( (_c_TVom*V)*(_c_TVom*V) + 4.0*T/_c_T0 )) ));
}

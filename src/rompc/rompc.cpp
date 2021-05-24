/**
	@file rompc.cpp
	Implementation of the ROMPC control scheme.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <utils/data.hpp>

#include <string>
#include <iostream>

/** Constructor */
ROMPC::ROMPC(const std::string& filepath) {
	// Load controller parameters from file
	_A = Data::load_matrix(filepath + "/A.csv");
	_B = Data::load_matrix(filepath + "/B.csv");
	_C = Data::load_matrix(filepath + "/C.csv");
	_H = Data::load_matrix(filepath + "/H.csv");
	_K = Data::load_matrix(filepath + "/K.csv");
	_L = Data::load_matrix(filepath + "/L.csv");
}

/** Initialize controller internal state */
void ROMPC::init(const Eigen::VectorXd x0, const double t0) {
	_xhat = x0;
	_xbar = x0;
	_t = t0;
}

/** Get the current control value u at the current time */ 
void ROMPC::update(const Eigen::VectorXd y, const double t) {
	double dt = t - _t; // time step since last update

    // Update state estimate from previous step
    _xhat += dt*(_A*_xhat + _B*_u + _L*(_y - _C*_xhat));

    // Update simulated ROM TODO
    //_xbar = _xbar;


	// Get simulated ROM state and control at time t
	ROMPC::update_sim_rom(t);

	// Control law
	_u = _ubar + _K*(_xhat - _xbar);

	// Save current measurement
	_y = y;
}

/** Update the simulated ROM state and control TODO */
void ROMPC::update_sim_rom(const double t) {
	_ubar.setZero();
	_xbar.setZero();
}

/** Get the current control */
Eigen::VectorXd ROMPC::get_u() {
	return _u;
}

/** Get the current state estimate */
Eigen::VectorXd ROMPC::get_xhat() {
    return _xhat;
}

/** Get the current nominal ROM state */
Eigen::VectorXd ROMPC::get_xbar() {
    return _xbar;
}


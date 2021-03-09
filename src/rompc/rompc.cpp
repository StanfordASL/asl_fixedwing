/**
	@file rompc.cpp
	Implementation of the ROMPC control scheme.

	@brief Does stuff.
*/

#include <rompc/rompc.hpp>
#include <string>

/** Constructor */
ROMPC::ROMPC(std::string filename) {
	// Load controller parameters from file
	
}

/** Initialize controller internal state */
void ROMPC::init(const Eigen::VectorXd x0) {
	_xhat = x0;
	_xbar = x0;
}

/** Get the current control value u at the current time */ 
Eigen::VectorXd ROMPC::control(const Eigen::VectorXd y, const double t) {
	// Get simulated ROM state and control at time t
	ROMPC::update_sim_rom(t);

	// Control law
	_u = _ubar + _K*(_xhat - _xbar);

	// Update state estimator
	_xhat = _A*_xhat + _B*_u + _L*(y - _C*_xhat);

	// Update simulated ROM TODO
	_xhat = _xhat;

	return _u;
}

/** Update the simulated ROM state and control TODO */
void ROMPC::update_sim_rom(const double t) {
	_ubar.setZero();
	_xbar.setZero();
}

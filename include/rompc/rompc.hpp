#ifndef ROMPC_CTRL_HPP
#define ROMPC_CTRL_HPP

/**
	@file rompc.hpp
	Header file defining the ROMPC controller class.

	@brief ROMPC controller class.
*/
#include <iostream>
#include <Eigen/Dense>

class ROMPC {
private:
	Eigen::MatrixXd _K; // controller gain matrix
	Eigen::MatrixXd _L; // estimator gain matrix
	Eigen::MatrixXd _A; // system dynamics ...
	Eigen::MatrixXd _B; // ... matrices

	Eigen::VectorXd _xbar; // nominal ROM state
	Eigen::VectorXd _ubar; // nominal ROM control
	Eigen::VectorXd _xhat; // ROM state estimate
	Eigen::VectorXd _u; // control to be applied

	Eigen::VectorXd _x_eq; // linearization state
	Eigen::VectorXd _u_eq; // linearization control

	void update_sim_rom(const double t);

public:
	void init(const Eigen::VectorXd x0);
	Eigen::VectorXd control(const Eigen::VectorXd y, const double t); 
};

#endif // ROMPC_CTRL_HPP

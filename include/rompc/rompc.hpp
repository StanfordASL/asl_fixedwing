#ifndef ROMPC_CTRL_HPP
#define ROMPC_CTRL_HPP

/**
	@file rompc.hpp
	Header file defining the ROMPC controller class.

	@brief ROMPC controller class.
*/
#include <string>
#include <Eigen/Dense>

class ROMPC {
private:
	Eigen::MatrixXd _K; // controller gain matrix
	Eigen::MatrixXd _L; // estimator gain matrix
	Eigen::MatrixXd _A; // system dynamics ...
	Eigen::MatrixXd _B; // ...
	Eigen::MatrixXd _C; // ...
	Eigen::MatrixXd _H; // ... matrices

	Eigen::VectorXd _xbar; // nominal ROM state
	Eigen::VectorXd _ubar; // nominal ROM control
	Eigen::VectorXd _xhat; // ROM state estimate
	Eigen::VectorXd _u; // control to be applied
	Eigen::VectorXd _y; // measurement

	double _t; // time (s)

	void update_sim_rom(const double t);

public:
	ROMPC(const std::string& filename);
	void init(const Eigen::VectorXd x0, const double t0);
	void update(const Eigen::VectorXd y, const double t); 
	Eigen::VectorXd get_u();
	Eigen::VectorXd get_xhat();
	Eigen::VectorXd get_xbar();
};

#endif // ROMPC_CTRL_HPP

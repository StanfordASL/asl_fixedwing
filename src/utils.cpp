/**
	@file utils.cpp
	Helper functions
*/

#include <utils.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <math.h>

/** Save matrix to .csv file. */
void Utils::save_matrix(const std::string& file_name, Eigen::MatrixXd matrix) {
	const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, 
										Eigen::DontAlignCols, ", ", "\n");
	std::ofstream file(file_name); // open a file to write
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

/** Load matrix from .csv file */
Eigen::MatrixXd Utils::load_matrix(const std::string& file_name) {
	std::ifstream file(file_name); // open a file to read
	if (!file.good()) {
		std::cerr << "File: " << file_name << " does not exist" << std::endl;
		exit(1);
	}
	
	// Read through file and extract all data elements
	std::string row;
	int i = 0; // row counter
	std::string entry;
	std::vector<double> entries;
	while (std::getline(file, row)) {
		std::stringstream row_stream(row);
		while (std::getline(row_stream, entry, ',')) {
			entries.push_back(std::stod(entry));
		}
		i++; // increment row counter
	}

	// Convert vector into matrix of proper shape
	return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(entries.data(), i, entries.size() / i);
}

/** Load vector from .csv file */
Eigen::VectorXd Utils::load_vector(const std::string& file_name) {
    Eigen::MatrixXd M = Utils::load_matrix(file_name);
    return Eigen::Map<Eigen::VectorXd>(M.data(), M.rows());



/**
All of the rotation parameterizations here represent rotations
of an initial fixed frame to a new frame. Therefore using these
parameterizations to operate on a vector is the same as transforming
the new frame coordinates into the original fixed frame.

Examples if rotation describes relation of body w.r.t. inertial
(how to rotate inertial to body):
R: coverts body frame coordinates into inertial frame coordinates
euler: usual roll-pitch-yaw of body frame w.r.t inertial frame
*/

/** 
	@brief Converts the rotation matrix R to the Euler angles (ZYX)
	(Yaw-Pitch-Roll) (psi, theta, phi) used for aircraft conventions.
	Note to compute the Euler angles for the aircraft this should be
	the R matrix that converts a vector from body frame coordinates
	to a vector in inertial frame coordinates.

	@param[in] R  rotation matrix
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)
*/
void Utils::R_to_euler(const Eigen::Matrix3d R, Eigen::Vector3d& e) {
	// Check for singularity that occurs when pitch = 90 deg
	if (sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)) >= 1e-6) {
		e(0) = atan2(R(2,1), R(2,2));
	    e(1) = asin(-R(2,0));
	    e(2) = atan2(R(1,0), R(0,0));
	}
	else {
		e(0) = atan2(-R(1,2), R(1,1));
	    e(1) = asin(-R(2,0));
	    e(2) = 0.0;
	}	
}

/**
    @brief Converts Euler angles (ZYX) (Yaw-Pitch-Roll) (psi, theta, phi) 
    into the inertial to body rotation matrix
	
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)
	@param[in] R  rotation matrix
*/
void Utils::euler_to_R(const Eigen::Vector3d e, Eigen::Matrix3d& R) {
	R(0,0) = cos(e(1))*cos(e(2));
	R(0,1) = sin(e(0))*sin(e(1))*cos(e(2)) - cos(e(0))*sin(e(2));
	R(0,2) = sin(e(0))*sin(e(2)) + cos(e(0))*sin(e(1))*cos(e(2));
	R(1,0) = cos(e(1))*sin(e(2));
	R(1,1) = cos(e(0))*cos(e(2)) + sin(e(0))*sin(e(1))*sin(e(2));
	R(1,2) = cos(e(0))*sin(e(1))*sin(e(2)) - sin(e(0))*cos(e(2));
	R(2,0) = -sin(e(1));
	R(2,1) = sin(e(0))*cos(e(1));
	R(2,2) = cos(e(0))*cos(e(1));
}

/**
	@brief Converts a unit quaternion into a rotation matrix

	@param[in] q  unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] R  rotation matrix
*/
void Utils::quat_to_R(const Eigen::Vector4d q, Eigen::Matrix3d& R) {
	R(0,0) = 1.0 - 2.0*q(2)*q(2) - 2.0*q(3)*q(3);
	R(0,1) = 2.0*q(1)*q(2) - 2.0*q(3)*q(0);
	R(0,2) = 2.0*q(1)*q(3) + 2.0*q(2)*q(0);
	R(1,0) = 2.0*q(1)*q(2) + 2.0*q(3)*q(0);
	R(1,1) = 1.0 - 2.0*q(1)*q(1) - 2.0*q(3)*q(3);
	R(1,2) = 2.0*q(2)*q(3) - 2.0*q(1)*q(0);
	R(2,0) = 2.0*q(1)*q(3) - 2.0*q(2)*q(0);
	R(2,1) = 2.0*q(2)*q(3) + 2.0*q(1)*q(0);
	R(2,2) = 1.0 - 2.0*q(1)*q(1) - 2.0*q(2)*q(2);
}

/**
	@brief Converts a unit quaternion into the aircraft Euler angles
	(ZYX) (Yaw-Pitch-Roll) (psi, theta, phi)

	@param[in] q  unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] e  euler angles e = (phi, theta, psi) = (roll, pitch, yaw)

*/
void Utils::quat_to_euler(const Eigen::Vector4d q, Eigen::Vector3d& e) {
	double R00 = 1.0 - 2.0*q(2)*q(2) - 2.0*q(3)*q(3);
    double R10 = 2.0*q(1)*q(2) + 2*q(3)*q(0);
    if (sqrt(R00*R00 + R10*R10 >= 1e-6)) {
	    e(0) = atan2(2.0*q(2)*q(3) + 2.0*q(1)*q(0), 1.0 - 2.0*q(1)*q(1) - 2.0*q(2)*q(2));
	    e(1) = asin(-2.0*q(1)*q(3) + 2.0*q(2)*q(0));
	    e(2) = atan2(R10, R00);
    }
    else {
    	e(0) = atan2(-2.0*q(2)*q(3) - 2.0*q(1)*q(0), 1.0 - 2.0*q(1)*q(1) - 2.0*q(3)*q(3));
        e(1) = asin(-2.0*q(1)*q(3) + 2.0*q(2)*q(0));
        e(2) = 0.0
    }
}

/**
	@brief Converts an axis and angle into a unit quaternion

	@param[in] th  angle
	@param[in] a   axis a = (x, y, z) 
	@param[in] q   unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
*/
void Utils::axis_to_quat(const double th, const Eigen::Vector3d a, Eigen::Vector3d& q) {
	a.normalize();
	q(0) = cos(th/2.0);
	q(1) = a(0)*sin(th/2.0);
	q(2) = a(1)*sin(th/2.0);
	q(3) = a(2)*sin(th/2.0);
}
    

/**
	@brief Converts a unit quaternion into a normalized axis and angle.

	@param[in] q   unit quaternion q = (w, x, y, z) = w + (x i, y j, z k) 
	@param[in] th  angle
	@param[in] a   axis a = (x, y, z) 
*/
void Utils::quat_to_axis(const Eigen::Vector3d q, double& th, Eigen::Vector3d& a) {
    th = 2.0*acos(q(0));
    double d = sqrt(1.0 - q(0)*q(0));
    a(0) = q(1)/d;
    a(1) = q(2)/d;
    a(2) = q(3)/d;
}

/**
	@brief Composes two rotations parameterized by unit quaternions
    and outputs a single quaternion representing the composed rotation.
    This performs rotation p first and then rotation q after.

	@param[in] p   unit quaternion p = (pw, px, py, pz) = pw + (px i, py j, pz k) 
	@param[in] q   unit quaternion q = (qw, qx, qy, qz) = qw + (qx i, qy j, qz k) 
	@param[in] o   composed unit quaternion o = (ow, ox, oy, oz) = ow + (ox i, oy j, oz k) 
*/
void Utils::compose_quats(const Eigen::Vector3d p, const Eigen::Vector3d q, Eigen::Vector3d& o) {
	o(0) = p(0)*q(0) - (p(1)*q(1) + p(2)*q(2) + p(3)*q(3));
	o(1) = p(0)*q(1) + p(1)*q(0) + p(2)*q(3) - p(3)*q(2);
	o(2) = p(0)*q(2) - p(1)*q(3) + p(2)*q(0) + p(3)*q(1);
	o(3) = p(0)*q(3) + p(1)*q(2) - p(2)*q(1) + p(3)*q(0);
}

/**
	@brief Inverts a unit quaternion, gives the opposite rotation

	@param[in] p   unit quaternion p = (pw, px, py, pz) = pw + (px i, py j, pz k) 
	@param[in] q   inverted unit quaternion q = (qw, qx, qy, qz) = qw + (qx i, qy j, qz k) 
*/
void Utils::invert_quat(const Eigen::Vector3d p, Eigen::Vector3d& q) {
    q(0) = p(0);
    q(1) = -p(1);
    q(2) = -p(2);
    q(3) = -p(3);
}

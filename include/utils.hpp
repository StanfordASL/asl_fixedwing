#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

namespace Utils {

void save_matrix(std::string file_name, Eigen::MatrixXd matrix);

Eigen::MatrixXd load_matrix(std::string file_name);

}

#endif // UTILS_HPP

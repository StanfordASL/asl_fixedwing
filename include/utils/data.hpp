#ifndef DATA_HPP
#define DATA_HPP

/**
    @file data.hpp
    Utilities for loading and saving data
*/

#include <Eigen/Dense>
#include <string>

namespace Data {

void save_matrix(const std::string& file_name, Eigen::MatrixXd matrix);

Eigen::MatrixXd load_matrix(const std::string& file_name);

Eigen::VectorXd load_vector(const std::string& file_name);

}

#endif // DATA_HPP

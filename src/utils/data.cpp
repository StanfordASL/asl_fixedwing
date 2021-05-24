/**
    @file data.cpp
    Helper functions for saving and loading data
*/

#include <utils/data.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

/** Save matrix to .csv file. */
void Data::save_matrix(const std::string& file_name, Eigen::MatrixXd matrix) {
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
Eigen::MatrixXd Data::load_matrix(const std::string& file_name) {
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
Eigen::VectorXd Data::load_vector(const std::string& file_name) {
    Eigen::MatrixXd M = Data::load_matrix(file_name);
    return Eigen::Map<Eigen::VectorXd>(M.data(), M.rows());
}

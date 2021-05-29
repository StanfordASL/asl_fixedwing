/** 
	@file rompc_utils.cpp
	Utility functions to support the ROMPC controller.
*/

#include <rompc/rompc_utils.hpp>

/**
    @brief Constructor, initialize variables to zero
*/
ROMPC_UTILS::Target::Target() {
    _p_r_i_I.setZero();
    _v_r_I_R.setZero();
    _q_I_to_R.setZero();
    _om_R_I_R.setZero();
}

/**
    @brief Gets the position of the target frame relative to inertial
    in I coordinates, at time t from when initialized

    @param[in] t  time (seconds) to query target
*/
ROMPC_UTILS::Target::get_pos(double t);

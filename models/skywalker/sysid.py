"""
System identification for Skywalker plane
"""
import numpy as np

def aircraft_ctrl_params():
    """
    Compute the parameters that define the aircraft's command to control mappings
    
    delta = c_delta*u_delta + delta_0 maps [-1,1] to radians
    T = c_T0*(1 - C_TVom*V/u_T)*u_T^2 maps [0,1] to Newtons
    """

    # TODO Thrust model parameters
    c_T0 = 0.0
    c_TVom = 0.0

    # TODO Determine mapping for control surface deflection
    a_0 = 0.0
    e_0 = 0.0
    r_0 = 0.0
    c_a = 1.0
    c_e = 1.0
    c_r = 1.0
    
    return np.array([c_T0, c_TVom, a_0, c_a, e_0, c_e, r_0, c_r])

if __name__ == '__main__':
    aircraft_params()
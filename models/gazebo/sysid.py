"""
Servo sysid data from running sysid_actuator_control.launch

ROLL = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # roll commands in (-1,1)
A = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.53, 0.53, 0.53, 0.53, 0.53] # aileron deflection angles (radians)

PITCH = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # pitch commands in (-1,1)
E = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.53, 0.53, 0.53, 0.53, 0.53] # elevator deflection angle (radians)

YAW = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # yaw commands in (-1,1)
R = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.53, 0.53, 0.53, 0.53, 0.53] # rudder deflection angle (radians)

THRUST = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # thrust commands in (0,1)
OM = [0.0, 35.0, 70.0, 105.0, 140.0, 175.0, 210.0, 245.0, 280.0, 315.0, 350.0] # propeller rotation speed (radians/s)

"""
import numpy as np

def aircraft_ctrl_params():
	"""
	Compute the parameters that define the aircraft's command to control mappings
	
	delta = c_delta*u_delta + delta_0 maps [-1,1] to radians
	T = c_T0*(1 - C_TVom*V/u_T)*u_T^2 maps [0,1] to Newtons
	"""

	k_slowdown = 10.0 # from .sdf file
	k_motor = 8.54858e-06 # from .sdf file
	c_om = 350.0 # from sysid data

	# Thrust model parameters
	c_T0 = k_motor*(c_om*k_slowdown)**2
	c_TVom = 0.2/25.0 # a guess because Gazebo doesn't model prop well

	# Control surface deflection parameters
	a_0 = 0.0
	e_0 = 0.0
	r_0 = 0.0
	c_a = 1.0
	c_e = 1.0
	c_r = 1.0
	
	return np.array([c_T0, c_TVom, a_0, c_a, e_0, c_e, r_0, c_r])

if __name__ == '__main__':
	aircraft_params()

This model is a CFD/rigid-body model with dimension 100. The state is
x = [fluid, pos_rate, rot_rate, pos, rot, ctrl_surf_def] where all
terms are perturbations from a nominal glideslope trajectory with a
total speed of S = 15 m/s and glideslope angle of -3.5 degrees. These
nominal values define the target trajectory, and are saved in 
target.csv as [S, gamma, th] where th is the pitch angle at 
equilibrium. Note that the rotation is parameterized as axis/angle
parameters in radians.

The control is u = [T, ad, ed, rd] where T is thrust in N, and ad, ed, rd are aileron, elevator, rudder deflection rates in rad/s. These are also perturbation values, and the equilibrium control is given in u_eq.csv. Positive aileron is roll right, positive elevator is pitch up, positive rudder is yaw right.

The output matrix C and H define y = z = [pos_rate, rot_rate, pos, rot, ctrl_surf_def].

The equilibrium values (thrust and pitch angle) are defined for an altitude of 30 meters, but do not change too significantly with altitude.

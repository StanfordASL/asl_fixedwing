"""
    @file rompc_synth.py
    Computes required components for ROMPC controller
    given the linear ROM matrices A, B, C, H
"""
import sys
from os.path import dirname, abspath, join, isdir
from os import mkdir
import numpy as np
from scipy.linalg import solve_continuous_are


if __name__ == '__main__':
    if len(sys.argv) <= 2 or sys.argv[1] not in ['sgf', 'slf', 'stf'] \
            or sys.argv[2] not in ['ctrl_surf', 'body_rate']:
        print('Correct usage: python rompc_synth.py {sgf, slf, stf} {ctrl_surf, body_rate}')
        sys.exit()

    if sys.argv[1] in ['sgf', 'stf']:
        print('SGF and STF not yet supported')
        sys.exit()

    # Generate save directory if it doesn't exist
    root = dirname(dirname(dirname(dirname(abspath(__file__)))))
    path = join(root, "src/rompc/skywalker")
    savepath = join(path, sys.argv[1])
    if not isdir(savepath):
        mkdir(savepath)

    # Append path to load the models
    sys.path.append(join(root, "models/skywalker"))
    from sysid import aircraft_ctrl_params

    # Get additional aircraft parameters
    p = aircraft_ctrl_params()
    np.savetxt(join(savepath, "ctrl_params.csv"), p, delimiter=",");

from os.path import join
from matplotlib import pyplot as plt
import sys
import numpy as np
import pdb

from utils import get_data_dir, RosbagData

if __name__ == '__main__':
    data_dir = get_data_dir()
    fpath = join(data_dir, 'rompc.bag')
    data = RosbagData(fpath)

    fig, axs = plt.subplots(5, 1, sharex='col')
    axs[0].plot(data.rompc.e_pos.t, data.rompc.e_pos.x, 'b.')
    # axs[0].set_xlim([data.t0 - 5, ...])
    
    plt.show()
    pdb.set_trace()



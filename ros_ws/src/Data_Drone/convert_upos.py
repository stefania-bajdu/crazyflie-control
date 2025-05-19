import numpy as np
from scipy.io import savemat

data_Upos = np.load('src/crazyflie_control/crazyflie_control/Upos.npy', allow_pickle=True).tolist()

A_vc = np.round(data_Upos['A'][0, 0], 5)
b_vc = np.round(data_Upos['b'][0, 0], 5)

savemat('src/Data_Drone/matlab_scripts/mats/Upos.mat', {'A_vc': A_vc, 'b_vc': b_vc})

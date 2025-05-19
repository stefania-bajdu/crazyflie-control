import numpy as np
from scipy.io import loadmat

"""
This file provides some methods to generate reference trajectory for drone.
A reference trajectory is defined as a dictionary with following information:
ref = {
        "trajectory":   Position and velocity [[x0, y0, z0, vx0, vy0, vz0],
                                                [x1, y1, z1, vx1, vy1, vz1]],
                                                [x2, y2, z2, vx2, vy2, vz2]],
                                                ...
                                                [x(Nsim), y(Nsim), z(Nsim), vx(Nsim), vy(Nsim), vz(Nsim)]],
        "time_step":    Time stamp [0, Ts, 2*Ts, ..., Nsim*Ts],
        "thrust":       Thrust force [m/s²],
        "phi":          Roll angle [rad],
        "theta":        Pitch angle [rad],
        "Nsim":         Simulation step,
        "v_ref":        Nominal control input 
                        ~ Reference Acceleration [[ax0, ay0, az0],
                                                    [ax1, ay1, az1],
                                                    [ax2, ay2, az2],
                                                    ...
                                                    [ax(Nsim), ay(Nsim), az(Nsim)]]
        }

"""


def get_ref_setpoints_stage(psi, Tsim, dt):
    knot = [0, Tsim]
    g = 9.81
    tt = np.arange(min(knot), max(knot), dt)
    # with open('khanh.mat', 'rb') as inp:
    #     sim_data = pickle.load(inp)
    sim_data = loadmat('./Trajectory/traj.mat')
    W = sim_data['valeurs']

    k_pass = 1
    ref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(W[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            ref_tmp = np.vstack((ref_tmp, cur))
            k_pass = k_pass + 1

    dW = sim_data['velocity_f']

    k_pass = 1
    vref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), dW.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(dW[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            vref_tmp = np.vstack((vref_tmp, cur))
            k_pass = k_pass + 1

    ref_full = np.block([
        [ref_tmp, vref_tmp]
    ])
    v_ref = 0 * ref_tmp.transpose()
    ddx, ddy, ddz = v_ref[0, :], v_ref[1, :], v_ref[2, :]
    thrust = np.sqrt(ddx ** 2 + ddy ** 2 + (ddz + 9.81) ** 2)
    phi = np.arcsin((ddx * np.sin(psi) - ddy * np.cos(psi)) / thrust)
    theta = np.arctan((ddx * np.cos(psi) + ddy * np.sin(psi)) / (ddz + g))
    ref = {
        "trajectory": ref_full,
        "time_step": tt,
        "thrust": thrust,
        "phi": phi,
        "theta": theta,
        "Nsim": tt.shape[0],
        "v_ref": v_ref.transpose()}

    return ref


def get_ref_setpoints(psi, Tsim, dt, version=1):
    knot = [0, Tsim]
    g = 9.81
    tt = np.arange(min(knot), max(knot), dt)
    if version == 1:
        W = np.array([[-0.4],
                      [-0.4],
                      [0.3]  # 3D test
                      ])
    elif version == 2:
        W = np.array([[],
                      [-0.4],
                      [0.3]  # 3D test
                      ])
    elif version == 3:
        W = np.array([[-0.4],
                      [0],
                      [0.3]  # 3D test
                      ])
    elif version == 4:
        W = np.array([[0.5],
                      [0.5],
                      [0.9]  # 3D test
                      ])
    elif version == 0:
        W = np.array([[-0.65],
                      [-0.65],
                      [0.7]  # 3D test
                      ])
    elif version == 5:
        W = np.array([[-0.2, -0.2, -0.2, -0.2, 0.3, 0.8, 0.8, 0.8, 0.3, -0.2, -0.2, -0.2],
                      [0, 0, 0, 0, 0.25, 0.5, 0.5, 0.5, 0.25, 0, 0, 0],
                      [0.15, 0.3, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.2, 0.15]  # 3D test
                      ])
    elif version == 6:
        W = np.array([[-0.2, -0.2, -0.2, -0.2, 0.8, 0.8, 0.8, 0.8, 0.8, -0.2, -0.2, -0.2],
                      [0,    0,      0,   0,  0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0],
                      [0.2,  0.2,    0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2]  # 3D test
                      ])
    elif version == 11:
        W = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0.2, 0.7, 0.7, 0.7]  # 3D test
                      ])
    elif version == 12:
        W = np.array([[-0.5, -0.5, -0.5, -0.5],
                      [0, 0, 0, 0],
                      [0.2, 0.7, 0.7, 0.7]  # 3D test
                      ])
    elif version == 13:
        W = np.array([[0.5, 0.5, 0.5, 0.5],
                      [0, 0, 0, 0],
                      [0.2, 0.7, 0.7, 0.7]  # 3D test
                      ])
    k_pass = 1
    ref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(W[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            ref_tmp = np.vstack((ref_tmp, cur))
            k_pass = k_pass + 1

    ref_full = np.block([
        [ref_tmp, ref_tmp * 0]
    ])
    v_ref = 0 * ref_tmp.transpose()
    # v_ref[2, :] = v_ref[2, :] - 0.075
    # v_ref[0, :] = v_ref[0, :] - 0.125
    ddx, ddy, ddz = v_ref[0, :], v_ref[1, :], v_ref[2, :]
    thrust = np.sqrt(ddx ** 2 + ddy ** 2 + (ddz + 9.81) ** 2)
    phi = np.arcsin((ddx * np.sin(psi) - ddy * np.cos(psi)) / thrust)
    theta = np.arctan((ddx * np.cos(psi) + ddy * np.sin(psi)) / (ddz + g))
    ref = {
        "trajectory": ref_full,
        "time_step": tt,
        "thrust": thrust,
        "phi": phi,
        "theta": theta,
        "Nsim": tt.shape[0],
        "v_ref": v_ref.transpose()}

    return ref


def get_trajectory_mat(path_to_file, dt, psi=0):
    g = 9.81
    sim_data = loadmat(path_to_file)
    x = sim_data["x"] - 0.4
    y = sim_data["y"] + 0.4
    z = sim_data["z"] + 0.30

    vx = sim_data["vx"]
    vy = sim_data["vy"]
    vz = sim_data["vz"]

    ax = sim_data["ax"]
    ay = sim_data["ay"]
    az = sim_data["az"]

    nbr_agents = np.size(x, 0)
    Tsim = dt*(np.size(x, 1))

    ref_full = {}
    for i in range(nbr_agents):
        ref_full[i] = np.vstack((x[i, :], y[i, :], z[i, :], vx[i, :], vy[i, :], vz[i, :])).transpose()

    v_ref = {}
    for i in range(nbr_agents):
        v_ref[i] = np.vstack((ax[i, :], ay[i, :], az[i, :])).transpose()

    tt = np.arange(0, Tsim, dt)

    thrust = {}
    phi = {}
    theta = {}
    for i in range(nbr_agents):
        thrust[i] = np.sqrt(ax[i, :] ** 2 + ay[i, :] ** 2 + (az[i, :] + 9.81) ** 2)
        phi[i] = np.arcsin((ax[i, :] * np.sin(psi) - ay[i, :] * np.cos(psi)) / thrust[i])
        theta[i] = np.arctan((ax[i, :] * np.cos(psi) + ay[i, :] * np.sin(psi)) / (az[i, :] + g))

    ref = {}
    for i in range(nbr_agents):
        ref[i] = {
            "trajectory": ref_full[i],
            "time_step": tt,
            "thrust": thrust[i],
            "phi": phi[i],
            "theta": theta[i],
            "Nsim": tt.shape[0],
            "v_ref": v_ref[i]}

    return ref


def get_ref_setpoints_takeoff(psi, Tto, dt, ref):
    knot = [0, Tto]
    g = 9.81
    tt = np.arange(min(knot), max(knot), dt)

    print(tt)
    k_pass = 1
    dest = ref[0, 0:3].reshape(-1, 1)
    nbr_step = 2
    W12 = np.repeat(dest[:2], nbr_step, axis=1)
    W3 = np.repeat(dest[2], nbr_step, axis=0)
    # W3 = np.linspace(0.15,dest[2],nbr_step).reshape((1,nbr_step))
    W = np.vstack((W12, W3))

    print(W)
    ref_tmp = np.empty((0, 3))
    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1] + 1)
    for i_tmp in range(waypoint_time_stamps.shape[0] - 1):
        cur = np.array(W[:, i_tmp])
        while dt * k_pass <= waypoint_time_stamps[i_tmp + 1]:
            ref_tmp = np.vstack((ref_tmp, cur))
            k_pass = k_pass + 1

    ref_full = np.block([[ref_tmp, ref_tmp * 0]])
    v_ref = 0 * ref_tmp.transpose()
    ddx, ddy, ddz = v_ref[0, :], v_ref[1, :], v_ref[2, :]
    thrust = np.sqrt(ddx ** 2 + ddy ** 2 + (ddz + 9.81) ** 2)
    phi = np.arcsin((ddx * np.sin(psi) - ddy * np.cos(psi)) / thrust)
    theta = np.arctan((ddx * np.cos(psi) + ddy * np.sin(psi)) / (ddz + g))
    ref = {
        "trajectory": ref_full,
        "time_step": tt,
        "thrust": thrust,
        "phi": phi,
        "theta": theta,
        "Nsim": tt.shape[0],
        "v_ref": v_ref.transpose()}

    return ref

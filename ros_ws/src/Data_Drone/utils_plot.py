import numpy as np
import scipy.io
import matplotlib.pyplot as plt


# def save_to_mat(filename, t, state,  pos_ref):
#     data = {'t': t, 'state_xi': state, 'pos_ref': pos_ref}
#     scipy.io.savemat(filename, data)
#     print(f"Data saved to {filename}.")


def save_to_mat(filename, data_dict):
    scipy.io.savemat(filename, data_dict)


def load_from_mat(filename):
    data = scipy.io.loadmat(filename)
    t = data['t'].flatten()
    state_xi = data['state_xi']
    pos_ref = data['pos_ref']
    return t, state_xi, pos_ref


def save_data(file_dest, data, ref):
    data = {'data': data, 'ref': ref}
    np.save(file_dest, data)

    import numpy as np


def quaternion_to_euler(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Clamping to 90 degrees
    else:
        pitch = np.arcsin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


def plot_positions(t, state_xi, pos_ref, id=0, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['x (m)', 'y (m)', 'z (m)']
    for i in range(3):
        axs[i].plot(t, state_xi[:, i], label='sim')
        axs[i].plot(t, pos_ref[:, i], '--', label='ref')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Positions', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def pose_with_ref_velocity(t_pose, state_xi, t_ref, pos_ref, id=0, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['vx (m/s)', 'vy (m/s)', 'vz (m/s)']
    for i in range(3):
        axs[i].plot(t_pose, state_xi[:, i], label='sim')
        axs[i].plot(t_ref, pos_ref[:, i], '--', label='ref')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Velocities', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_velocity(t, state_xi, pos_ref, id=0, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['vx (m/s)', 'vy (m/s)', 'vz (m/s)']
    for i in range(3):
        axs[i].plot(t, state_xi[:, i], label='sim')
        axs[i].plot(t, pos_ref[:, i], '--', label='ref')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Velocities', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_v(t, v, vref, id=0, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['ax (m^2/s)', 'ay (m^2/s)', 'az (m^2/s)']
    for i in range(3):
        axs[i].plot(t, v[:, i], label='sim')
        axs[i].plot(t, vref[:, i], '--', label='ref')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Accelerations', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_pose_velocity(t, state_xi, id=0, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['vx (m/s)', 'vy (m/s)', 'vz (m/s)']
    for i in range(3):
        axs[i].plot(t, state_xi[:, i], label='sim')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Pose Velocities at 100 Hz', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_angles(t, state_eta, controls, eps_max, id=0, title=None, option='rad'):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    if option == 'rad':
        labels = [r'$\phi$ (rad)', r'$\theta$ (rad)', r'$\psi$ (rad)']
    else:
        labels = [r'$\phi$ (deg)', r'$\theta$ (deg)', r'$\psi$ (deg)']
        state_eta = state_eta * 180 / np.pi
        controls[:, 1:2] = controls[:, 1:2] * 180 / np.pi

    for i in range(3):
        axs[i].plot(t, state_eta[:, i], label='sim')
        if i != 2:
            axs[i].plot(t, controls[:, i+1], '--', label='ref')
            axs[i].axhline(eps_max, color='r', linestyle='--', alpha=0.7)
            axs[i].axhline(-eps_max, color='r', linestyle='--', alpha=0.7)

        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Angles', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_position_difference(t, pose_data, qtm_data, id=0):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['ex (m)', 'ey (m)', 'ez (m)']
    for i in range(3):
        axs[i].plot(t, pose_data[:, i] - qtm_data[:, i], label='sim')
        axs[i].legend(loc="best")
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')
    fig.suptitle(f'Drone {id+1} Position Difference POSE - QTM', fontsize=14)


def plot_velocity_difference(t, pose_data, qtm_data, id=0):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['evx (m/s)', 'evy (m/s)', 'evz (m/s)']
    for i in range(3):
        axs[i].plot(t, pose_data[:, i] - qtm_data[:, i], label='sim')
        axs[i].legend()
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[2].set_xlabel('Time (s)')
    fig.suptitle(f'Drone {id+1} Velocity Difference POSE - QTM', fontsize=14)


def plot_controls(t, controls, id=0, Tmax=None, eps_max=None, title=None):
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    labels = ['Thrust', 'Roll (rad)', 'Pitch (rad)']
    for i in range(3):
        axs[i].plot(t, controls[:, i])
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    if Tmax is not None:
        axs[0].axhline(Tmax, color='r', linestyle='--', alpha=0.7)
        axs[0].axhline(0, color='r', linestyle='--', alpha=0.7)

    if eps_max is not None:
        for i in [1, 2]:
            axs[i].axhline(eps_max, color='r', linestyle='--', alpha=0.7)
            axs[i].axhline(-eps_max, color='r', linestyle='--', alpha=0.7)

    axs[2].set_xlabel('Time (s)')

    if title is None:
        fig.suptitle(f'Drone {id+1} Controls', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)


def plot_yaw(t, yaws, id=0, title=None, option='rad'):
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))

    if option == 'deg':
        ax.plot(t, yaws * 180/np.pi, label=r'$\psi$')
        ax.set_ylabel('Yaw (deg)')
    elif option == 'rad':
        ax.plot(t, yaws, label=r'$\psi$')
        ax.set_ylabel('Yaw (rad)')

    ax.set_xlabel('Time (s)')
    ax.grid(True)
    ax.legend()

    if title is None:
        fig.suptitle(f'Drone {id+1} Measured Yaw', fontsize=14)
    else:
        fig.suptitle(title, fontsize=14)

import numpy as np
import matplotlib.pyplot as plt
from utils_plot import *
import argparse
import os

parser = argparse.ArgumentParser(description="Drone simulation analysis")
parser.add_argument('simid', type=int, help="Simulation ID to load data")
args = parser.parse_args()
simid = args.simid


def resolve_data_files(simid):
    base_path = "src/Data_Drone/data_files/"

    # Check for 'parallel' versions first
    takeoff_file_parallel = os.path.join(base_path, f"drone_data_{simid}_takeoff_parallel.npy")
    data_file_parallel = os.path.join(base_path, f"drone_data_{simid}_parallel.npy")

    # Check if the parallel files exist
    if all(os.path.exists(f) for f in [takeoff_file_parallel, data_file_parallel]):
        print("[INFO] Using PARALLEL data files.")
        return takeoff_file_parallel, data_file_parallel

    # Fallback to sequential versions
    takeoff_file = os.path.join(base_path, f"drone_data_{simid}_takeoff.npy")
    data_file = os.path.join(base_path, f"drone_data_{simid}.npy")

    if all(os.path.exists(f) for f in [takeoff_file, data_file]):
        print("[INFO] Using SEQUENTIAL data files.")
        return takeoff_file, data_file

    raise FileNotFoundError(f"[ERROR] No suitable data files found for simid = {simid}.")


TAKEOFF_FILE, DATA_FILE = resolve_data_files(simid)

DRONE_URIS = {'DroneE1': 'radio://0/80/2M/E7E7E7E7E1', 'DroneE2': 'radio://0/80/2M/E7E7E7E7E2',
              'DroneE3': 'radio://0/80/2M/E7E7E7E7E3', 'DroneE9': 'radio://0/80/2M/E7E7E7E7E9'}


class YawUnwrapper:
    def __init__(self):
        self.previous_yaw = None

    def unwrap(self, current_yaw):
        if self.previous_yaw is None:
            self.previous_yaw = current_yaw
            return current_yaw

        delta = current_yaw - self.previous_yaw

        # Adjust yaw to prevent large backward turns
        if delta > np.pi:
            current_yaw -= 2 * np.pi
        elif delta < -np.pi:
            current_yaw += 2 * np.pi

        # Ensure yaw remains forward-aligned
        if abs(current_yaw - self.previous_yaw) > np.pi / 2:
            current_yaw += np.pi  # Flip 180 degrees to maintain forward motion
            current_yaw = (current_yaw + np.pi) % (2 * np.pi) - np.pi  # Keep within [-pi, pi]

        self.previous_yaw = current_yaw
        return current_yaw


def load_data(file_dest):
    loaded_dict = np.load(file_dest, allow_pickle=True).item()
    return loaded_dict['data'], loaded_dict['ref'], loaded_dict['vref']


if __name__ == "__main__":
    takeoff_data, takeoff_ref, takeoff_vref = load_data(TAKEOFF_FILE)
    flight_data, flight_ref, flight_vref = load_data(DATA_FILE)

    nb = len(flight_data)
    drone_names = flight_data.keys()

    print(f"Detected {nb} bodies with names {list(drone_names)}!")

    for name in drone_names:
        print(name)
        # Get the takeoff data
        to_data = takeoff_data[name]  # (x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz, T, phid, thetad, yaw_t, t, ax, ay, az)
        to_ref = takeoff_ref[DRONE_URIS[name]]
        to_vref = takeoff_vref[DRONE_URIS[name]]

        to_state = to_data[:, 0:6]
        to_angles = np.array([quaternion_to_euler(to_data[i, 6:10]) for i in range(len(to_data))])
        to_controls = to_data[:, 13:16]
        to_yaws = to_data[:, 16]
        to_t = to_data[:, 17].reshape(-1, 1)
        to_v = to_data[:, 18:21]

        # Get the normal data after takeoff, when refrence tracking
        track_data = flight_data[name]
        track_ref = flight_ref[DRONE_URIS[name]]
        track_vref = flight_vref[DRONE_URIS[name]]

        track_state = track_data[:, 0:6]
        track_angles = np.array([quaternion_to_euler(track_data[i, 6:10]) for i in range(len(track_data))])
        track_controls = track_data[:, 13:16]
        track_yaws = track_data[:, 16]
        track_t = track_data[:, 17].reshape(-1, 1)
        track_v = track_data[:, 18:21]

        # Combine to get the total simulated data
        t = np.vstack([to_t, track_t])
        state = np.vstack([to_state, track_state])
        angles = np.vstack([to_angles, track_angles])
        ref = np.vstack([to_ref, track_ref])
        vref = np.vstack([to_vref, track_vref])
        controls = np.vstack([to_controls, track_controls])
        yaws = np.vstack([to_yaws.reshape(-1, 1), track_yaws.reshape(-1, 1)])
        v = np.vstack([to_v, track_v])

        # unwrapper = YawUnwrapper()
        # yaws = np.array([unwrapper.unwrap(yaw) for yaw in yaws])

        plot_positions(t, state[:, 0:3], ref[:, 0:3], id=0, title=f"Drone {name} Positions")
        plot_velocity(t, state[:, 3:6], ref[:, 3:6], id=0, title=f"Drone {name} Velocity")
        plot_angles(t, angles, controls, eps_max=0.5, title=f"Drone {name} Angles")
        # plot_angles(t[0:len(pose_angles)], pose_angles, controls[0:len(pose_angles), :], eps_max=0.5, title=f"Drone {name} Angles")

        plot_controls(t[0:len(controls)], controls, Tmax=17, eps_max=0.2, id=0, title=f"Drone {name} Controls")

        print(t.shape)
        print(v.shape)
        plot_v(t, v, vref, id=0, title=f"Drone {name} Accelerations")

        # plot_yaw(t, yaws, id=0)

    plt.show()

import rclpy
from rclpy.node import Node
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray

from motion_capture_tracking_interfaces.msg import TargetPosition, TargetPositionArray
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from . import control_packagecf as cfcontrol
from . import Trajectory_generation as trajgen
from . import generate_traj
from .grape_3d import *
from .mpc_utils import *


QOSP = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT
)


class CrazyflieController(Node):
    def __init__(self):
        super().__init__('crazyflie_control_node')

        # Load parameters from YAML for the simulation
        config_file = os.path.join(get_package_share_directory('crazyflie_control'), 'config', 'Config_Crazyflie_Simulation.yaml')
        self.system_parameters = self.load_params(config_file)

        controller_config = os.path.join(get_package_share_directory('crazyflie_control'), 'config', 'Config_MPC.yaml')
        self.solver_config = self.load_mpc_configuration(controller_config)

        cft_config_file = os.path.join(get_package_share_directory('crazyflie_control'), 'config', 'Config_CFT.yaml')
        self.cft_config = self.load_cft_configuration(cft_config_file)

        self.pose_subscriber = self.create_subscription(NamedPoseArray, f'/poses', self.poses_callback, QOSP, callback_group=ReentrantCallbackGroup())
        self.target_publisher = self.create_publisher(TargetPositionArray, '/target_positions', 10)
        self.allocation_subscription = self.create_subscription(Int32MultiArray, '/allocation', self.allocation_callback, 10, callback_group=ReentrantCallbackGroup())

        self.latest_allocations = None
        self.allocation_history = []

        self.state = dict.fromkeys(self.drone_bodies, np.empty((0, 13)))  # (x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz)
        # Debugging Data
        self.output_data = dict.fromkeys(self.drone_bodies, np.empty((0, 21)))  # (self.state, T, roll, pitch, yaw, time, v)
        self.takeoff_data = dict.fromkeys(self.drone_bodies, np.empty((0, 21)))

        self.current_step = 0
        self.controls = {}
        self.T_land = {}
        self.comp_times = []
        self.has_landed = [0] * len(self.drone_bodies)

        self.ref, self.vref = self.load_trajectories()
        self.refto, self.vrefto = self.load_takeoff_traj()
        self.ref_land, self.vref_land = self.load_landing_point()

        # Define Controllers
        self.Kf, self.Klqi = self.set_controller()
        self.Kp = np.diag([2, 2, 2])
        self.Kd = np.diag([6, 6, 6])

        self.solver_data = setup_solver(self.solver_config)

        # Create the processes for each drone
        ray.init()
        self.workers = [MPCWorkerCFT.remote(id, drone, self.uris[id], self.solver_config) for id, drone in enumerate(self.drone_bodies)]

        # Warmup ray actors with a dummy process call to spun up the internal threads, and decrease the overhead caused by the first remote call
        warmup_futures = [worker.solve.remote({drone: np.zeros((1, 13)) for drone in self.drone_bodies},
                                              np.zeros((self.solver_config['Npred'], 6)),
                                              np.zeros((self.solver_config['Npred'], 3)),
                                              0,
                                              0,
                                              1) for worker in self.workers]
        ray.get(warmup_futures)

        self.file = open("src/Data_Drone/log_cft_controller.log", "w")

        self.start_swarm()

        # self.flight_phase = "takeoff"
        self.flight_phase = "allocation"

        self.roll_int = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.pitch_int = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.thrust_pwm_int = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.roll_p = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.pitch_p = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.thrust_p = {i: 0 for i, _ in enumerate(self.drone_bodies)}
        self.Thrust_pwm_eq = {i: int(cfcontrol.Thrust_to_PWM_modified(1.0, m=self.m[i])) for i, _ in enumerate(self.drone_bodies)}

        self.timer = self.create_timer(self.Ts, self.control_callback, callback_group=ReentrantCallbackGroup())

        self.start_time = time.time()
        self.file.write(f"[START] at {self.start_time}\n")
        FILE.write(f"[CONTROLLER START at {self.start_time}]\n")

    def start_swarm(self):
        """ Initializes the swarm interface. """
        try:
            cflib.crtp.init_drivers(enable_debug_driver=False)
            factory = CachedCfFactory(rw_cache='./cache')
            self.swarm = Swarm(self.uris, factory=factory)
            self.swarm.open_links()
            self.swarm.reset_estimators()
            self.get_logger().info("Estimators have been reset.")
            self.swarm.parallel_safe(self.wait_for_param_download)
            self.swarm.sequential(self.unlock_safety_check)
            self.get_logger().info("Swarm initialization complete.")
        except Exception as e:
            self.get_logger().error(f"Error during swarm initialization: {e}.")
            self.shutdown_swarm()

    def shutdown_swarm(self):
        """ Shuts down the swarm interface. """
        try:
            if self.swarm:
                self.swarm.parallel_safe(self.land)
                self.swarm.close_links()
        except Exception as e:
            self.get_logger().error(f"Error shutting down swarm: {e}")

    def allocation_callback(self, msg: Int32MultiArray):
        """ Receive the latest allocation from the cluster node. """
        incoming_alloc = np.array(msg.data).reshape(1, -1)
        void_allocation = np.full_like(incoming_alloc, -1)
        allocation_is_valid = np.all(incoming_alloc != void_allocation)

        if allocation_is_valid:
            if self.flight_phase == "allocation":
                self.flight_phase = "takeoff"
                # self.latest_allocation = incoming_alloc
                self.latest_allocation = np.array([0, 1, 0]).reshape(1, -1)

            # self.latest_allocation = incoming_alloc
            self.allocation_history.append(np.hstack((incoming_alloc,
                                                      np.array(time.time() - self.start_time).reshape(1, -1),
                                                      np.array(self.current_step).reshape(1, -1))))
        FILE.write(f"[{self.flight_phase.upper()}] [STEP {self.current_step}] [ALLOC_CB] Received allocation: {incoming_alloc}\n")

    def poses_callback(self, msg: NamedPoseArray):
        """ Saves the state information from the motion capture package. """
        for i, named_pose in enumerate(msg.poses):
            position = named_pose.pose.position
            orientation = named_pose.pose.orientation
            linear_velocity = named_pose.velocity.linear
            angular_velocity = named_pose.velocity.angular

            if named_pose.name in self.state:
                self.state[named_pose.name] = np.array([
                    position.x, position.y, position.z,
                    linear_velocity.x, linear_velocity.y, linear_velocity.z,
                    orientation.x, orientation.y, orientation.z, orientation.w,
                    angular_velocity.x, angular_velocity.y, angular_velocity.z
                ]).reshape(1, -1)

    def publish_target_positions(self, ref, step_idx):
        """ Publish all target positions with IDs at the current timestep. """
        msg = TargetPositionArray()
        for i in range(self.cft_config['num_targets']):
            target_msg = TargetPosition()
            target_msg.target_id = i

            position = Point()
            # position.x = ref[self.uris[i]][step_idx, 0]
            # position.y = ref[self.uris[i]][step_idx, 1]
            # position.z = ref[self.uris[i]][step_idx, 2]
            position.x = ref[i][step_idx, 0]
            position.y = ref[i][step_idx, 1]
            position.z = ref[i][step_idx, 2]

            target_msg.position = position
            msg.targets.append(target_msg)

        self.target_publisher.publish(msg)
        FILE.write(f"[{self.flight_phase.upper()}] [STEP {self.current_step}] [TARGET_PUB] Published target message: {msg}\n")
        FILE.flush()

    def load_params(self, config_file):
        """ Load parameters from config file. """
        with open(config_file) as f:
            system_parameters = yaml.load(f, Loader=yaml.FullLoader)

        self.simid = system_parameters['simid']
        self.g = 9.81
        self.qtm_ip = system_parameters['qtm_ip']
        self.Ts = system_parameters['Ts']
        self.Tsim = system_parameters['Tsim']
        self.m = system_parameters['mass']
        self.uris = system_parameters['uris']
        self.drone_bodies = system_parameters['drone_bodies']
        self.T_coeff = system_parameters['T_coeff']
        self.alpha = system_parameters['alpha']
        self.pwm0 = system_parameters['PWM0']
        self.v_land = 0.01  # m/s
        self.T_land = 1.5  # second
        self.Tto = 10  # Takeoff time in seconds

        return system_parameters

    def load_mpc_configuration(self, config_file):
        with open(config_file) as f:
            params = yaml.load(f, Loader=yaml.FullLoader)
        solver_config = {'Na': len(self.drone_bodies), 'Npred': params['Npred'], 'Ts': self.Ts,
                         'Q': np.array(params['Q']), 'R': np.array(params['R']), 'Qto': np.array(params['Qto']), 'Rto': np.array(params['Rto'])}
        return solver_config

    def load_cft_configuration(self, config_file):
        with open(config_file) as f:
            params = yaml.load(f, Loader=yaml.FullLoader)

        Na = len(self.drone_bodies)
        Nt = params['num_targets']

        self.agent_resources = np.random.uniform(10, 60, Na)  # agent resource (e.g., energy, capacity)
        self.target_importance = np.random.uniform(1000 * Na / Nt, 2000 * Na / Nt, Nt)

        cft_config = {'num_agents': len(self.drone_bodies), 'num_targets': params['num_targets'], 'comm_dist': params['comm_dist'],
                      'agent_resources': self.agent_resources, 'target_importance': self.target_importance}
        return cft_config

    def load_trajectories(self):
        """ Load trajectories for the drones. """
        full_ref = trajgen.get_ref_setpoints(psi=0, Tsim=self.Tsim, dt=self.Ts, version=4)
        # full_ref = generate_traj.get_ref(psi=0, Tsim=30, dt=self.Ts)
        ref, vref = {}, {}

        # ref[self.uris[0]] = full_ref["trajectory"]
        # vref[self.uris[0]] = full_ref["v_ref"]

        ref[0] = full_ref["trajectory"]
        vref[0] = full_ref["v_ref"]

        full_ref = trajgen.get_ref_setpoints(psi=0, Tsim=self.Tsim, dt=self.Ts, version=0)
        # full_ref = trajgen.get_ref_setpoints(psi=0, Tsim=30, dt=self.Ts, version=0)
        # ref[self.uris[1]] = full_ref["trajectory"] + np.array([0.5, -0.5, 0.2, 0, 0, 0])
        # vref[self.uris[1]] = full_ref["v_ref"]

        ref[1] = full_ref["trajectory"]  # + np.array([0.5, -0.5, 0.2, 0, 0, 0])
        vref[1] = full_ref["v_ref"]

        return ref, vref

    def load_takeoff_traj(self):
        """ Generate takeoff trajectory. """
        refTo, vrefTo = {}, {}
        # for i, uri in enumerate(self.uris):
        # full_refTo = trajgen.get_ref_setpoints_takeoff(psi=0, Tto=self.Tto, dt=self.Ts, ref=self.ref[self.uris[i]])
        # # offset = np.array([i, i, 0, 0, 0, 0]).reshape(1, -1)
        # refTo[uri] = full_refTo["trajectory"]
        # vrefTo[uri] = full_refTo["v_ref"]
        for i in range(self.cft_config['num_targets']):
            full_refTo = trajgen.get_ref_setpoints_takeoff(psi=0, Tto=self.Tto, dt=self.Ts, ref=self.ref[i])
            refTo[i] = full_refTo["trajectory"]
            vrefTo[i] = full_refTo["v_ref"]
        return refTo, vrefTo

    def load_landing_point(self):
        """ Set landing reference point. """
        ref_land, vref_land = {}, {}
        # for i, uri in enumerate(self.uris):
        # last_state_ref = self.ref[uri][-1]  # make the landing point be the last point of the reference
        # ref_land[uri] = np.array([[last_state_ref[0]], [last_state_ref[1]], [0.05], [0], [0], [0]]).reshape(1, -1)
        # vref_land[uri] = np.array([[0], [0], [0]]).reshape(1, -1)
        for i in range(self.cft_config['num_targets']):
            last_state_ref = self.ref[i][-1]  # make the landing point be the last point of the reference
            ref_land[i] = np.array([[last_state_ref[0]], [last_state_ref[1]], [0.05], [0], [0], [0]]).reshape(1, -1)
            vref_land[i] = np.array([[0], [0], [0]]).reshape(1, -1)
        return ref_land, vref_land

    def set_controller(self):
        """ Define the LQR controller. """
        Kf, Klqi = {}, {}

        for i, uri in enumerate(self.uris):
            Kf[uri] = -2.0 * np.array([[2.5, 0, 0, 1.5, 0, 0],
                                       [0, 2.5, 0, 0, 1.5, 0],
                                       [0, 0, 2.5, 0, 0, 1.5]])

        A = np.diag([6.19, 6.19, 6.19])
        B = np.diag([4.24, 4.24, 4.24])
        C = np.diag([-2.58, -2.58, -2.58])
        Ki = np.hstack((A, B, C))

        for drone in self.drone_bodies:
            Klqi[drone] = Ki * -1.0

        return Kf, Klqi

    def wait_for_param_download(self, scf: Swarm):
        while not scf.cf.param.is_updated:
            time.sleep(1.0)
        self.get_logger().info(f'Parameters downloaded for {scf.cf.link_uri}.')

    def unlock_safety_check(self, scf: Swarm):
        self.get_logger().info(f'Unlocking safety for {scf.cf.link_uri}.')
        scf.cf.commander.send_setpoint(0, 0, 0, 0)

    def apply_control(self, scf: Swarm, controller_cf):
        scf.cf.commander.send_setpoint(*controller_cf)

    def land_grad(self, scf, currentThrust, currentZ):  # currently unused
        T_land = float(currentZ[0]) / self.v_land
        for i in np.arange(0, T_land, self.Ts):
            scf.cf.commander.send_setpoint(0, 0, 0, int(currentThrust * (1 - 0.1*i/T_land)))

    def land(self, scf: Swarm):
        scf.cf.commander.send_stop_setpoint()
        # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
        scf.cf.commander.send_notify_setpoint_stop()

    def control_callback(self):
        """ Executes the control logic periodically using the ROS timer. """

        self.get_logger().info(f"[{self.flight_phase.upper()}] [STEP {self.current_step}]\n")

        if self.flight_phase == "allocation":
            self.publish_target_positions(self.refto, self.current_step)
            cf_input = {}
            for i, drone in enumerate(self.drone_bodies):
                cf_input[self.uris[i]] = [[0, 0, 0, 10001]]  # Lowest possible thrust
            self.swarm.parallel_safe(self.apply_control, args_dict=cf_input)

        if self.flight_phase == "takeoff":
            self.publish_target_positions(self.refto, self.current_step)
            tic = time.time()
            cf_control, _ = self.control_loop(self.current_step, self.refto, self.vrefto)
            toc = time.time()
            self.comp_times.append(toc-tic)

            self.swarm.parallel_safe(self.apply_control, args_dict=cf_control)
            self.current_step += 1
            if self.current_step == len(self.refto[0]):
                self.flight_phase = "flying"
                self.current_step = 0  # Reset step for normal trajectory

        elif self.flight_phase == "flying":
            self.publish_target_positions(self.ref, self.current_step)
            tic = time.time()
            cf_control, landing_args = self.control_loop(self.current_step, self.ref, self.vref)
            toc = time.time()
            self.comp_times.append(toc-tic)

            self.swarm.parallel_safe(self.apply_control, args_dict=cf_control)
            self.current_step += 1
            if self.current_step == len(self.ref[0]):
                self.flight_phase = "landing"
                self.current_step = 0
                self.landing_args = landing_args
                for item in self.landing_args.keys():
                    self.T_land[item] = float(self.landing_args[item][1]) / self.v_land

        elif self.flight_phase == "landing":
            cf_input = {}

            self.publish_target_positions(self.ref_land, 0)

            cluster_to_drones, drone_to_cluster_idx = get_cluster_mappings(self.latest_allocation.flatten().tolist(), self.drone_bodies)

            # Land gradually, decrease thrust to 0 iteratively
            for i, drone in enumerate(self.drone_bodies):
                currentZ = self.state[drone][:, 2][0]
                if currentZ <= 0.03 or self.has_landed[i] == 1:
                    cf_input[self.uris[i]] = [[0, 0, 0, 10001]]
                    self.has_landed[i] = 1
                elif self.current_step * self.Ts < self.T_land[self.uris[i]]:
                    cluster_id, idx_in_cluster = drone_to_cluster_idx[drone]
                    cluster_members = cluster_to_drones[cluster_id]
                    offsets = generate_formation_offsets(len(cluster_members))
                    full_offset = np.hstack((offsets[idx_in_cluster].reshape(1, 3), np.zeros((1, 3))))

                    target_idx = int(self.latest_allocation[0, i])
                    ref_with_offset = self.ref_land[target_idx] + full_offset

                    v = cfcontrol.compute_PD_land_control(self.state[drone][:, 0:6], ref_with_offset, self.Kp, self.Kd)
                    _, _, yaw_tmp = cfcontrol.quaternion_to_euler(self.state[drone][:, 6:10][0])
                    [roll, pitch, _, _] = cfcontrol.get_cf_input(v, yaw_tmp, T_coeff=self.T_coeff[i], alpha=self.alpha[i], mass=self.m[i])
                    cf_input[self.uris[i]] = [[roll, pitch, 0, int(self.landing_args[self.uris[i]][0] * (1 - 0.1*self.current_step/self.T_land[self.uris[i]]))]]
                    self.has_landed[i] = 0
                else:
                    self.has_landed[i] = 1

            if any(self.has_landed) == 0:
                self.swarm.parallel_safe(self.apply_control, args_dict=cf_input)
                self.current_step = self.current_step + 1

            if all(self.has_landed) == 1:
                cfcontrol.save_data(f"src/Data_Drone/data_files/drone_data_{self.simid}_parallel.npy", self.output_data, self.ref, self.vref)
                cfcontrol.save_data(f"src/Data_Drone/data_files/drone_data_{self.simid}_takeoff_parallel.npy", self.takeoff_data, self.refto, self.vrefto)
                np.save(f"src/Data_Drone/data_files/computation_times_{self.simid}_parallel.npy", {"comp_times": self.comp_times})
                np.save(f"src/Data_Drone/data_files/allocations_{self.simid}_parallel.npy", {"alloc_hist": self.allocation_history})

                self.swarm.parallel_safe(self.land)
                self.swarm.close_links()
                self.timer.cancel()

    def control_loop(self, cnt, ref_set, vref_set):
        cf_control = {}
        currentTz = {}

        cluster_to_drones, drone_to_cluster_idx = get_cluster_mappings(self.latest_allocation.flatten().tolist(), self.drone_bodies)

        futures = []
        for i, drone in enumerate(self.drone_bodies):
            cluster_id, idx_in_cluster = drone_to_cluster_idx[drone]
            cluster_members = cluster_to_drones[cluster_id]

            futures.append(self.workers[i].solve.remote(
                states=self.state,
                pos_ref=ref_set[cluster_id],
                v_ref=vref_set[cluster_id],
                i=cnt,
                drone_index_in_cluster=idx_in_cluster,
                cluster_size=len(cluster_members)
            ))

        results = ray.get(futures)

        for i, v, yaw_tmp in results:
            drone = self.drone_bodies[i]
            [Roll, Pitch, _, Thrust_pwm] = cfcontrol.get_cf_input(v, yaw_tmp, T_coeff=self.T_coeff[i], alpha=self.alpha[i], mass=self.m[i])
            self.controls[drone] = cfcontrol.get_real_input(v, yaw_tmp)

            # a small PI loop to cancel out steady state error
            cf_control[self.uris[i]] = [[
                Roll + 0.055 * self.roll_int[i] + 0.125 * self.roll_p[i],
                Pitch + 0.055 * self.pitch_int[i] + 0.125 * self.pitch_p[i],
                -0.0001 * yaw_tmp * 0.0,
                int(Thrust_pwm + self.thrust_pwm_int[i] * 0.0055 + 0.012 * self.thrust_p[i])  # thrust in PWM unit
            ]]

            self.roll_int[i] = Roll * self.Ts + self.roll_int[i]
            self.pitch_int[i] = Pitch * self.Ts + self.pitch_int[i]
            self.thrust_pwm_int[i] = (Thrust_pwm - self.Thrust_pwm_eq[i]) * self.Ts + self.thrust_pwm_int[i]

            self.roll_p[i] = Roll
            self.pitch_p[i] = Pitch
            self.thrust_p[i] = Thrust_pwm - self.Thrust_pwm_eq[i]

            currentTz[self.uris[i]] = [cf_control[self.uris[i]][0][3], self.state[drone][:, 2][0]]

            if self.flight_phase == "takeoff":
                self.takeoff_data[drone] = np.vstack((self.takeoff_data[drone],
                                                      np.hstack((np.array(self.state[drone]).reshape(1, -1),
                                                                 np.array(self.controls[drone]).reshape(1, -1),
                                                                 np.array(yaw_tmp).reshape(1, -1),
                                                                 np.array(time.time() - self.start_time).reshape(1, -1),
                                                                 np.array(v).reshape(1, -1)))))
            if self.flight_phase == "flying":
                self.output_data[drone] = np.vstack((self.output_data[drone],
                                                     np.hstack((np.array(self.state[drone]).reshape(1, -1),
                                                               np.array(self.controls[drone]).reshape(1, -1),
                                                               np.array(yaw_tmp).reshape(1, -1),
                                                               np.array(time.time() - self.start_time).reshape(1, -1),
                                                               np.array(v).reshape(1, -1)))))

        return cf_control, currentTz


def main(args=None):
    rclpy.init(args=args)
    controller = CrazyflieController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
        controller.swarm.parallel_safe(controller.land)
        controller.swarm.close_links()
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    controller.file.close()


if __name__ == '__main__':
    main()

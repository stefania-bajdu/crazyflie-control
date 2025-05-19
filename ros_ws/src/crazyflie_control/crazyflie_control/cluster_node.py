import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import yaml
import os
import ray
import time
from ament_index_python.packages import get_package_share_directory
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from motion_capture_tracking_interfaces.msg import TargetPositionArray
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from .grape_3d import *
from .control_packagecf import *

QOSP = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT
)


@ray.remote
def run_grape_allocation(config, agent_locations, target_locations, last_allocation):
    grape = GrapeClustering3D(config)
    scenario = grape.generate_scenario(agent_locations, target_locations, last_allocation)
    allocation_result = grape.grape_allocation(scenario, display_progress=False)
    return allocation_result


class ClusterFormationNode(Node):
    def __init__(self):
        super().__init__('cluster_formation_node')

        config_file = os.path.join(get_package_share_directory('crazyflie_control'), 'config', 'Config_Crazyflie_single.yaml')
        self.system_parameters = self.load_params(config_file)

        cft_config_file = os.path.join(get_package_share_directory('crazyflie_control'), 'config', 'Config_CFT.yaml')
        self.cft_config = self.load_cft_configuration(cft_config_file)

        self.pose_subscription = self.create_subscription(NamedPoseArray, f'/poses', self.poses_callback, QOSP, callback_group=ReentrantCallbackGroup())
        self.target_subscription = self.create_subscription(TargetPositionArray, '/target_positions', self.target_callback, 10, callback_group=ReentrantCallbackGroup())

        self.allocation_publisher = self.create_publisher(Int32MultiArray, '/allocation', 10)

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        ray.init(ignore_reinit_error=True)

        self.latest_positions = None
        self.current_targets = None
        self.latest_allocation = np.full(self.cft_config['num_agents'], -1, dtype=int)
        self.future = None
        self.received_positions = 0
        self.received_targets = 0

        self.file = open("src/Data_Drone/log_cluster_node.log", "w")

        self.start_time = time.time()

    def load_params(self, config_file):
        """ Load parameters from config file. """
        with open(config_file) as f:
            system_parameters = yaml.load(f, Loader=yaml.FullLoader)

        self.uris = system_parameters['uris']
        self.drone_bodies = system_parameters['drone_bodies']
        return system_parameters

    def load_cft_configuration(self, config_file):
        with open(config_file) as f:
            params = yaml.load(f, Loader=yaml.FullLoader)

        Na = len(self.drone_bodies)
        Nt = params['num_targets']

        self.agent_resources = np.random.uniform(10, 60, Na)
        # self.target_importance = np.random.uniform(1000 * Na / Nt, 2000 * Na / Nt, Nt)
        self.target_importance = [10] * Nt

        cft_config = {'num_agents': len(self.drone_bodies), 'num_targets': params['num_targets'], 'comm_dist': params['comm_dist'],
                      'agent_resources': self.agent_resources, 'target_importance': self.target_importance}
        return cft_config

    def target_callback(self, msg: TargetPositionArray):
        target_map = {target.target_id: target.position for target in msg.targets}
        targets = []
        for target_id in range(self.cft_config['num_targets']):
            if target_id in target_map:
                pos = target_map[target_id]
                targets.append([pos.x, pos.y, pos.z])
            else:
                self.received_targets = 0

        if len(targets) == self.cft_config['num_targets']:
            self.current_targets = np.array(targets)
            self.file.write(f"[CLUSTER] [TIME {time.time() - self.start_time}] Received targets {targets}\n")
            self.received_targets = 1
        else:
            # self.get_logger().info(f"Missing targets, got {len(targets)} instead of {self.cft_config['num_targets']}")
            self.file.write(f"[CLUSTER] [TIME {time.time() - self.start_time}] Missing targets, got {len(targets)} instead of {self.cft_config['num_targets']}\n")

    def poses_callback(self, msg: NamedPoseArray):
        pose_map = {named_pose.name: named_pose.pose.position for named_pose in msg.poses}

        positions = []
        for drone_name in self.drone_bodies:
            if drone_name in pose_map:
                pos = pose_map[drone_name]
                positions.append([pos.x, pos.y, pos.z])
            else:
                self.received_positions = 0
                # self.get_logger().warn(f"Drone body {drone_name} not found in poses!")

        if len(positions) == len(self.drone_bodies):
            self.latest_positions = np.array(positions)
            self.received_positions = 1
        else:
            # self.get_logger().warn(f"Missing drone poses, got {len(positions)} instead of {len(self.drone_bodies)}")
            self.file.write(f"[CLUSTER] [TIME {time.time() - self.start_time}] Missing drone poses, got {len(positions)} instead of {len(self.drone_bodies)}\n")

    def timer_callback(self):
        # If the positions arent available, wait until the first QTM pack
        if self.received_positions == 0 or self.received_targets == 0:
            self.file.write(f"[CLUSTER] [ALLOC] [TIME {time.time() - self.start_time}] Didnt find the first positions or targets, returning\n")
            allocation_msg = Int32MultiArray()
            allocation_msg.data = self.latest_allocation.tolist()
            self.allocation_publisher.publish(allocation_msg)
            return

        # Check if clustering finished
        if self.future is not None:
            ready, _ = ray.wait([self.future], timeout=0)

            self.file.write(f"[CLUSTER] [ALLOC] [TIME {time.time() - self.start_time}] Waiting for future to be finished\n")

            if ready:
                result = ray.get(self.future)
                self.file.write(f"[CLUSTER] [ALLOC] [TIME {time.time() - self.start_time}] [FINISHED] Old allocation: {self.latest_allocation}, Updated allocation {result['final_allocation']}\n")
                self.latest_allocation = np.array(result['final_allocation'])
                self.future = None

        # Always publish the current known allocation (even if it's old)
        self.file.write(f"[CLUSTER] [OLD ALLOC] [TIME {time.time() - self.start_time}] Publishing Old allocation: {self.latest_allocation}\n")
        allocation_msg = Int32MultiArray()
        allocation_msg.data = self.latest_allocation.tolist()
        self.allocation_publisher.publish(allocation_msg)

        # If no clustering algorithm is running, start the reclustering based on the current positions
        if self.future is None:
            self.file.write(f"[CLUSTER] [ALLOC] [TIME {time.time() - self.start_time}] Restart the Clustering algorithm\n")
            self.future = run_grape_allocation.remote(self.cft_config, self.latest_positions, self.current_targets, self.latest_allocation)


def main(args=None):
    rclpy.init(args=args)
    cluster_node = ClusterFormationNode()

    try:
        rclpy.spin(cluster_node)
    except KeyboardInterrupt:
        cluster_node.get_logger().info("Shutting down cluster node...")
    finally:
        cluster_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    cluster_node.file.close()


if __name__ == '__main__':
    main()

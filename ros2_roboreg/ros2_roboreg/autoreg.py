### code to call into
### ros2_roboreg services
### ros2_control joint trajectory controller action server

import argparse
import csv
import glob
import os

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from ros2_roboreg_idl.srv import CollectSample


class AutoReg(Node):
    def __init__(self, joint_states_path: str) -> None:
        super().__init__("autoreg")

        self.collect_sample_client_ = self.create_client(
            CollectSample, "collect_sample"
        )
        while not self.collect_sample_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.collect_sample_client_.srv_name} service..."
            )
        self.joint_trajectory_action_client_ = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
        )
        while not self.joint_trajectory_action_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.joint_trajectory_action_client_._action_name} action server..."
            )
        joint_state_files = glob.glob(
            os.path.join(joint_states_path, "joint_state*.npy")
        )
        self.joint_states_ = [
            np.load(joint_state_file) for joint_state_file in joint_state_files
        ]

    def run(self) -> None:
        for joint_state in self.joint_states_:
            # trajectory execution
            self.get_logger().info(f"Executing trajectory...")
            if not self.execute_trajectory_(joint_state):
                self.get_logger().error(f"Failed to execute trajectory. Exiting...")
                return
            self.get_logger().info(f"Trajectory executed successfully.")

            # data collection
            self.get_logger().info(f"Collecting data for joint state...")
            if not self.collect_sample_():
                self.get_logger().error(f"Failed to collect data. Continuing...")
                continue
            self.get_logger().info(f"Data collected successfully.")

    def collect_sample_(self) -> bool:
        self.collect_sample_client_.wait_for_service()
        req = CollectSample.Request()
        future = self.collect_sample_client_.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            return False
        if result.success:
            return True
        return False

    def execute_trajectory_(self, joint_state: np.ndarray) -> bool:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
        point = JointTrajectoryPoint()
        point.positions = joint_state.tolist()
        point.time_from_start = self.get_clock().now().to_msg()
        goal_msg.trajectory.points.append(point)
        self.joint_trajectory_action_client_.wait_for_server()
        self.joint_trajectory_action_client_.send_goal(goal_msg)


def main() -> None:
    rclpy.init(args=None)
    # parser = argparse.ArgumentParser()
    # args, unkown_args = parser.parse_known_args()
    path = "/media/martin/Samsung_T5/24_04_22_faros_integration/24_04_29_pig_specimen/calib"
    autoreg = AutoReg(joint_states_path=path)
    autoreg.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

### code to call into
### roboreg_nodes services
### ros2_control joint trajectory controller action server

import glob
import os

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from roboreg_idl.srv import CollectSample


class AutoReg(Node):
    def __init__(self) -> None:
        super().__init__("autoreg")

        self.declare_parameter("path", "")
        joint_states_path = self.get_parameter("path").get_parameter_value().string_value

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
        point.time_from_start.sec = 5
        goal_msg.trajectory.points.append(point)
        self.joint_trajectory_action_client_.wait_for_server()
        goal_future = self.joint_trajectory_action_client_.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True


def main() -> None:
    rclpy.init(args=None)
    autoreg = AutoReg()
    autoreg.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

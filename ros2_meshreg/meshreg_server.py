from rclpy.node import Node

from sensor_msgs.msg import JointState, PointCloud2, Image
from std_srvs.srv import Trigger


from message_filters import ApproximateTimeSynchronizer, Subscriber

# point cloud
# image
# joint state
# robot descritpion


class MeshregServerNode(Node):
    def __init__(self, node_name: str) -> None:
        super.__init__(node_name)

        self.image_sub = Subscriber(
            self, Image, "/image"
        )
        

        self.joint_state_sub = Subscriber(
            self, JointState, "/joint_states"
        )
        
        self.point_cloud_sub = Subscriber(
            self, PointCloud2, "/point_cloud"
        )
        
        self.approximate_time_sync = ApproximateTimeSynchronizer([
            self.image_sub,
            self.joint_state_sub,
            self.point_cloud_sub
        ], queue_size=1, slop=0.1)
        self.approximate_time_sync.registerCallback(
            self.on_sync
        )

    def on_sync(self, image: Image, joint_state: JointState, point_cloud: PointCloud2):


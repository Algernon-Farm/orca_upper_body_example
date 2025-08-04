import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from cy_manipulation_msgs.srv import HandInit


class RightExample(Node):
    def __init__(self, name: str, pos):
        super().__init__(name)

        self._dt = 0.01
        self._pos = pos

        self._sub = self.create_subscription(JointState, "/right_hand_policy", callback=self._sub_callback,
                                             qos_profile=1)
        self._pub = self.create_publisher(PoseStamped, "/right_hand_target", 1)
        self._client = self.create_client(HandInit, "/right_hand_init")

        self._timer = self.create_timer(self._dt, self._timer_callback)

    def _sub_callback(self, msg: JointState):
        print(msg.position) # print the joint position

    def _timer_callback(self):
        pose_stamped = PoseStamped() # set the pose
        pose_stamped.pose.position.x = self._pos[0]
        pose_stamped.pose.position.y = self._pos[1]
        pose_stamped.pose.position.z = self._pos[2]
        pose_stamped.pose.orientation.x = self._pos[3]
        pose_stamped.pose.orientation.y = self._pos[4]
        pose_stamped.pose.orientation.z = self._pos[5]
        pose_stamped.pose.orientation.w = self._pos[6]
        self._pub.publish(pose_stamped)

    def set_joint(self, joint):
        hand_init = HandInit.Request()
        hand_init.qpos = joint
        self._client.call_async(hand_init)


def main(args=None):
    rclpy.init(args=args)
    pos = np.array([0.36, -0.24, 0.055, 0.7056, 0.7045, -0.0603, 0.0456])  # px py pz qx qy qz qw
    node = RightExample("right_example", pos)
    node.set_joint([0.0, -0.15, 0.0, 1.55, 0.0, 0.0])   # set the initial value of the joint
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()

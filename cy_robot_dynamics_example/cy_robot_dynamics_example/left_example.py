import rclpy
from rclpy.node import Node
from cy_manipulation_msgs.srv import IdentifyPayload


class LeftExample(Node):
    def __init__(self, name: str):
        super().__init__(name)

        self._client = self.create_client(IdentifyPayload, "/identify_payload")

    def call(self):
        request = IdentifyPayload.Request()
        request.id = 0
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: IdentifyPayload.Response = future.result()
        print('mass: ',  response.mass)


def main(args=None):
    rclpy.init(args=args)
    node = LeftExample("left_example")
    node.call()

import time

import rclpy
from rclpy.node import Node
import rclpy.qos as qos

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('my_node')

        self.cli = self.create_client(GetParameters, '/crawler_bot/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()

        self.t = self.create_timer(0.1, self.callback)
        self.t = time.time()

    def send_request(self):
        print("sending")
        self.req.names = ['operating_mode']
        self.future = self.cli.call_async(self.req)

    def callback(self):

        if self.future.done():
            print(self.future.result())
            time.sleep(1)
            print(time.time())
            self.future.set_result(None)
            if time.time() > self.t + 10:
                self.send_request()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import sys
from rclpy.node import Node
from practice_robotarm_msgs.srv import AddTwoInts

class SimpleServiceClient(Node):

    def __init__(self, a, b):
        super().__init__('simple_service_client')

        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting more...')
        
        self.req_ = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b
        self.future_ = self.client_.call_async(self.req_)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info('Service Response: %d' % future.result().sum)

def main(args=None):
    rclpy.init()
    if len(sys.argv) != 3:
        print('usage: simple_service_client.py a b')
        return -1

    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
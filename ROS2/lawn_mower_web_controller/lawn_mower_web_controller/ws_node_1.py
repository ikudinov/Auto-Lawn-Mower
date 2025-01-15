import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class WsNode(Node):
    def __init__(self): 
        super().__init__('ws_node')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Node started 2!')

def main(args=None):
    try:
      rclpy.init(args=args)
      node = WsNode()
      rclpy.spin(node)
      rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
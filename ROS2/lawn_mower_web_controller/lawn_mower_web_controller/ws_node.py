import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tornado.ioloop
import tornado.httpserver
import tornado.websocket
from robot_interfaces.msg import Motors


WEB_SERVER_PORT = 4041
MOTORS_TOPIC = '/stm32/motors'

ws_clients = set()

class WsHandler(tornado.websocket.WebSocketHandler):
  def open(self, args):
    print("Socket connected")
    ws_clients.add(self)

  def on_close(self):
    ws_clients.remove(self)

  def on_message(self, message):
    self.write_message(u"You said: " + message)
  
  def check_origin(self, origin):
    return True


class WsServer():
  http_server = None

  def run(self, port):
    app = tornado.web.Application(
      [(r'/(.*)', WsHandler)],
      websocket_ping_interval=10,
      websocket_ping_timeout=30,
    )
    self.http_server = tornado.httpserver.HTTPServer(app)
    self.http_server.listen(port)

  def spin(self, callback):
    tornado.ioloop.PeriodicCallback(callback, 10).start()
    tornado.ioloop.IOLoop.current().start()


class WsNode(Node):
  def __init__(self): 
    super().__init__('ws_node')

    self.motor_subscriber = self.create_subscription(
      msg_type=Motors,
      topic=MOTORS_TOPIC,
      callback=self.motors_subscriber_callback,
      qos_profile=1)

  def spin_callback(self):
    rclpy.spin_once(node=self, timeout_sec=0)

  def motors_subscriber_callback(self, msg: Motors):
    # self.get_logger().info(f"""Motor msg left={msg.left} right={msg.right} trimmer={msg.trimmer}  """)

    json = f'{{"type":"motors","data":{{"left":{msg.left},"right":{msg.right},"trimmer":{str(msg.trimmer).lower()}}}}}'

    for client in ws_clients:
      client.write_message(json)


def main(args=None):
  try:
    rclpy.init(args=args)

    wsServer = WsServer()
    wsServer.run(WEB_SERVER_PORT)

    node = WsNode()
    wsServer.spin(node.spin_callback)

    rclpy.shutdown()
  except (KeyboardInterrupt, ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
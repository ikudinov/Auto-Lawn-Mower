import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tornado.ioloop
import tornado.httpserver
import tornado.websocket


WEB_SERVER_PORT = 4041


class WsHandler(tornado.websocket.WebSocketHandler):
  def open(self, args):
    print("WebSocket opened")

  def on_close(self):
    print("WebSocket closed")

  def on_message(self, message):
    print("WS message '%s'" % message)
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
    print('WS server started at port=%s' % port)

  def spin(self, callback):
    tornado.ioloop.PeriodicCallback(callback, 100).start()
    tornado.ioloop.IOLoop.current().start()


class WsNode(Node):
    def __init__(self): 
        super().__init__('ws_node')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Node started 3')

    def spin_callback(self):
      rclpy.spin_once(self)


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
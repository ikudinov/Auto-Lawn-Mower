from os import path
from rclpy.executors import ExternalShutdownException
import tornado.ioloop
import tornado.web
from ament_index_python.packages import get_package_share_directory


WEB_SERVER_PORT = 80


class WebServer():
  http_server = None

  def get_html_dir(self):
    share_dir = get_package_share_directory('lawn_mower_web_controller')
    return '%s/resource/html' % share_dir

  def run(self, port):
    app = tornado.web.Application([
      (
        r'/(.*)',
        tornado.web.StaticFileHandler, {
          'path': self.get_html_dir(),
          'default_filename': 'index.html'
        }
      ),
    ])
    self.http_server = tornado.httpserver.HTTPServer(app)
    self.http_server.listen(port)
    print('Web server started at port=%s' % port)

  def spin(self):
    tornado.ioloop.IOLoop.current().start()


def main(args=None):
  try:
    server = WebServer()
    server.run(WEB_SERVER_PORT)
    server.spin()
  except (KeyboardInterrupt, ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
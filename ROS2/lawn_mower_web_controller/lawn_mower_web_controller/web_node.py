from os import path
from rclpy.executors import ExternalShutdownException
import tornado.ioloop
import tornado.web


WEB_SERVER_PORT = 4040


class WebServer():
  http_server = None

  def get_html_dir(self):
    python_script_dir = path.dirname(path.realpath(__file__))
    package_dir = path.dirname(python_script_dir)

    print('%s/resource/html' % package_dir)

    return '%s/resource/html' % package_dir

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
import threading
import json
import asyncio
import rclpy
from rclpy.node import Node
from serial import Serial
from robot_interfaces.msg import Motors


class NonBlockingSerial():
  def __init__(self, port, baudrate, on_receive):
    self.serial_port = Serial(
      port=port,
      baudrate=baudrate
    )
    self.on_receive = on_receive

  def open(self):
    asyncio.run(self.read_port_loop())

  async def read_port_loop(self):
    self.loop = asyncio.get_running_loop()
    while True:
      await asyncio.sleep(0.005) # 5ms
      line = self.serial_port.readline().decode().strip()
      if line:
        self.on_receive(line)

  def close(self):
    self.loop.close()
    self.serial_port.close()


class SerialPublisherNode(Node):
  def __init__(self):
    super().__init__('serial_node')

    self.declare_parameter('serial_port_name', '')
    self.declare_parameter('serial_port_baudrate', 9600)
    self.declare_parameter('topic_name_motors', '')
    serial_port_name = self.get_parameter('serial_port_name').value
    serial_port_baudrate = self.get_parameter('serial_port_baudrate').value
    topic_name_motors = self.get_parameter('topic_name_motors').value

    self.motors_publisher = self.create_publisher(
      msg_type=Motors,
      topic=topic_name_motors,
      qos_profile=1)

    self.serial_port = NonBlockingSerial(serial_port_name, serial_port_baudrate, self.on_serial_data)
    self.serial_port.open()
    self.get_logger().info('Serial Node started')

  def on_serial_data(self, msg):
    # self.get_logger().info(f"""Serial data: {msg}""")

    obj = json.loads(msg)
    msg_type = obj["type"]
    msg_data = obj["data"]

    if (msg_type == "motors"):
      self.publish_motor_data(msg_data)

  def publish_motor_data(self, data):
    motors = Motors()
    motors.left = data["l"];
    motors.right = data["r"];
    motors.trimmer = data["t"] == 1;
    self.motors_publisher.publish(motors)


def main(args=None):
  # try:
    rclpy.init(args=args)

    serial_publisher_node = SerialPublisherNode()

    rclpy.spin(serial_publisher_node)
  # except KeyboardInterrupt:
    # pass
  # except Exception as e:
    # print(e)


if __name__ == '__main__':
  main()

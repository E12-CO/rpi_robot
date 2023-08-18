#sub joy then convert to cmd_velimport rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Rpi_Joy(Node):

    def __init__(self):
        super().__init__('rpi_joy')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            qos_profile_sensor_data) #QoS value must be same as Publisher
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSReliabilityPolicy.RELIABLE
		)

    def listener_callback(self, msg):
        header = msg.header
        axes = msg.axes
        buttons = msg.buttons
        twstmsg = Twist()

        # logitech extreme 3d pro
        x1, x2, x3 = axes[0], axes[1], axes[2]
        capture = buttons[2]
        twstmsg.linear.x, twstmsg.linear.y, twstmsg.linear.z, twstmsg.angular.x, twstmsg.angular.y, twstmsg.angular.z = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        if x1 < -0.5: #rightward
            twstmsg.linear.x = 5.0
        elif x1 > 0.5: #leftward
            twstmsg.linear.x = -5.0
        elif x2 < -0.5: #backward
            twstmsg.linear.y = -5.0
        elif x2 > 0.5: #frontward
            twstmsg.linear.y = 5.0
        if x3 < -0.5: #rotateright
            twstmsg.angular.z = 5.0
        elif x3 > 0.5: #rotateleft
            twstmsg.angular.z = -5.0         

        self.publisher_.publish(twstmsg)

def main(args=None):
    rclpy.init(args=args)

    rpi_joy = Rpi_Joy()

    rclpy.spin(rpi_joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

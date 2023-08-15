#sub cmd_vel then drive motor
# Modified to work with OrangePi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
import OPi.GPIO as GPIO 

from geometry_msgs.msg import Twist

# Left Front
EnableLF = 23
In1LF = 22    
In2LF = 19

# Left Back
EnableLB = 2
In1LB = 4    
In2LB = 18

# Right Front
EnableRF = 26
In1RF = 33    
In2RF = 25

# Right Back
EnableRB = 14
In1RB = 27    
In2RB = 13

class Rpi_Motor(Node):

    def __init__(self):
        super().__init__('rpi_motor')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            QoSReliabilityPolicy.RELIABLE) #QoS value must be same as Publisher
        self.subscription  # prevent unused variable warning
        self.state = 'idle'
        self.dumstate = 'idle'

    def pinSetup(self):

        GPIO.setmode(GPIO.SUNXI)
        # Left Front
        GPIO.setup(In1LF,GPIO.OUT)
        GPIO.setup(In2LF,GPIO.OUT)
        GPIO.setup(EnableLF,GPIO.OUT)

        # Left Back
        GPIO.setup(In1LB,GPIO.OUT)
        GPIO.setup(In2LB,GPIO.OUT)
        GPIO.setup(EnableLB,GPIO.OUT)

        # Right Front
        GPIO.setup(In1RF,GPIO.OUT)
        GPIO.setup(In2RF,GPIO.OUT)
        GPIO.setup(EnableRF,GPIO.OUT)

        # Right Back
        GPIO.setup(In1RB,GPIO.OUT)
        GPIO.setup(In2RB,GPIO.OUT)
        GPIO.setup(EnableRB,GPIO.OUT)

        GPIO.output(EnableLF, GPIO.HIGH)
	GPIO.output(EnableLB, GPIO.HIGH)
	GPIO.output(EnableRF, GPIO.HIGH)
	GPIO.output(EnableRB, GPIO.HIGH)
               

    def listener_callback(self, msg):
        l_x,l_y,a_z = msg.linear.x,msg.linear.y,msg.angular.z
        if (l_x < - 0.5):
            #forward
            self.dumstate = 'forward'
        elif (l_x > 0.5):
            #backward
            self.dumstate = 'backward'
        elif (l_y < -0.5):
            #rightward
            self.dumstate = 'rightward'
        elif (l_y > 0.5):
            #leftward
            self.dumstate = 'leftward'
        elif (a_z > 0.5):
            #rotateleft
            self.dumstate = 'rotateleft'
        elif (a_z < -0.5):
            #rotateright
            self.dumstate = 'rotateright'
        else:
            self.dumstate = 'idle'

        if(self.dumstate != self.state):
            self.state = self.dumstate
            if (self.state == 'forward'):
                GPIO.output(In1LF,GPIO.LOW)
                GPIO.output(In2LF,GPIO.HIGH)
                GPIO.output(In1LB,GPIO.LOW)
                GPIO.output(In2LB,GPIO.HIGH)
                GPIO.output(In1RF,GPIO.HIGH)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.HIGH)
                GPIO.output(In2RB,GPIO.LOW)
            elif (self.state == 'backward'):
                GPIO.output(In1LF,GPIO.HIGH)
                GPIO.output(In2LF,GPIO.LOW)
                GPIO.output(In1LB,GPIO.HIGH)
                GPIO.output(In2LB,GPIO.LOW)
                GPIO.output(In1RF,GPIO.LOW)
                GPIO.output(In2RF,GPIO.HIGH)
                GPIO.output(In1RB,GPIO.LOW)
                GPIO.output(In2RB,GPIO.HIGH)
            elif (self.state == 'rightward'):
                GPIO.output(In1LF,GPIO.LOW)
                GPIO.output(In2LF,GPIO.HIGH)
                GPIO.output(In1LB,GPIO.HIGH)
                GPIO.output(In2LB,GPIO.LOW)
                GPIO.output(In1RF,GPIO.LOW)
                GPIO.output(In2RF,GPIO.HIGH)
                GPIO.output(In1RB,GPIO.HIGH)
                GPIO.output(In2RB,GPIO.LOW)
            elif (self.state == 'leftward'):
                GPIO.output(In1LF,GPIO.HIGH)
                GPIO.output(In2LF,GPIO.LOW)
                GPIO.output(In1LB,GPIO.LOW)
                GPIO.output(In2LB,GPIO.HIGH)
                GPIO.output(In1RF,GPIO.HIGH)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.LOW)
                GPIO.output(In2RB,GPIO.HIGH)
            elif (self.state == 'rotateleft'):
                GPIO.output(In1LF,GPIO.LOW)
                GPIO.output(In2LF,GPIO.LOW)
                GPIO.output(In1LB,GPIO.LOW)
                GPIO.output(In2LB,GPIO.LOW)
                GPIO.output(In1RF,GPIO.HIGH)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.HIGH)
                GPIO.output(In2RB,GPIO.LOW)
            elif (self.state == 'rotateright'):
                GPIO.output(In1LF,GPIO.HIGH)
                GPIO.output(In2LF,GPIO.LOW)
                GPIO.output(In1LB,GPIO.HIGH)
                GPIO.output(In2LB,GPIO.LOW)
                GPIO.output(In1RF,GPIO.LOW)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.LOW)
                GPIO.output(In2RB,GPIO.LOW)
            

        

def main(args=None):
    rclpy.init(args=args)

    rpi_motor = Rpi_Motor()
    rpi_motor.pinSetup()

    rclpy.spin(rpi_motor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

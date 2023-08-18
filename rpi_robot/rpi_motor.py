#sub cmd_vel then drive motor
# Modified to work with OrangePi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
import gpio as GPIO
from periphery import PWM
from geometry_msgs.msg import Twist

# Left Front
EnableLF = PWM(0, 1)
EnableLF.frequency = 10e3
EnableLF.duty_cycle = 0.0
In1LF =  73 #'PC9'    
In2LF =  70 #'PC6'

# Left Back
EnableLB = PWM(0, 2)
EnableLB.frequency = 10e3
EnableLB.duty_cycle = 0.0
In1LB =  69 # 'PC5'
In2LB =  72 # 'PC8'

# Right Front
EnableRF = PWM(0, 3)
EnableRF.frequency = 10e3
EnableRF.duty_cycle = 0.0
In1RF = 74 #'PC10'
In2RF = 78 #'PC14'

# Right Back
EnableRB = PWM(0, 4)
EnableRB.frequency = 10e3
EnableRB.duty_cycle = 0.0
In1RB = 71 # 'PC7'    
In2RB = 75 # 'PC11'


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
        # Left Front
        GPIO.setup(In1LF,GPIO.OUT)
        GPIO.setup(In2LF,GPIO.OUT)
#        GPIO.setup(EnableLF,GPIO.OUT)

        # Left Back
        GPIO.setup(In1LB,GPIO.OUT)
        GPIO.setup(In2LB,GPIO.OUT)
#        GPIO.setup(EnableLB,GPIO.OUT)

        # Right Front
        GPIO.setup(In1RF,GPIO.OUT)
        GPIO.setup(In2RF,GPIO.OUT)
#        GPIO.setup(EnableRF,GPIO.OUT)

        # Right Back
        GPIO.setup(In1RB,GPIO.OUT)
        GPIO.setup(In2RB,GPIO.OUT)

        GPIO.output(In1LF,GPIO.LOW)
        GPIO.output(In2LF,GPIO.LOW)
        GPIO.output(In1LB,GPIO.LOW)
        GPIO.output(In2LB,GPIO.LOW)
        GPIO.output(In1RF,GPIO.LOW)
        GPIO.output(In2RF,GPIO.LOW)
        GPIO.output(In1RB,GPIO.LOW)
        GPIO.output(In2RB,GPIO.LOW)

    def mecanum_cal(self, x_vel, y_vel, az_vel):
        #y_vel = -y_vel # invert the vector

        Denom = max(abs(self.x_vel), abs(self.y_vel), abs(self.az_vel)) # calculate the demoniator 

        LeftFront  = (self.x_vel + self.y_vel + self.az_vel) / Denom
        LeftBack   = (self.x_vel - self.y_vel + self.az_vel) / Denom
        RighFront  = (self.x_vel - self.y_vel - self.az_vel) / Denom
        RightBack  = (self.x_vel + self.y_vel - self.az_vel) / Denom

        print("LF: "+str(LeftFront))
        print("LB: "+str(LeftBack))
        print("RF: "+str(RightFront))
        print("RB: "+str(RightBack))

        EnableLF.duty_cycle = abs(LeftFront)
        EnableLB.duty_cycle = abs(LeftBack) 
        EnableRF.duty_cycle = abs(RightFront)
        EnableRB.duty_cycle = abs(RightBack)

        if(LeftFront > 0):
            GPIO.output(In1LF, GPIO.HIGH)
            GPIO.output(In2LF, GPIO.LOW)
        elif(LeftFront < 0): 
            GPIO.output(In1LF, GPIO.LOW)
            GPIO.output(In2LF, GPIO.HIGH)
        else:
            GPIO.output(In1LF, GPIO.LOW)
            GPIO.output(In2LF, GPIO.LOW)

        if(LeftBack > 0):
            GPIO.output(In1LB, GPIO.HIGH)
            GPIO.output(In2LB, GPIO.LOW)
        elif(LeftFront < 0):
            GPIO.output(In1LB, GPIO.LOW)
            GPIO.output(In2LB, GPIO.HIGH)
        else:
            GPIO.output(In1LB, GPIO.LOW)
            GPIO.output(In2LB, GPIO.LOW)

        if(RightFront > 0):
            GPIO.output(In1RF, GPIO.LOW)
            GPIO.output(In2RF, GPIO.HIGH)
        elif(RightFront < 0):
            GPIO.output(In1RF, GPIO.HIGH)
            GPIO.output(In2RF, GPIO.LOW)
        else:
            GPIO.output(In1RF, GPIO.LOW)
            GPIO.output(In2RF, GPIO.LOW)

        if(RightBack > 0):
            GPIO.output(In1RB, GPIO.LOW)
            GPIO.output(In2RB, GPIO.HIGH)
        elif(RightBack < 0):
            GPIO.output(In1RB, GPIO.HIGH)
            GPIO.output(In2RB, GPIO.LOW)
        else:
            GPIO.output(In1RB, GPIO.LOW)
            GPIO.output(In2RB, GPIO.LOW)


    def listener_callback(self, msg):
        l_x,l_y,a_z = msg.linear.x,msg.linear.y,msg.angular.z
        self.mecanum_cal(l_x, l_y, a_z)
        

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

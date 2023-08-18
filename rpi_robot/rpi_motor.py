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
EnableLF.duty_cycle = 0.5
In1LF =  73 #'PC9'    
In2LF =  70 #'PC6'

# Left Back
EnableLB = PWM(0, 2)
EnableLB.frequency = 10e3
EnableLB.duty_cycle = 0.5
In1LB =  69 # 'PC5'
In2LB =  72 # 'PC8'

# Right Front
EnableRF = PWM(0, 3)
EnableRF.frequency = 10e3
EnableRF.duty_cycle = 0.5
In1RF = 74 #'PC10'
In2RF = 78 #'PC14'

# Right Back
EnableRB = PWM(0, 4)
EnableRB.frequency = 10e3
EnableRB.duty_cycle = 0.5
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
        M1 = x_vel + y_vel - az_vel
        M2 = -x_vel + y_vel - az_vel
        M3 = -x_vel - y_vel - az_vel
        M4 = x_vel - y_vel - az_vel

        maxComponent = abs(M1)
        if(maxComponent < abs(M2)):
            maxComponent = abs(M2)
        if(maxComponent < abs(M3)):
            maxComponent = abs(M3)
        if(maxComponent < abs(M4)):
            maxComponent = abs(M4)
        if(maxComponent > 1.0):
            maxComponent = 1.0

        m_pwm1 = M1 / maxComponent
        m_pwm2 = M2 / maxComponent
        m_pwm3 = M3 / maxComponent
        m_pwm4 = M4 / maxComponent

        print("m_pwm1: "+str(m_pwm1))
        print("m_pwm2: "+str(m_pwm2))
        print("m_pwm3: "+str(m_pwm3))
        print("m_pwm4: "+str(m_pwm4))




    def listener_callback(self, msg):
        l_x,l_y,a_z = msg.linear.x,msg.linear.y,msg.angular.z
#        self.mecanum_cal(l_x, l_y, a_z)

        if (l_x < -0.5):
            #rightward
            self.dumstate = 'rightward'
        elif(l_x > 0.5):
            #leftward
            self.dumstate = 'leftward'
        elif (l_y < -0.5):
            #forward
            self.dumstate = 'forward'
        elif (l_y > 0.5):
            #backward
            self.dumstate = 'backward'
        elif (a_z < -0.5):
            #rotateleft
            self.dumstate = 'rotateleft'
        elif (a_z > 0.5):
            #rotateright
            self.dumstate = 'rotateright'
        else:
            self.dumstate = 'idle'

        if(self.dumstate != self.state):
            self.state = self.dumstate
            if (self.state == 'backward'):
                GPIO.output(In1LF,GPIO.LOW)
                GPIO.output(In2LF,GPIO.HIGH)
                GPIO.output(In1LB,GPIO.LOW)
                GPIO.output(In2LB,GPIO.HIGH)
                GPIO.output(In1RF,GPIO.HIGH)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.HIGH)
                GPIO.output(In2RB,GPIO.LOW)
            elif (self.state == 'forward'):
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
                GPIO.output(In1LF,GPIO.HIGH)
                GPIO.output(In2LF,GPIO.LOW)
                GPIO.output(In1LB,GPIO.HIGH)
                GPIO.output(In2LB,GPIO.LOW)
                GPIO.output(In1RF,GPIO.HIGH)
                GPIO.output(In2RF,GPIO.LOW)
                GPIO.output(In1RB,GPIO.HIGH)
                GPIO.output(In2RB,GPIO.LOW)
            elif (self.state == 'rotateright'):
                GPIO.output(In1LF,GPIO.LOW)
                GPIO.output(In2LF,GPIO.HIGH)
                GPIO.output(In1LB,GPIO.LOW)
                GPIO.output(In2LB,GPIO.HIGH)
                GPIO.output(In1RF,GPIO.LOW)
                GPIO.output(In2RF,GPIO.HIGH)
                GPIO.output(In1RB,GPIO.LOW)
                GPIO.output(In2RB,GPIO.HIGH)
            elif (self.state == 'idle'):
            	GPIO.output(In1LF,GPIO.LOW)
            	GPIO.output(In2LF,GPIO.LOW)
            	GPIO.output(In1LB,GPIO.LOW)
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

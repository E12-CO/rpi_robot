import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Controller_Cam(Node):
    def __init__(self):
        super().__init__("controller_cam")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        self.cap.set(3,192)
        self.cap.set(4,144)
        self.pub = self.create_publisher(Image, "/video_stream", qos_profile_sensor_data)

    def run(self):
        while True:
            try:
                r, frame = self.cap.read()
                tframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if not r:
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(tframe, "mono8"))


            except CvBridgeError as e:
                print(e)
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    controller_cam = Controller_Cam()
    controller_cam.run()

    controller_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
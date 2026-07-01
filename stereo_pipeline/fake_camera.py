import rclpy
from rclpy.node import Node
#Always this format did it correctly first time
from sensor_msgs.msg import Image
#Following found on official documentation
from cv_bridge import CvBridge
import cv2

#Naming convection of nodes is lowercase with _
class fake_camera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        # create publishers, subscribers, timers here
        #topics chosen are /left/image and /right/image
        self.left_publisher_ = self.create_publisher(Image, '/left/image', 10)
        self.right_publisher_ = self.create_publisher(Image, '/right/image', 10)
        #CvBridge instance
        self.bridge = CvBridge()

        self.capture = cv2.VideoCapture('/home/einarj17/stereo_ws/data/P1/video.mp4')
        self.timer_ = self.create_timer(1/30.0,self.publish_frame)
    
    def publish_frame(self):
        ok, frame = self.capture.read()
        if ok == False:
            self.capture.release()
            return
        else:
            #Split video top and bottom half
            h = frame.shape[0]
            left = frame[:h//2, :, :]
            right = frame[h//2:, :, :]
            #Convert to ROS format - "bgr8" is the colour format 8-bit BGR
            left_image  = self.bridge.cv2_to_imgmsg(left,"bgr8")
            right_image  = self.bridge.cv2_to_imgmsg(right,"bgr8")
            #Now publish to the topic
            self.left_publisher_.publish(left_image)
            self.right_publisher_.publish(right_image)


def main():
    rclpy.init()
    node = fake_camera()
    rclpy.spin(node)
    rclpy.shutdown()
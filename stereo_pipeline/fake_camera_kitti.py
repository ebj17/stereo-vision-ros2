import rclpy
from rclpy.node import Node
#Always this format did it correctly first time
from sensor_msgs.msg import Image
#Following found on official documentation
from cv_bridge import CvBridge
import cv2
import glob

#Naming convection of nodes is lowercase with _
class fake_camera_kitti(Node):
    def __init__(self):
        super().__init__('fake_camera_kitti')
        # create publishers, subscribers, timers here
        #topics chosen are /left/image and /right/image
        self.left_publisher_ = self.create_publisher(Image, '/left/image', 10)
        self.right_publisher_ = self.create_publisher(Image, '/right/image', 10)
        #CvBridge instance
        self.bridge = CvBridge()

        #KITTI gives numbered image pairs instead of one video file
        #only the _10 frame has ground truth disparity, _11 is just the next frame for optical flow so skip those
        self.left_files = sorted(glob.glob('/home/einarj17/stereo_ws/data/kitti/training/colored_0/*_10.png'))
        self.right_files = sorted(glob.glob('/home/einarj17/stereo_ws/data/kitti/training/colored_1/*_10.png'))
        self.index = 0

        self.timer_ = self.create_timer(1/30.0,self.publish_frame)

    def publish_frame(self):
        if self.index >= len(self.left_files):
            #ran out of pairs, stop publishing new frames
            return
        else:
            left = cv2.imread(self.left_files[self.index])
            right = cv2.imread(self.right_files[self.index])
            self.index += 1

            #Convert to ROS format - "bgr8" is the colour format 8-bit BGR
            left_image  = self.bridge.cv2_to_imgmsg(left,"bgr8")
            right_image  = self.bridge.cv2_to_imgmsg(right,"bgr8")
            #Now publish to the topic
            self.left_publisher_.publish(left_image)
            self.right_publisher_.publish(right_image)


def main():
    rclpy.init()
    node = fake_camera_kitti()
    rclpy.spin(node)
    rclpy.shutdown()

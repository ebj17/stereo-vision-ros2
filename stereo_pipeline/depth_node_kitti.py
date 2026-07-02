
import rclpy
from rclpy.node import Node
#Always this format did it correctly first time
from sensor_msgs.msg import Image   #since we still deal with these
#Following found on official documentation
from cv_bridge import CvBridge
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer

#This node subscribes to topics: /left/image and /right/image and computes disparity
#KITTI images come pre-rectified so no calibration file or remap is needed here

class depth_node_kitti(Node):
    def __init__(self):
        super().__init__('depth_node_kitti')
        #since we are using message filters we dont use self.create_subscription
        self.bridge = CvBridge()
        self.left_subscriber_ = Subscriber(self, Image, '/left/image')
        self.right_subscriber_ = Subscriber(self, Image, '/right/image')
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.left_subscriber_, self.right_subscriber_],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)

        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            disp12MaxDiff=1,
            mode=cv2.StereoSGBM_MODE_SGBM_3WAY
        )

        # uniquenessRatio - rejects match if 2nd best match is too similar to best. Higher = stricter, fewer but more reliable matches
        # disp12MaxDiff - matches left-to-right AND right-to-left, rejects pixels where both disagree. Catches occlusions and bad matches
        # MODE_SGBM_3WAY - optimizes in more directions than default, smoother result, slightly slower
        # speckleWindowSize - minimum size a connected region must be to be kept, small isolated blobs below this get zeroed
        # speckleRange - max disparity variation allowed within a connected region to count as one blob


    def SyncCallback(self, left_msg, right_msg):
        #Here left_msg takes the value recieved by the first sub defined in ApprimxiateTimeSyncrhonizer
        self.get_logger().info('Got synchronized pair')
        img_l = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        img_r = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')

        #KITTI images are already rectified so no cv2.remap here, straight to grayscale
        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(gray_l, gray_r)
        stereo_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        stereo_color = cv2.applyColorMap(stereo_norm, cv2.COLORMAP_JET)

        cv2.imshow("left image", img_l)
        cv2.imshow("depth map",stereo_color)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = depth_node_kitti()
    rclpy.spin(node)
    rclpy.shutdown()

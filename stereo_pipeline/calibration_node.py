import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Skeleton node for running stereo calibration from live checkerboard images.
# Subscribes to /left/image and /right/image, looks for a checkerboard in each
# synchronized pair, and collects the corner detections needed to run
# cv2.stereoCalibrate once enough pairs have been gathered.


class calibration_node(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.bridge = CvBridge()

        self.left_subscriber_ = Subscriber(self, Image, '/left/image')
        self.right_subscriber_ = Subscriber(self, Image, '/right/image')
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.left_subscriber_, self.right_subscriber_],
                                                      queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)

        # checkerboard size in inner corners (columns, rows)
        self.board_size = (9, 6)
        self.square_size = 1.0  # arbitrary units, set to real square size for metric calibration

        # 3D points are the same for every view: a flat grid in the board's own frame
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        self.objp = objp * self.square_size

        self.objpoints = []
        self.imgpoints_l = []
        self.imgpoints_r = []
        self.min_pairs = 15

    def sync_callback(self, left_msg, right_msg):
        img_l = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        img_r = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

        found_l, corners_l = cv2.findChessboardCorners(gray_l, self.board_size)
        found_r, corners_r = cv2.findChessboardCorners(gray_r, self.board_size)

        if not (found_l and found_r):
            self.get_logger().info('Checkerboard not found in both views, skipping pair')
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria)
        corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria)

        self.objpoints.append(self.objp)
        self.imgpoints_l.append(corners_l)
        self.imgpoints_r.append(corners_r)
        self.get_logger().info(f'Collected pair {len(self.objpoints)}/{self.min_pairs}')

        if len(self.objpoints) >= self.min_pairs:
            self.run_calibration(gray_l.shape[::-1])

    def run_calibration(self, image_size):
        self.get_logger().info('Running stereoCalibrate...')
        ret, cam_l, dist_l, cam_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_l, self.imgpoints_r,
            None, None, None, None, image_size,
            flags=cv2.CALIB_FIX_INTRINSIC
        )
        self.get_logger().info(f'stereoCalibrate reprojection error: {ret}')
        # TODO: write cam_l/dist_l/cam_r/dist_r/R/T out in ROS camera_info yaml format


def main():
    rclpy.init()
    node = calibration_node()
    rclpy.spin(node)
    rclpy.shutdown()


import rclpy
from rclpy.node import Node
#Always this format did it correctly first time
from sensor_msgs.msg import Image   #since we still deal with these
#Following found on official documentation
from cv_bridge import CvBridge
import cv2

import numpy as np
import configparser
from message_filters import Subscriber, ApproximateTimeSynchronizer

#This node subscribes to topics: /left/image and /right/image, loads the calbration results fom the .ini file in the dataset,
#rectifies both images, computes disparity, and publishes the results

class depth_node(Node):
    def __init__(self):
        super().__init__('depth_node')
        #since we are using message filters we dont use self.create_subscription
        #self.left_subscriber_ = self.create_subscription(Image, '/left/image', self.left_callback, 10)
        #self.right_subscriber_ = self.create_subscription(Image, '/right/image', self.right_callback, 10)
        self.bridge = CvBridge()
        self.left_subscriber_ = Subscriber(self, Image, '/left/image')
        self.right_subscriber_ = Subscriber(self, Image, '/right/image')
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.left_subscriber_, self.right_subscriber_],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)
        self.load_calibration('/home/einarj17/stereo_ws/data/P1/StereoCalibration.ini')
        self.setup_rectification()

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

        rectified_l = cv2.remap(img_l,self.map_l1,self.map_l2,cv2.INTER_LANCZOS4)
        rectified_r = cv2.remap(img_r,self.map_r1,self.map_r2,cv2.INTER_LANCZOS4)

        gray_rectified_l = cv2.cvtColor(rectified_l, cv2.COLOR_BGR2GRAY)
        gray_rectified_r = cv2.cvtColor(rectified_r, cv2.COLOR_BGR2GRAY)

        #stereo = cv2.StereoSGBM_create(minDisparity=0,numDisparities=128, blockSize=11)
        
        disparity = self.stereo.compute(gray_rectified_l, gray_rectified_r)
        stereo_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        

        cv2.imshow("depth map",stereo_norm)
        cv2.waitKey(1)


    def load_calibration(self, path):
        path = '/home/einarj17/stereo_ws/data/P1/StereoCalibration.ini'
        config_obj = configparser.ConfigParser()
        config_obj.read(path)

        #left side
        #build camera matrix 3x3 NumPy array using fc_x, fc_y, cc_x, cc_y from stereooLeft
        fx_l = float(config_obj['StereoLeft']['fc_x'])
        fy_l = float(config_obj['StereoLeft']['fc_y'])
        cx_l = float(config_obj['StereoLeft']['cc_x'])
        cy_l = float(config_obj['StereoLeft']['cc_y'])

        #self. added infront of the variable to make persistant
        #SyncCallback needs these every time a new frame arrives so tthis prevents them from diksappearing
        self.cam_matrix_left = np.array([[fx_l, 0, cx_l], [0,fy_l, cy_l], [0, 0, 1]])
        print(self.cam_matrix_left)

        #len distortion coefficients
        kc_0l = float(config_obj['StereoLeft']['kc_0'])
        kc_1l = float(config_obj['StereoLeft']['kc_1'])
        kc_2l = float(config_obj['StereoLeft']['kc_2'])
        kc_3l = float(config_obj['StereoLeft']['kc_3'])
        kc_4l = float(config_obj['StereoLeft']['kc_4'])
        self.distCoeffs_l = np.array([kc_0l, kc_1l, kc_2l, kc_3l, kc_4l])

        #right side
        #build camera matrix 3x3 NumPy array using fc_x, fc_y, cc_x, cc_y from stereooLeft
        fx_r = float(config_obj['StereoRight']['fc_x'])
        fy_r = float(config_obj['StereoRight']['fc_y'])
        cx_r = float(config_obj['StereoRight']['cc_x'])
        cy_r = float(config_obj['StereoRight']['cc_y'])

        self.cam_matrix_Right = np.array([[fx_r, 0, cx_r], [0,fy_r, cy_r], [0, 0, 1]])
        print(self.cam_matrix_Right)

        #len distortion coefficients
        kc_0r = float(config_obj['StereoRight']['kc_0'])
        kc_1r = float(config_obj['StereoRight']['kc_1'])
        kc_2r = float(config_obj['StereoRight']['kc_2'])
        kc_3r = float(config_obj['StereoRight']['kc_3'])
        kc_4r = float(config_obj['StereoRight']['kc_4'])
        self.distCoeffs_r = np.array([kc_0r, kc_1r, kc_2r, kc_3r, kc_4r])

        #From the calibration file, StereoLeft has identify R and zero T meaing that is the reference camera
        #Therefore only those for StereoRight are important as that gives us relation

        R_0 = float(config_obj['StereoRight']['R_0'])
        R_1 = float(config_obj['StereoRight']['R_1'])
        R_2 = float(config_obj['StereoRight']['R_2'])
        R_3 = float(config_obj['StereoRight']['R_3'])
        R_4 = float(config_obj['StereoRight']['R_4'])
        R_5 = float(config_obj['StereoRight']['R_5'])
        R_6 = float(config_obj['StereoRight']['R_6'])
        R_7 = float(config_obj['StereoRight']['R_7'])
        R_8 = float(config_obj['StereoRight']['R_8'])

        T_0 = float(config_obj['StereoRight']['T_0'])
        T_1 = float(config_obj['StereoRight']['T_1'])
        T_2 = float(config_obj['StereoRight']['T_2'])

        self.R = np.array([[R_0, R_1, R_2], [R_3, R_4, R_5], [R_6, R_7, R_8]])
        self.T = np.array([T_0, T_1, T_2])
    
    def setup_rectification(self):
        #image size as defined in dataset 1280,1024
        #R1, R2 - how t rotate each cameras images to have same points on horizontal 
        #P1, P2 - projection matrices in the rectified coordinae system
        #Q - disaprity to depth mapping matrix, Disparity map multiplies by Q to get real 3d coord (disparity -> actual depth in m)
        #roi_l, roi_r - after rectification some pixels around the edges become invalid (black) these tell u whatareas are valid

        image_size = (1280,1024)
        self.R1, self.R2, self.P1, self.P2, self.Q, self.roi_l, self.roi_r = cv2.stereoRectify(self.cam_matrix_left, self.distCoeffs_l, self.cam_matrix_Right, self.distCoeffs_r, image_size, self.R, self.T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

        self.map_l1, self.map_l2 = cv2.initUndistortRectifyMap(self.cam_matrix_left, self.distCoeffs_l, self.R1,self.P1,image_size, cv2.CV_16SC2)
        self.map_r1, self.map_r2 = cv2.initUndistortRectifyMap(self.cam_matrix_Right, self.distCoeffs_r, self.R2,self.P2,image_size, cv2.CV_16SC2)

def main():
    rclpy.init()
    node = depth_node()
    rclpy.spin(node)
    rclpy.shutdown()                                                   
    
    
    
    #In the case of not using message filters we would use these callbacks.
    #However, they fire independently and dont make same frames simultaneously
    #ApproximateTimeSynchronizer replaces these with one callback 
    # def left_callback(self, msg):
    #     img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #     # do something with the left image alone

    # def right_callback(self, msg):
    #     img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #     # do something with the right image alone
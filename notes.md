ROS2 Stereo Imaging Application

cv_bridge in ROS2 converts between OpenCV images (numpy arrays) into ROS2 images (sensor_msgs/Image)
createa a package:
ros2 pkg create --build-type ament_python --node-name my_node my_package

ros2 pkg create stereo_pipeline --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge image_transport message_filters
--dependencies automatically adds the packages needed for the new package

colcon build:
cd ~/stereo_ws
colcon build
source install/setup.bash

Subscriber — self.create_subscription(MessageType, 'topic', self.callback, 10). The callback fires every time a message arrives.
message_filters — synchronizes two subscribers so the callback only fires when you have a matched left+right pair. You'll need this for stereo.
cv_bridge — CvBridge().imgmsg_to_cv2(msg) converts a ROS Image message to a NumPy array.

Python Node Skeleton:
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # create publishers, subscribers, timers here

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


    After running the fakecamera node I can see that it is working since using ros2 topic list shows the new topics it creates: /left/image and /right/image/

    # uniquenessRatio - rejects match if 2nd best match is too similar to best. Higher = stricter, fewer but more reliable matches
# disp12MaxDiff - matches left-to-right AND right-to-left, rejects pixels where both disagree. Catches occlusions and bad matches
# MODE_SGBM_3WAY - optimizes in more directions than default, smoother result, slightly slower
# speckleWindowSize - minimum size a connected region must be to be kept, small isolated blobs below this get zeroed
# speckleRange - max disparity variation allowed within a connected region to count as one blob
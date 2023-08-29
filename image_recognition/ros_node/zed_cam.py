#!/usr/bin/env python

## @package zed_camera_node
# This script captures images from a ZED camera, processes them, and publishes them using ROS topics and services.

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from zed_recognition.srv import CaptureImage  # Import the service definition

## Callback function for the 'capture_image' service.
# This function captures images from the ZED camera, processes them, and publishes them to the appropriate ROS topics.
# @param req The service request (unused).
# @return A success status indicating whether the operation was successful.
def capture_image_callback(req):
    ret, frame = zed.read()  # Read frame from ZED camera

    if ret:
        rgb_image = frame[:, :, :3]
        depth_image = frame[:, :, 3]

        rgb_image_msg = bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
        depth_image_msg = bridge.cv2_to_imgmsg(depth_image, '32FC1')

        pub_rgb.publish(rgb_image_msg)
        pub_depth.publish(depth_image_msg)

    return True  # Return success

## Main function.
# Initializes the ROS node, sets up the ZED camera, creates publishers and a service,
# and enters the ROS spin loop to process events.
def main():
    rospy.init_node('zed_camera_node')  # Initialize ROS node
    bridge = CvBridge()  # Initialize CvBridge for image conversion

    zed = cv2.VideoCapture(cv2.CAP_TEGRA_CUDA + 2)  # Open ZED camera using OpenCV

    pub_rgb = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)  # Publish RGB images
    pub_depth = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)  # Publish depth images

    service = rospy.Service('capture_image', CaptureImage, capture_image_callback)  # Create the 'capture_image' service

    rospy.spin()  # Enter the ROS spin loop

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

/**
 * @file zed_camera_node.cpp
 * @brief This script captures images from a ZED camera, processes them, and publishes them using ROS topics and services.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zed_recognition/CaptureImage.h>  // Import the service definition

cv::VideoCapture zed;  // ZED camera capture
cv::Mat frame;  // Captured frame from ZED camera
cv_bridge::CvImage rgb_image_msg;  // ROS message for RGB image
cv_bridge::CvImage depth_image_msg;  // ROS message for depth image
ros::Publisher pub_rgb;
ros::Publisher pub_depth;

/**
 * @brief Callback function for the 'capture_image' service.
 * This function captures images from the ZED camera, processes them, and publishes them to the appropriate ROS topics.
 * @param req The service request (unused).
 * @param res The service response indicating the success status.
 * @return A success status indicating whether the operation was successful.
 */
bool captureImageCallback(zed_recognition::CaptureImage::Request& req, zed_recognition::CaptureImage::Response& res){ 
    
    zed >> frame;  // Read frame from ZED camera

    if (!frame.empty()){
        cv::Mat rgb_image = frame(cv::Rect(0, 0, frame.cols, frame.rows)).clone();
        cv::Mat depth_image = frame(cv::Rect(0, 0, frame.cols, frame.rows)).clone();

        rgb_image_msg.header.stamp = ros::Time::now();
        rgb_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        rgb_image_msg.image = rgb_image;
        pub_rgb.publish(rgb_image_msg.toImageMsg());

        depth_image_msg.header.stamp = ros::Time::now();
        depth_image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth_image_msg.image = depth_image;
        pub_depth.publish(depth_image_msg.toImageMsg());
    }

    res.success = true;
    return true;
}

/**
 * @brief Main function.
 * Initializes the ROS node, sets up the ZED camera, creates publishers and a service,
 * and enters the ROS spin loop to process events.
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "zed_camera_node");  // Initialize ROS node
    ros::NodeHandle nh;

    zed.open(2);  // Open ZED camera using OpenCV (assuming index 2)
    if (!zed.isOpened()){
        ROS_ERROR("Failed to open ZED camera!");
        return 1;
    }

    pub_rgb = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 10);  // Publish RGB images
    pub_depth = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 10);  // Publish depth images

    ros::ServiceServer service = nh.advertiseService("capture_image", captureImageCallback);  // Create the 'capture_image' service

    ros::spin();  // Enter the ROS spin loop

    return 0;
}

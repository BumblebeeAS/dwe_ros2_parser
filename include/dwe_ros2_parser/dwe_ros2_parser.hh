#ifndef DWE_ROS2_PARSER_HH_
#define DWE_ROS2_PARSER_HH_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <csignal>
#include <thread>
#include <filesystem>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;

class DWE_Ros2_Parser : public rclcpp::Node {

    // Methods
    public:
        DWE_Ros2_Parser();
        ~DWE_Ros2_Parser();

    // Methods
    private:
        void fetch_ros_parameters();
        void dwe_loop();

    // Members
    private:
        cv::Mat camera_matrix_, dist_coeffs_;
        std::string calib_file_path_;
        // ROS2 Parameters
        string image_topic_, save_folder_, image_prefix_, device_, camera_info_topic_;
        int width_, height_, framerate_, exposure_;
        bool auto_exposure_, show_image_, use_h264_, save_images_;

        // ROS2 variables
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

};

#endif
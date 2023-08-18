#pragma once 

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <atomic>

#include "apriltag_detector.h"

#include<opencv2/opencv.hpp>


#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <filesystem>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp_components/register_node_macro.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


using namespace std;
using namespace cv;

namespace airlab
{
    class multicam_tag_state_estimator : public rclcpp::Node, public ApriltagDetector
    {
        using CAM_SUBS = rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr;

    public:
        multicam_tag_state_estimator(const rclcpp::NodeOptions & options);
        
    private:
        enum COLOR{
              RED,
              GREEN,
              DEFAULT
          };
        // index map is needed to memorize compressed image subscribers
        std::unordered_map<string, int> _indexMap; 
        std::vector<CAM_SUBS> subscriptions_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr create3_state_pub_;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
        std::unordered_map<int, std::vector<geometry_msgs::msg::Point>> pubData_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::Mat _image;
        mutable std::mutex _mutex; 

    protected:
        void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, int index);
        
        void processImage(const cv::Mat &image, int camIndex);

        void state_callback(const tf2::Transform& tf, const COLOR& color, int id);    

        void publish_traj();

    }; 
}
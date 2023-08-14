#pragma once
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include "ComplementaryFilter.h"

using namespace std::chrono_literals;

namespace airlab
{
    class MapTransformer: public rclcpp::Node{
    public:
        MapTransformer(const std::string& fromTF, const std::string& toTF, double alpha); 
        tf2::Transform getPose() const; 

    private:
        std::string _fromTF, _toTF; 
        std::unique_ptr<ComplementaryFilter> _filter; 
        rclcpp::TimerBase::SharedPtr _timer;

        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
        

    protected:
        bool getTransformation(const std::string& fromFrameRel, const std::string &toFrameRel, geometry_msgs::msg::TransformStamped& transformStamped);
        void updateFilter(const geometry_msgs::msg::TransformStamped& transformStamped);
    };

    MapTransformer::MapTransformer(const std::string& fromTF, const std::string& toTF, double alpha):
    _fromTF(fromTF), _toTF(toTF), rclcpp::Node(fromTF + "_" + toTF)
    {
        _filter = std::make_unique<ComplementaryFilter>(alpha);
        _timer = this->create_wall_timer(15ms, [this] { 
            geometry_msgs::msg::TransformStamped transformStamped; 
            if (getTransformation(_fromTF, _toTF, transformStamped))
                updateFilter(transformStamped); 

        });
            
    }

    bool MapTransformer::getTransformation(const std::string& fromFrameRel, const std::string &toFrameRel, geometry_msgs::msg::TransformStamped& transformStamped)
    {
        bool success = false;
        try{
            transformStamped = _tf_buffer->lookupTransform(
                    fromFrameRel, toFrameRel,
                    tf2::TimePointZero);
            success = true;

        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        }

        return success;
    }

    void MapTransformer::updateFilter(const geometry_msgs::msg::TransformStamped& transformStamped)
    {
        auto p = transformStamped.transform.translation; 
        auto q = transformStamped.transform.rotation; 
        POSE_ARRAY data{p.x, p.y, p.z, q.x, q.y, q.z, q.w};
        _filter->update(data);

    }

    tf2::Transform MapTransformer::getPose() const 
    {
        auto state = _filter->getState();
        tf2::Vector3 origin(state[0], state[1], state[2]); 
        tf2::Quaternion q(state[3], state[4], state[5], state[6]); 

        tf2::Transform pose; 
        pose.setOrigin(origin); 
        pose.setRotation(q); 
        return pose;   
    }
    
} // namespace airlab


#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class MonoInetrialNode : public rclcpp::Node
{
public:
    MonoInetrialNode(ORB_SLAM3::System* pSLAM);

    ~MonoInetrialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Pose publishing functions
    tf2::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat);
    void publish_ros_pose_tf(cv::Mat Tcw, rclcpp::Time current_frame_time);
    void publish_tf_transform(tf2::Transform tf_transform, rclcpp::Time current_frame_time);
    void publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time);

    ORB_SLAM3::System* m_SLAM;

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    
    // Pose publishing
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    std::string map_frame_id;
    std::string pose_frame_id;
    tf2::Matrix3x3 tf_orb_to_ros;

    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;
    std::unique_ptr<ORB_SLAM3::IMU::Point> lastImuMeasurement_;  // Store last IMU measurement for fallback
    bool hasLastImuMeasurement_;  // Flag to track if we have a valid last measurement

    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_;
    std::mutex bufMutexLeft_;
};

#endif

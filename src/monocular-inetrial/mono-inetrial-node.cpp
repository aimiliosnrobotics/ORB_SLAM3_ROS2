#include "mono-inetrial-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonoInetrialNode::MonoInetrialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    size_t depth = 10;
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
    qos_profile.depth = depth;

    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos_profile.reliability = reliability_policy;

    // The history policy determines how messages are saved until the message is taken by the reader.
    // KEEP_ALL saves all messages until they are taken.
    // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    // parameter.
    qos_profile.history = history_policy;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));

    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "image",
        qos,
        std::bind(&MonoInetrialNode::GrabImage, this, std::placeholders::_1));
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonoInetrialNode::GrabImu, this, _1));
    
    // Initialize pose publishing
    map_frame_id = "map";
    pose_frame_id = "odom";
    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", qos);
    
    // Initialize IMU fallback variables
    hasLastImuMeasurement_ = false;
    lastImuMeasurement_ = nullptr;
    
    syncThread_ = new std::thread(&MonoInetrialNode::SyncWithImu, this);
    std::cout << "slam changed" << std::endl;
}

MonoInetrialNode::~MonoInetrialNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInetrialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInetrialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msg);

    bufMutexLeft_.unlock();

}

cv::Mat MonoInetrialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInetrialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft;
        double tImLeft = 0;
        if (!imgLeftBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            
            // Let ORB_SLAM3 handle time synchronization internally
            // No manual time shift application

            bufMutexLeft_.lock();
            while (imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                // No manual time shift application
            }
            bufMutexLeft_.unlock();

            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    
                    // Store the last measurement for fallback
                    lastImuMeasurement_ = std::make_unique<ORB_SLAM3::IMU::Point>(acc, gyr, t);
                    hasLastImuMeasurement_ = true;
                    
                    imuBuf_.pop();
                }
            }
            else if (hasLastImuMeasurement_ && lastImuMeasurement_)
            {
                // Use last IMU measurement as fallback when buffer is empty
                std::cout << "Warning: Empty IMU buffer, using last measurement as fallback" << std::endl;
                vImuMeas.clear();
                vImuMeas.push_back(*lastImuMeasurement_);
            }
            bufMutex_.unlock();

            // std::cout<<"one frame has been sent"<<std::endl;
            cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(m_SLAM->TrackMonocular(imLeft, tImLeft, vImuMeas).matrix());
            
            // Publish pose only if SLAM is initialized and tracking
            if (!Tcw.empty() && m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK)
            {
                rclcpp::Time current_frame_time = rclcpp::Time(static_cast<int64_t>(tImLeft * 1e9));
                publish_ros_pose_tf(Tcw, current_frame_time);
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

tf2::Transform MonoInetrialNode::from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
{
    cv::Mat orb_rotation(3, 3, CV_32F);
    cv::Mat orb_translation(3, 1, CV_32F);

    orb_rotation = transformation_mat.rowRange(0, 3).colRange(0, 3);
    orb_translation = transformation_mat.rowRange(0, 3).col(3);

    tf2::Matrix3x3 tf_camera_rotation(
        orb_rotation.at<float>(0, 0), orb_rotation.at<float>(0, 1),
        orb_rotation.at<float>(0, 2), orb_rotation.at<float>(1, 0),
        orb_rotation.at<float>(1, 1), orb_rotation.at<float>(1, 2),
        orb_rotation.at<float>(2, 0), orb_rotation.at<float>(2, 1),
        orb_rotation.at<float>(2, 2));

    tf2::Vector3 tf_camera_translation(orb_translation.at<float>(0),
                                       orb_translation.at<float>(1),
                                       orb_translation.at<float>(2));

    // Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

void MonoInetrialNode::publish_ros_pose_tf(cv::Mat Tcw, rclcpp::Time current_frame_time)
{
    if (!Tcw.empty())
    {
        tf2::Transform tf_transform = from_orb_to_ros_tf_transform(Tcw);
        publish_tf_transform(tf_transform, current_frame_time);
        publish_pose_stamped(tf_transform, current_frame_time);
    }
}

void MonoInetrialNode::publish_tf_transform(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster(this);

    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = header;
    tf_msg.child_frame_id = pose_frame_id;
    tf_msg.transform = tf2::toMsg(tf_transform);

    tf_broadcaster.sendTransform(tf_msg);
}

void MonoInetrialNode::publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_frame_time;
    pose_msg.header.frame_id = pose_frame_id;
    
    // Convert tf2::Transform to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf_transform, pose);
    pose_msg.pose = pose;

    pose_pub->publish(pose_msg);
}

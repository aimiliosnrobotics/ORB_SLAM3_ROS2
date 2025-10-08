#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    // Create IMU subscription with BEST_EFFORT QoS to match camera publisher
    rclcpp::QoS imu_qos(1000);
    imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    subImu_ = this->create_subscription<ImuMsg>("imu", imu_qos, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    // Initialize pose publishing
    map_frame_id = "map";
    pose_frame_id = "odom";
    // Transform: ORB Z→ROS X (forward), ORB X→ROS Y (right), ORB Y→ROS Z (up)
    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", 10);

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
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

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

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
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            // Try to get pose from TrackStereo return value like TrackMonocular
            cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas).matrix());

            // Publish camera pose only if SLAM is initialized and tracking
            if (!Tcw.empty() && SLAM_->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK)
            {
                rclcpp::Time current_frame_time = rclcpp::Time(static_cast<int64_t>(tImLeft * 1e9));
                publish_ros_pose_tf(Tcw, current_frame_time);
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

tf2::Transform StereoInertialNode::from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
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

    // Transform from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros(tf_orb_to_ros);
    tf2::Transform tf_orb_to_ros_transform(tf_orb_to_ros, tf2::Vector3(0, 0, 0));

    return tf_orb_to_ros_transform * tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

void StereoInertialNode::publish_ros_pose_tf(cv::Mat Tcw, rclcpp::Time current_frame_time)
{
    if (!Tcw.empty())
    {
        tf2::Transform tf_transform = from_orb_to_ros_tf_transform(Tcw);
        publish_tf_transform(tf_transform, current_frame_time);
        publish_pose_stamped(tf_transform, current_frame_time);
    }
}

void StereoInertialNode::publish_tf_transform(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster(this);

    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = header;
    tf_msg.child_frame_id = pose_frame_id;

    tf2::Vector3 translation = tf_transform.getOrigin();
    tf_msg.transform.translation.x = translation.getX();
    tf_msg.transform.translation.y = translation.getY();
    tf_msg.transform.translation.z = translation.getZ();

    tf2::Quaternion quat = tf_transform.getRotation();
    tf_msg.transform.rotation.x = quat.getX();
    tf_msg.transform.rotation.y = quat.getY();
    tf_msg.transform.rotation.z = quat.getZ();
    tf_msg.transform.rotation.w = quat.getW();

    tf_broadcaster.sendTransform(tf_msg);
}

void StereoInertialNode::publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
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

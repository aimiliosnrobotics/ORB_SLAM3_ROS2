#!/usr/bin/env python3

"""
IMU Frame Corrector Node

This node transforms IMU data from the actual hardware frame to the expected
convention that ORB_SLAM3 and Kalibr expect.

Actual IMU frame: X-down, Y-right, Z-forward
Expected IMU frame: X-forward, Y-left, Z-up
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUFrameCorrector(Node):
    def __init__(self):
        super().__init__('imu_frame_corrector')
        
        # Create the frame correction transformation
        # Transform from actual frame (X-down, Y-right, Z-forward) 
        # to expected frame (X-forward, Y-left, Z-up)
        
        # Testing approach: Start with no transformation to establish baseline
        # Then adjust based on actual ORB_SLAM3 behavior
        
        # Time shift only - no frame transformation
        # From Kalibr: timeshift_cam_imu: -0.052995580757996576
        # Camera timestamp = IMU timestamp + time_shift
        # So we need to subtract the time shift from IMU timestamps
        self.time_shift = -0.05991748202629507  # seconds
        # self.time_shift = -0.052995580757996576  # seconds
        
        # Identity matrix - no frame transformation
        self.frame_correction = np.array([
            [1, 0, 0],   # X_expected = X_actual (no change)
            [0, 1, 0],   # Y_expected = Y_actual (no change)
            [0, 0, 1]    # Z_expected = Z_actual (no change)
        ])
        
        # Create rotation object for quaternion operations
        self.rotation_correction = R.from_matrix(self.frame_correction)
        
        self.get_logger().info("IMU Time Shift Corrector initialized")
        self.get_logger().info(f"Time shift: {self.time_shift} seconds")
        self.get_logger().info("No frame transformation (identity matrix)")
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Input: IMU data with gravity
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_corrected',  # Output: time-shifted IMU data
            10
        )
        
        self.get_logger().info("Subscribing to /imu/data")
        self.get_logger().info("Publishing to /imu/data_corrected")

    def imu_callback(self, msg):
        """Apply time shift to IMU data"""
        
        # Create corrected message
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        
        # Apply time shift to timestamp
        # Camera timestamp = IMU timestamp + time_shift
        # So IMU timestamp = Camera timestamp - time_shift
        # We need to subtract the time shift from IMU timestamp
        original_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        corrected_time = original_time - self.time_shift
        
        corrected_msg.header.stamp.sec = int(corrected_time)
        corrected_msg.header.stamp.nanosec = int((corrected_time - int(corrected_time)) * 1e9)
        corrected_msg.header.frame_id = 'imu_link_corrected'
        
        # Copy IMU data without transformation (identity matrix)
        corrected_msg.linear_acceleration.x = msg.linear_acceleration.x
        corrected_msg.linear_acceleration.y = msg.linear_acceleration.y
        corrected_msg.linear_acceleration.z = msg.linear_acceleration.z
        
        corrected_msg.angular_velocity.x = msg.angular_velocity.x
        corrected_msg.angular_velocity.y = msg.angular_velocity.y
        corrected_msg.angular_velocity.z = msg.angular_velocity.z
        
        # Copy orientation without transformation (identity matrix)
        corrected_msg.orientation.x = msg.orientation.x
        corrected_msg.orientation.y = msg.orientation.y
        corrected_msg.orientation.z = msg.orientation.z
        corrected_msg.orientation.w = msg.orientation.w
        
        # Copy covariance matrices (they should remain the same)
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_msg.orientation_covariance = msg.orientation_covariance
        
        # Publish corrected message
        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    
    imu_corrector = IMUFrameCorrector()
    
    try:
        rclpy.spin(imu_corrector)
    except KeyboardInterrupt:
        pass
    finally:
        imu_corrector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

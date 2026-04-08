#!/usr/bin/env python3
"""
ROS IMU Node for GY-91 (MPU6500)
Reads IMU data and publishes to /imu topic

NO SOFTWARE AXIS REMAPPING - TF handles coordinate transformations via URDF
"""

import rospy
import smbus
import time
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

# MPU6500 registers
MPU6500_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
CONFIG = 0x1A
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

class MPU6500IMU:
    def __init__(self):
        rospy.init_node('imu_node', anonymous=False)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # Hz
        self.calibration_samples = rospy.get_param('~calibration_samples', 100)
        
        # I2C setup
        self.bus = smbus.SMBus(1)
        self.address = MPU6500_ADDR
        
        # Sensitivity scales (based on ±2g and ±250°/s configuration)
        self.accel_scale = 16384.0  # LSB/g
        self.gyro_scale = 131.0     # LSB/(°/s)
        
        # Gyroscope bias (will be calibrated on startup)
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        
        # Publisher
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        
        # Initialize sensor
        self.initialize_sensor()
        
        # Calibrate gyroscope bias
        self.calibrate_gyro_bias()
        
        rospy.loginfo("IMU node initialized and ready")
    
    def initialize_sensor(self):
        """Configure MPU6500 sensor"""
        rospy.loginfo("Initializing MPU6500...")
        
        # Wake up sensor
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # Configure accelerometer (±2g, best sensitivity)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
        
        # Configure gyroscope (±250°/s, best sensitivity)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        
        # Set digital low-pass filter (bandwidth = 20Hz)
        self.bus.write_byte_data(self.address, CONFIG, 0x04)
        
        time.sleep(0.1)
        rospy.loginfo("MPU6500 configured successfully")
    
    def read_word_2c(self, reg):
        """Read signed 16-bit value from register"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def read_raw_data(self):
        """Read raw accelerometer and gyroscope data"""
        # Read accelerometer (m/s²)
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / self.accel_scale * 9.80665
        accel_y = self.read_word_2c(ACCEL_XOUT_H + 2) / self.accel_scale * 9.80665
        accel_z = self.read_word_2c(ACCEL_XOUT_H + 4) / self.accel_scale * 9.80665
        
        # Read gyroscope (rad/s)
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / self.gyro_scale * (math.pi / 180.0)
        gyro_y = self.read_word_2c(GYRO_XOUT_H + 2) / self.gyro_scale * (math.pi / 180.0)
        gyro_z = self.read_word_2c(GYRO_XOUT_H + 4) / self.gyro_scale * (math.pi / 180.0)
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    def calibrate_gyro_bias(self):
        """Calibrate gyroscope bias (measure drift when stationary)"""
        rospy.loginfo("Calibrating gyroscope bias...")
        rospy.loginfo("Keep robot STATIONARY for 5 seconds...")
        
        gyro_x_samples = []
        gyro_y_samples = []
        gyro_z_samples = []
        
        for i in range(self.calibration_samples):
            _, _, _, gx, gy, gz = self.read_raw_data()
            gyro_x_samples.append(gx)
            gyro_y_samples.append(gy)
            gyro_z_samples.append(gz)
            time.sleep(0.05)  # 20Hz sampling
        
        # Calculate average bias
        self.gyro_bias_x = sum(gyro_x_samples) / len(gyro_x_samples)
        self.gyro_bias_y = sum(gyro_y_samples) / len(gyro_y_samples)
        self.gyro_bias_z = sum(gyro_z_samples) / len(gyro_z_samples)
        
        rospy.loginfo(f"Gyro bias calibrated:")
        rospy.loginfo(f"  X: {self.gyro_bias_x * 180/math.pi:.2f} deg/s")
        rospy.loginfo(f"  Y: {self.gyro_bias_y * 180/math.pi:.2f} deg/s")
        rospy.loginfo(f"  Z: {self.gyro_bias_z * 180/math.pi:.2f} deg/s")
    
    def publish_imu_data(self):
        """Read IMU and publish to /imu topic"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            try:
                # Read raw IMU data
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_raw_data()
                
                # Apply gyroscope bias compensation
                gyro_x -= self.gyro_bias_x
                gyro_y -= self.gyro_bias_y
                gyro_z -= self.gyro_bias_z
                
                # NO AXIS REMAPPING - Use raw IMU frame data
                # The TF system (via URDF) will handle coordinate transformation
                
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = self.frame_id
                
                # Linear acceleration (raw IMU frame)
                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                
                # Angular velocity (raw IMU frame)
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z
                
                # Orientation (not provided by this IMU - set to unknown)
                # Note: robot_localization will estimate orientation from gyro integration
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 1.0
                
                # Covariance matrices
                # Orientation covariance: Set to -1 to indicate orientation is not provided
                imu_msg.orientation_covariance[0] = -1.0
                
                # Angular velocity covariance (based on MPU6500 datasheet)
                # ±250°/s range, noise density ~0.01°/s/√Hz
                gyro_variance = 0.0001  # (rad/s)²
                imu_msg.angular_velocity_covariance[0] = gyro_variance
                imu_msg.angular_velocity_covariance[4] = gyro_variance
                imu_msg.angular_velocity_covariance[8] = gyro_variance
                
                # Linear acceleration covariance (based on MPU6500 datasheet)
                # ±2g range, noise density ~400 μg/√Hz
                accel_variance = 0.01  # (m/s²)²
                imu_msg.linear_acceleration_covariance[0] = accel_variance
                imu_msg.linear_acceleration_covariance[4] = accel_variance
                imu_msg.linear_acceleration_covariance[8] = accel_variance
                
                # Publish
                self.imu_pub.publish(imu_msg)
                
            except Exception as e:
                rospy.logerr(f"Error reading IMU: {e}")
            
            rate.sleep()
    
    def run(self):
        """Main loop"""
        try:
            self.publish_imu_data()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        imu = MPU6500IMU()
        imu.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU node terminated")
    except Exception as e:
        rospy.logerr(f"IMU node error: {e}")
        import traceback
        traceback.print_exc()
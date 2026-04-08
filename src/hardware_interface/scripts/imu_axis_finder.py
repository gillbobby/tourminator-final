#!/usr/bin/env python3
"""
IMU Axis Orientation Finder
Physical test to determine which IMU axis points in which robot direction

This will help you configure the IMU coordinate transform in ROS
"""

import smbus
import time
import math

MPU6500_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

class IMUAxisFinder:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = MPU6500_ADDR
        self.accel_scale = 16384.0
        self.gyro_scale = 131.0
        
        # Wake up and configure sensor
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        time.sleep(0.1)
    
    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def get_accel_data(self):
        x = self.read_word_2c(ACCEL_XOUT_H) / self.accel_scale
        y = self.read_word_2c(ACCEL_XOUT_H + 2) / self.accel_scale
        z = self.read_word_2c(ACCEL_XOUT_H + 4) / self.accel_scale
        return (x, y, z)
    
    def get_gyro_data(self):
        x = self.read_word_2c(GYRO_XOUT_H) / self.gyro_scale
        y = self.read_word_2c(GYRO_XOUT_H + 2) / self.gyro_scale
        z = self.read_word_2c(GYRO_XOUT_H + 4) / self.gyro_scale
        return (x, y, z)
    
    def find_dominant_axis(self, x, y, z):
        """Find which axis has the largest absolute value"""
        abs_vals = [abs(x), abs(y), abs(z)]
        max_val = max(abs_vals)
        max_idx = abs_vals.index(max_val)
        
        values = [x, y, z]
        axes = ['X', 'Y', 'Z']
        
        return axes[max_idx], values[max_idx]

def test_gravity_direction():
    """Use gravity to determine which axis points up"""
    print("🌍 GRAVITY TEST - Finding UP direction")
    print("=" * 60)
    print("Keep robot sitting FLAT on table...")
    input("Press ENTER when ready...")
    
    imu = IMUAxisFinder()
    
    # Take multiple readings
    readings = []
    for i in range(10):
        ax, ay, az = imu.get_accel_data()
        readings.append((ax, ay, az))
        time.sleep(0.1)
    
    # Average the readings
    avg_x = sum(r[0] for r in readings) / len(readings)
    avg_y = sum(r[1] for r in readings) / len(readings)
    avg_z = sum(r[2] for r in readings) / len(readings)
    
    print(f"\nAccelerometer readings:")
    print(f"  IMU X-axis: {avg_x:+7.3f}g")
    print(f"  IMU Y-axis: {avg_y:+7.3f}g")
    print(f"  IMU Z-axis: {avg_z:+7.3f}g")
    
    # Find which axis is pointing up (should be ~+1g)
    axis, value = imu.find_dominant_axis(avg_x, avg_y, avg_z)
    
    print(f"\n✅ IMU {axis}-axis points {'UP' if value > 0 else 'DOWN'}")
    print(f"   (Gravity reading: {value:+.3f}g)")
    
    return axis, value

def test_forward_direction():
    """Use motion to determine which axis points forward"""
    print("\n🚗 FORWARD/BACKWARD TEST")
    print("=" * 60)
    print("Instructions:")
    print("1. Hold the robot in the air")
    print("2. When test starts, PUSH robot FORWARD quickly (like a car)")
    print("3. Keep pushing for 2 seconds")
    print()
    input("Press ENTER when ready to start test...")
    
    imu = IMUAxisFinder()
    
    print("\n🏃 GO! Push robot FORWARD NOW!")
    print("Keep pushing...")
    
    # Collect data during motion
    accel_readings = []
    for i in range(20):
        ax, ay, az = imu.get_accel_data()
        accel_readings.append((ax, ay, az))
        time.sleep(0.1)
    
    print("✅ Done collecting data\n")
    
    # Find peak accelerations (ignore gravity by looking at changes)
    max_x = max(abs(r[0]) for r in accel_readings[5:15])  # Middle section
    max_y = max(abs(r[1]) for r in accel_readings[5:15])
    max_z = max(abs(r[2]) for r in accel_readings[5:15])
    
    print(f"Peak accelerations during forward push:")
    print(f"  IMU X-axis: {max_x:.3f}g")
    print(f"  IMU Y-axis: {max_y:.3f}g")
    print(f"  IMU Z-axis: {max_z:.3f}g")
    
    axis, value = imu.find_dominant_axis(max_x, max_y, max_z)
    
    print(f"\n✅ IMU {axis}-axis is aligned with FORWARD/BACKWARD motion")
    
    return axis

def test_rotation_direction():
    """Use rotation to determine yaw axis"""
    print("\n🔄 ROTATION TEST - Finding YAW axis")
    print("=" * 60)
    print("Instructions:")
    print("1. Hold robot in the air")
    print("2. When test starts, SPIN robot CLOCKWISE (when viewed from above)")
    print("3. Like turning a steering wheel to the right")
    print()
    input("Press ENTER when ready to start test...")
    
    imu = IMUAxisFinder()
    
    print("\n🌀 GO! Spin robot CLOCKWISE NOW!")
    print("Keep spinning...")
    
    # Collect gyro data during rotation
    gyro_readings = []
    for i in range(20):
        gx, gy, gz = imu.get_gyro_data()
        gyro_readings.append((gx, gy, gz))
        time.sleep(0.1)
    
    print("✅ Done collecting data\n")
    
    # Find which axis had the most rotation
    avg_gx = sum(abs(r[0]) for r in gyro_readings[5:15]) / 10
    avg_gy = sum(abs(r[1]) for r in gyro_readings[5:15]) / 10
    avg_gz = sum(abs(r[2]) for r in gyro_readings[5:15]) / 10
    
    print(f"Average rotation rates during clockwise spin:")
    print(f"  IMU X-axis: {avg_gx:.1f}°/s")
    print(f"  IMU Y-axis: {avg_gy:.1f}°/s")
    print(f"  IMU Z-axis: {avg_gz:.1f}°/s")
    
    axis, value = imu.find_dominant_axis(avg_gx, avg_gy, avg_gz)
    
    # Determine sign (positive = counterclockwise in right-hand rule)
    sum_values = sum(r[['X','Y','Z'].index(axis)] for r in gyro_readings[5:15])
    direction = "NEGATIVE" if sum_values < 0 else "POSITIVE"
    
    print(f"\n✅ IMU {axis}-axis is the YAW (spin) axis")
    print(f"   Clockwise rotation = {direction} {axis} value")
    
    return axis, direction

def generate_ros_config(up_axis, up_sign, forward_axis, yaw_axis, yaw_clockwise_sign):
    """Generate ROS configuration based on test results"""
    print("\n" + "=" * 60)
    print("📋 ROS CONFIGURATION")
    print("=" * 60)
    
    print("\n🎯 IMU Orientation Summary:")
    print(f"  UP direction:      IMU {up_axis}-axis ({'+' if up_sign > 0 else '-'})")
    print(f"  FORWARD direction: IMU {forward_axis}-axis")
    print(f"  YAW axis:          IMU {yaw_axis}-axis")
    print(f"  Clockwise spin:    {yaw_clockwise_sign} {yaw_axis}")
    
    print("\n📝 Add this to your URDF (robot.urdf.xacro):")
    print("=" * 60)
    
    # Determine RPY values for transform
    # Standard robot frame: X=forward, Y=left, Z=up
    
    print(f"""
<!-- IMU Link and Joint -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="base_to_imu" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <!-- Position: adjust x,y,z to actual IMU mounting location on your robot -->
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
  <!-- NOTE: You may need to adjust the RPY values based on physical mounting -->
</joint>
""")
    
    print("\n📝 In your IMU ROS node, you'll need to remap axes:")
    print("=" * 60)
    
    # Standard ROS REP-103: X=forward, Y=left, Z=up
    print(f"""
# In your IMU node configuration:
# Physical IMU axes → ROS standard frame

Robot Frame Mapping:
  X (forward) ← IMU {forward_axis}-axis
  Y (left)    ← IMU {'X' if forward_axis != 'X' else 'Y'}-axis  
  Z (up)      ← IMU {up_axis}-axis ({'+' if up_sign > 0 else '-'})
    
# Most important for navigation: YAW axis
# IMU {yaw_axis}-axis provides heading data
# Clockwise = {yaw_clockwise_sign} {yaw_axis} value
""")

def main():
    print("🔍 IMU AXIS ORIENTATION FINDER")
    print("=" * 60)
    print("This test will determine how your IMU is physically oriented")
    print("on your robot so we can configure ROS correctly.")
    print()
    
    try:
        # Test 1: Find UP direction using gravity
        up_axis, up_sign = test_gravity_direction()
        
        # Test 2: Find FORWARD direction using acceleration
        forward_axis = test_forward_direction()
        
        # Test 3: Find YAW axis using rotation
        yaw_axis, yaw_clockwise_sign = test_rotation_direction()
        
        # Generate configuration
        generate_ros_config(up_axis, up_sign, forward_axis, yaw_axis, yaw_clockwise_sign)
        
        print("\n✅ Orientation testing complete!")
        print("\nSave these results - you'll need them for ROS integration!")
        
    except KeyboardInterrupt:
        print("\n\n🛑 Test interrupted")
    except Exception as e:
        print(f"\n💥 Error: {e}")

if __name__ == "__main__":
    main()
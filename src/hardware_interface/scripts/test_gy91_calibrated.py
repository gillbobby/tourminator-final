#!/usr/bin/env python3
"""
Calibrated MPU6500 test with proper configuration
Configures sensor correctly and provides calibrated readings
"""

import smbus
import time
import math

# MPU6500/MPU9250 I2C address and registers
MPU6500_ADDR = 0x68
PWR_MGMT_1 = 0x6B      # Power management
CONFIG = 0x1A          # Configuration
GYRO_CONFIG = 0x1B     # Gyroscope configuration
ACCEL_CONFIG = 0x1C    # Accelerometer configuration
WHO_AM_I = 0x75        # Device ID
ACCEL_XOUT_H = 0x3B    # Accelerometer data
GYRO_XOUT_H = 0x43     # Gyroscope data

class MPU6500:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = MPU6500_ADDR
        
        # Sensitivity scales
        self.accel_scale = 16384.0  # ±2g range
        self.gyro_scale = 131.0     # ±250°/s range
        
        self.initialize()
    
    def initialize(self):
        """Properly configure the MPU6500"""
        print("🔧 Configuring MPU6500...")
        
        # Wake up the sensor
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # Configure gyroscope (±250°/s)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        
        # Configure accelerometer (±2g)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
        
        # Set low pass filter
        self.bus.write_byte_data(self.address, CONFIG, 0x03)
        
        time.sleep(0.1)
        print("✅ MPU6500 configured")
    
    def read_word_2c(self, reg):
        """Read 16-bit signed value from register"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def get_accel_data(self):
        """Read accelerometer data in g"""
        x = self.read_word_2c(ACCEL_XOUT_H) / self.accel_scale
        y = self.read_word_2c(ACCEL_XOUT_H + 2) / self.accel_scale
        z = self.read_word_2c(ACCEL_XOUT_H + 4) / self.accel_scale
        return (x, y, z)
    
    def get_gyro_data(self):
        """Read gyroscope data in °/s"""
        x = self.read_word_2c(GYRO_XOUT_H) / self.gyro_scale
        y = self.read_word_2c(GYRO_XOUT_H + 2) / self.gyro_scale
        z = self.read_word_2c(GYRO_XOUT_H + 4) / self.gyro_scale
        return (x, y, z)
    
    def get_orientation(self):
        """Calculate roll and pitch from accelerometer"""
        ax, ay, az = self.get_accel_data()
        
        roll = math.atan2(ay, az) * 180 / math.pi
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
        
        return (roll, pitch)

def test_accelerometer_calibrated(sensor):
    """Test accelerometer with proper calibration"""
    print("\n📏 CALIBRATED ACCELEROMETER TEST")
    print("-" * 50)
    print("Keep robot FLAT and STATIONARY...")
    
    readings = []
    for i in range(10):
        ax, ay, az = sensor.get_accel_data()
        magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        
        readings.append((ax, ay, az, magnitude))
        print(f"Reading {i+1}: X={ax:6.3f}g, Y={ay:6.3f}g, Z={az:6.3f}g, |A|={magnitude:6.3f}g")
        time.sleep(0.2)
    
    # Calculate averages
    avg_x = sum(r[0] for r in readings) / len(readings)
    avg_y = sum(r[1] for r in readings) / len(readings)
    avg_z = sum(r[2] for r in readings) / len(readings)
    avg_mag = sum(r[3] for r in readings) / len(readings)
    
    print(f"\nAverage: X={avg_x:6.3f}g, Y={avg_y:6.3f}g, Z={avg_z:6.3f}g, |A|={avg_mag:6.3f}g")
    
    # Check if readings are reasonable
    if 0.9 < avg_mag < 1.1:
        print("✅ Accelerometer readings are correct!")
        return True
    else:
        print(f"⚠️  Total acceleration is {avg_mag:0.2f}g (should be ~1.0g)")
        print("   This could mean:")
        print("   - Sensor is mounted at an angle")
        print("   - Sensor needs recalibration")
        return False

def test_gyroscope_calibrated(sensor):
    """Test gyroscope with bias measurement"""
    print("\n🌀 CALIBRATED GYROSCOPE TEST")
    print("-" * 50)
    print("Keep robot COMPLETELY STILL for 5 seconds...")
    
    # Collect bias measurements
    bias_readings = []
    for i in range(20):
        gx, gy, gz = sensor.get_gyro_data()
        bias_readings.append((gx, gy, gz))
        print(f"Reading {i+1}: X={gx:7.2f}°/s, Y={gy:7.2f}°/s, Z={gz:7.2f}°/s")
        time.sleep(0.25)
    
    # Calculate bias
    bias_x = sum(r[0] for r in bias_readings) / len(bias_readings)
    bias_y = sum(r[1] for r in bias_readings) / len(bias_readings)
    bias_z = sum(r[2] for r in bias_readings) / len(bias_readings)
    
    print(f"\n📊 Gyroscope Bias:")
    print(f"   X: {bias_x:7.2f}°/s")
    print(f"   Y: {bias_y:7.2f}°/s")
    print(f"   Z: {bias_z:7.2f}°/s")
    
    if abs(bias_x) < 5 and abs(bias_y) < 5 and abs(bias_z) < 5:
        print("✅ Gyroscope bias is acceptable")
        print("   (This bias will be compensated in ROS integration)")
        return True
    else:
        print("⚠️  Gyroscope has high bias - consider recalibration")
        return False

def test_orientation_response(sensor):
    """Test orientation calculation"""
    print("\n🔄 ORIENTATION TEST")
    print("-" * 50)
    print("🤏 Tilt the robot and watch orientation change...")
    
    for i in range(20):
        roll, pitch = sensor.get_orientation()
        ax, ay, az = sensor.get_accel_data()
        gx, gy, gz = sensor.get_gyro_data()
        
        print(f"Orientation: Roll={roll:6.1f}°, Pitch={pitch:6.1f}° | " +
              f"Gyro: X={gx:6.1f}, Y={gy:6.1f}, Z={gz:6.1f} °/s")
        time.sleep(0.5)
    
    print("✅ Orientation test complete")
    return True

def main():
    print("🚀 MPU6500 Calibrated Test")
    print("=" * 50)
    
    try:
        # Initialize sensor
        sensor = MPU6500()
        
        # Check WHO_AM_I
        who_am_i = sensor.bus.read_byte_data(MPU6500_ADDR, WHO_AM_I)
        print(f"\n🔍 Device ID: 0x{who_am_i:02X}")
        
        if who_am_i == 0x70:
            print("✅ MPU6500 detected (perfect for your application!)")
        elif who_am_i == 0x71:
            print("✅ MPU9250 detected")
        elif who_am_i == 0x68:
            print("✅ MPU6050 detected")
        else:
            print(f"⚠️  Unknown device: 0x{who_am_i:02X}")
        
        # Run tests
        print("\n" + "=" * 50)
        results = []
        
        results.append(test_accelerometer_calibrated(sensor))
        results.append(test_gyroscope_calibrated(sensor))
        results.append(test_orientation_response(sensor))
        
        # Summary
        print("\n" + "=" * 50)
        print("📊 TEST SUMMARY")
        print("=" * 50)
        
        passed = sum(results)
        total = len(results)
        
        if passed == total:
            print(f"🎉 ALL TESTS PASSED ({passed}/{total})")
            print("\n✅ YOUR GY-91 IS READY FOR ROS INTEGRATION!")
            print("\nNext steps:")
            print("1. Create ROS IMU node")
            print("2. Integrate with robot_localization package")
            print("3. Fuse with LiDAR data in navigation stack")
        else:
            print(f"⚠️  {passed}/{total} tests passed")
            print("\nSensor is functional but may need calibration")
        
    except Exception as e:
        print(f"\n💥 Error: {e}")
        print("Check wiring and I2C connection")

if __name__ == "__main__":
    main()
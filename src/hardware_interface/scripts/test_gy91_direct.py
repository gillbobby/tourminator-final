#!/usr/bin/env python3
"""
Direct I2C test for GY-91 MPU9250
Bypasses libraries and talks directly to the sensor
"""

import smbus
import time

# MPU9250 I2C address and registers
MPU9250_ADDR = 0x68
PWR_MGMT_1 = 0x6B    # Power management register
WHO_AM_I = 0x75      # Device ID register
ACCEL_XOUT_H = 0x3B  # Accelerometer data start
GYRO_XOUT_H = 0x43   # Gyroscope data start

def test_mpu9250_direct():
    """Direct communication test with MPU9250"""
    print("🔧 Direct MPU9250 I2C Test")
    print("=" * 50)
    
    try:
        # Initialize I2C bus
        bus = smbus.SMBus(1)
        print("✅ I2C bus initialized")
        
        # Step 1: Wake up MPU9250 (it starts in sleep mode!)
        print("\n1️⃣  Waking up MPU9250...")
        try:
            bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
            time.sleep(0.1)  # Wait for sensor to wake up
            print("✅ Wake-up command sent")
        except Exception as e:
            print(f"❌ Failed to wake sensor: {e}")
            return False
        
        # Step 2: Read WHO_AM_I register (should return 0x71)
        print("\n2️⃣  Reading WHO_AM_I register...")
        try:
            who_am_i = bus.read_byte_data(MPU9250_ADDR, WHO_AM_I)
            print(f"   WHO_AM_I = 0x{who_am_i:02X}")
            
            if who_am_i == 0x71:
                print("✅ MPU9250 identified correctly!")
            elif who_am_i == 0x73:
                print("✅ MPU9255 identified (compatible variant)")
            else:
                print(f"⚠️  Unexpected device ID: 0x{who_am_i:02X}")
        except Exception as e:
            print(f"❌ Failed to read WHO_AM_I: {e}")
            return False
        
        # Step 3: Read accelerometer data
        print("\n3️⃣  Reading accelerometer data...")
        try:
            for i in range(5):
                # Read 6 bytes starting from ACCEL_XOUT_H
                data = bus.read_i2c_block_data(MPU9250_ADDR, ACCEL_XOUT_H, 6)
                
                # Convert to signed 16-bit values
                accel_x = (data[0] << 8) | data[1]
                accel_y = (data[2] << 8) | data[3]
                accel_z = (data[4] << 8) | data[5]
                
                # Convert to signed
                if accel_x > 32767: accel_x -= 65536
                if accel_y > 32767: accel_y -= 65536
                if accel_z > 32767: accel_z -= 65536
                
                # Convert to g (assuming ±2g range, 16384 LSB/g)
                ax = accel_x / 16384.0
                ay = accel_y / 16384.0
                az = accel_z / 16384.0
                
                print(f"   Reading {i+1}: X={ax:6.3f}g, Y={ay:6.3f}g, Z={az:6.3f}g")
                time.sleep(0.2)
            
            print("✅ Accelerometer data reading successfully!")
        except Exception as e:
            print(f"❌ Failed to read accelerometer: {e}")
            return False
        
        # Step 4: Read gyroscope data
        print("\n4️⃣  Reading gyroscope data...")
        try:
            for i in range(5):
                # Read 6 bytes starting from GYRO_XOUT_H
                data = bus.read_i2c_block_data(MPU9250_ADDR, GYRO_XOUT_H, 6)
                
                # Convert to signed 16-bit values
                gyro_x = (data[0] << 8) | data[1]
                gyro_y = (data[2] << 8) | data[3]
                gyro_z = (data[4] << 8) | data[5]
                
                # Convert to signed
                if gyro_x > 32767: gyro_x -= 65536
                if gyro_y > 32767: gyro_y -= 65536
                if gyro_z > 32767: gyro_z -= 65536
                
                # Convert to °/s (assuming ±250°/s range, 131 LSB/°/s)
                gx = gyro_x / 131.0
                gy = gyro_y / 131.0
                gz = gyro_z / 131.0
                
                print(f"   Reading {i+1}: X={gx:7.2f}°/s, Y={gy:7.2f}°/s, Z={gz:7.2f}°/s")
                time.sleep(0.2)
            
            print("✅ Gyroscope data reading successfully!")
        except Exception as e:
            print(f"❌ Failed to read gyroscope: {e}")
            return False
        
        print("\n" + "=" * 50)
        print("🎉 MPU9250 IS WORKING!")
        print("   The sensor is responding correctly via I2C.")
        print("   The library issue can be fixed.")
        
        return True
        
    except Exception as e:
        print(f"\n💥 Error: {e}")
        return False

def check_i2c_connection():
    """Verify I2C device is visible"""
    print("🔍 Checking I2C connection...")
    try:
        bus = smbus.SMBus(1)
        
        # Try to read from device
        bus.read_byte(MPU9250_ADDR)
        print("✅ Device responds on I2C bus")
        return True
    except Exception as e:
        print(f"❌ Cannot communicate with device: {e}")
        print("\n💡 Troubleshooting:")
        print("   1. Check wiring: VCC, GND, SDA, SCL")
        print("   2. Verify I2C is enabled: sudo raspi-config")
        print("   3. Check power supply is 3.3V (NOT 5V!)")
        return False

def main():
    print("🚀 GY-91 Direct Hardware Test")
    print()
    
    # First check basic I2C connectivity
    if not check_i2c_connection():
        print("\n❌ Fix I2C connection first!")
        return
    
    print()
    
    # Run direct sensor test
    if test_mpu9250_direct():
        print("\n✅ NEXT STEPS:")
        print("   1. The hardware is working correctly")
        print("   2. Try alternative Python library:")
        print("      pip3 install mpu6050-raspberrypi")
        print("   3. Or proceed to ROS integration directly")
    else:
        print("\n❌ Hardware communication issue detected")
        print("   Check wiring and power supply")

if __name__ == "__main__":
    main()
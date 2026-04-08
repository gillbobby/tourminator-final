#!/usr/bin/env python3
"""
Focused Forward Direction Test
Determines which IMU axis aligns with robot's forward direction
"""

import smbus
import time
import math

MPU6500_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG = 0x1C

class IMU:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = MPU6500_ADDR
        self.accel_scale = 16384.0
        
        # Wake up and configure
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
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

def test_forward_axis():
    """Focused test for forward direction"""
    print("🎯 FORWARD AXIS TEST")
    print("=" * 70)
    print()
    print("📸 IMPORTANT: Identify your robot's FORWARD direction")
    print("   FORWARD = Direction the CAMERAS point")
    print("   FORWARD = Front of robot")
    print()
    print("🚨 CRITICAL INSTRUCTIONS:")
    print("   1. Hold robot in the air with BOTH HANDS")
    print("   2. Identify the FRONT (camera side)")
    print("   3. When test starts, SHOVE robot FORWARD aggressively")
    print("   4. Push HARD and FAST like launching it forward")
    print("   5. Keep pushing horizontally (NOT up or down)")
    print("   6. Push for FULL 2 seconds")
    print()
    print("💡 Think: You're aggressively pushing a toy car across a table")
    print()
    
    input("Press ENTER when ready...")
    print()
    
    imu = IMU()
    
    # Get baseline (before motion)
    print("📊 Taking baseline reading (keep still)...")
    baseline_readings = []
    for i in range(10):
        ax, ay, az = imu.get_accel_data()
        baseline_readings.append((ax, ay, az))
        time.sleep(0.05)
    
    base_x = sum(r[0] for r in baseline_readings) / 10
    base_y = sum(r[1] for r in baseline_readings) / 10
    base_z = sum(r[2] for r in baseline_readings) / 10
    
    print(f"   Baseline: X={base_x:+.3f}g, Y={base_y:+.3f}g, Z={base_z:+.3f}g")
    print()
    
    # Motion test
    print("⏱️  Starting in 3 seconds...")
    time.sleep(1)
    print("   2...")
    time.sleep(1)
    print("   1...")
    time.sleep(1)
    print()
    print("🏃 GO! PUSH FORWARD NOW! HARD AND FAST!")
    print("=" * 70)
    
    # Collect data during motion
    motion_readings = []
    for i in range(30):  # 3 seconds at 10Hz
        ax, ay, az = imu.get_accel_data()
        
        # Subtract baseline to remove gravity component
        ax_motion = ax - base_x
        ay_motion = ay - base_y
        az_motion = az - base_z
        
        motion_readings.append((ax_motion, ay_motion, az_motion))
        
        # Real-time feedback
        if i % 3 == 0:
            print(f"  [{i//3 + 1}/10] X={ax_motion:+.3f}g  Y={ay_motion:+.3f}g  Z={az_motion:+.3f}g")
        
        time.sleep(0.1)
    
    print("=" * 70)
    print("✅ Done collecting data")
    print()
    
    # Analysis
    print("📊 ANALYSIS - Motion-induced acceleration (gravity removed):")
    print("-" * 70)
    
    # Find peak accelerations during the push (middle section)
    peak_x = max(abs(r[0]) for r in motion_readings[5:25])
    peak_y = max(abs(r[1]) for r in motion_readings[5:25])
    peak_z = max(abs(r[2]) for r in motion_readings[5:25])
    
    # Also check which direction (positive or negative)
    sum_x = sum(r[0] for r in motion_readings[5:25])
    sum_y = sum(r[1] for r in motion_readings[5:25])
    sum_z = sum(r[2] for r in motion_readings[5:25])
    
    print(f"Peak accelerations during forward push:")
    print(f"  IMU X-axis: {peak_x:.3f}g  (sum: {sum_x:+.2f})")
    print(f"  IMU Y-axis: {peak_y:.3f}g  (sum: {sum_y:+.2f})")
    print(f"  IMU Z-axis: {peak_z:.3f}g  (sum: {sum_z:+.2f})")
    print()
    
    # Determine forward axis
    peaks = [peak_x, peak_y, peak_z]
    max_peak = max(peaks)
    max_idx = peaks.index(max_peak)
    axes = ['X', 'Y', 'Z']
    sums = [sum_x, sum_y, sum_z]
    
    forward_axis = axes[max_idx]
    forward_sign = '+' if sums[max_idx] > 0 else '-'
    
    print("=" * 70)
    
    # Validation
    if max_peak < 0.3:
        print("⚠️  WARNING: Acceleration too weak!")
        print(f"   Peak was only {max_peak:.3f}g - need at least 0.5g")
        print()
        print("❌ TEST FAILED - Not enough motion detected")
        print()
        print("💡 Tips for next attempt:")
        print("   1. Push MUCH harder - really shove it forward")
        print("   2. Make sure you're pushing toward the CAMERAS")
        print("   3. Push horizontally (parallel to ground)")
        print("   4. Don't be gentle - be aggressive!")
        return None, None
    
    print(f"✅ IMU {forward_axis}-axis aligns with FORWARD direction")
    print(f"   Forward motion = {forward_sign}{forward_axis} acceleration")
    print()
    
    # Additional interpretation
    print("📝 INTERPRETATION:")
    if forward_axis == 'Z':
        print("   ⚠️  Z-axis forward? This seems wrong!")
        print("   Did you push UP instead of FORWARD?")
        print("   Z-axis should be UP (gravity), not forward.")
    else:
        print(f"   ✅ {forward_axis}-axis is your robot's forward direction")
        print("   This makes sense for a typical IMU mounting.")
    
    return forward_axis, forward_sign

def main():
    print("🔍 IMU FORWARD DIRECTION FINDER")
    print()
    
    try:
        forward_axis, forward_sign = test_forward_axis()
        
        if forward_axis:
            print()
            print("=" * 70)
            print("✅ TEST COMPLETE!")
            print()
            print("🎯 FINAL RESULTS:")
            print(f"   UP direction:      Z-axis (from previous test)")
            print(f"   FORWARD direction: {forward_axis}-axis ({forward_sign})")
            print(f"   YAW axis:          Z-axis (from previous test)")
            print()
            print("Save this information for ROS configuration!")
        else:
            print()
            print("Please run the test again and push HARDER!")
        
    except KeyboardInterrupt:
        print("\n\n🛑 Test interrupted")
    except Exception as e:
        print(f"\n💥 Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
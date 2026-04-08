#!/usr/bin/env python3
"""
LiDAR Robot Boundary Calibration Tool

Usage:
1. Place robot's RIGHT edge against wall, run: python3 calibrate_boundaries.py --angle 0
2. Place robot's FRONT edge against wall, run: python3 calibrate_boundaries.py --angle 90
3. Place robot's LEFT edge against wall, run: python3 calibrate_boundaries.py --angle 180
4. Place robot's BACK edge against wall, run: python3 calibrate_boundaries.py --angle 270

This will give you the exact distances from LiDAR to each robot edge.
"""

import rospy
import math
import argparse
from sensor_msgs.msg import LaserScan

class BoundaryCalibrator:
    def __init__(self, target_angle):
        rospy.init_node('boundary_calibrator', anonymous=True)
        
        self.target_angle = target_angle
        self.readings_collected = []
        self.num_samples = 10  # Average multiple readings for accuracy
        
        # POLE FILTERING - Same as safety_supervisor.py
        self.pole_filter_enabled = True
        self.pole_angle_center = 280.0    # Angle where pole appears
        self.pole_angle_range = 20.0      # ±10° around pole
        self.pole_min_distance = 0.15     # Filter readings < 15cm
        
        # Subscribe to LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        print(f"\n🎯 CALIBRATING FOR {target_angle}° ({self.get_direction_name(target_angle)})")
        print(f"📍 Position robot so {self.get_edge_name(target_angle)} edge is against wall")
        print(f"⏳ Collecting {self.num_samples} readings...")
        print("   (Press Ctrl+C when done)\n")
    
    def get_direction_name(self, angle):
        """Convert angle to direction name"""
        directions = {0: "RIGHT", 90: "FORWARD", 180: "LEFT", 270: "BACKWARD"}
        return directions.get(angle, f"{angle}°")
    
    def get_edge_name(self, angle):
        """Convert angle to robot edge name"""
        edges = {0: "RIGHT", 90: "FRONT", 180: "LEFT", 270: "BACK"}
        return edges.get(angle, f"{angle}°")
    
    def get_boundary_name(self, angle):
        """Convert angle to boundary parameter name"""
        boundary_names = {
            0: "x_max",     # Right edge
            90: "y_max",    # Front edge  
            180: "x_min",   # Left edge (will be negative)
            270: "y_min"    # Back edge (will be negative)
        }
        return boundary_names.get(angle, f"angle_{angle}")
    
    def is_pole_detection(self, angle_deg, distance):
        """Filter out robot's camera pole detections - SAME AS SAFETY_SUPERVISOR"""
        if not self.pole_filter_enabled:
            return False
            
        angle_diff = abs(angle_deg - self.pole_angle_center)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff  # Handle wrap-around
        
        in_pole_sector = angle_diff <= (self.pole_angle_range / 2.0)
        too_close = distance <= self.pole_min_distance
        
        return in_pole_sector and too_close

    def scan_callback(self, scan_msg):
        """Process LiDAR scan and find distance at target angle"""
        
        # Find the LiDAR reading closest to our target angle
        target_index = self.find_closest_angle_index(scan_msg, self.target_angle)
        
        if target_index is not None:
            distance = scan_msg.ranges[target_index]
            
            # Validate reading
            if not (math.isnan(distance) or math.isinf(distance) or 
                   distance < scan_msg.range_min or distance > scan_msg.range_max):
                
                # CRITICAL: Check for pole detection
                if self.is_pole_detection(self.target_angle, distance):
                    print(f"⚠️  POLE DETECTED at {self.target_angle}° - distance {distance:.3f}m - IGNORING")
                    return  # Skip this reading
                
                self.readings_collected.append(distance)
                
                # Show progress
                if len(self.readings_collected) <= self.num_samples:
                    print(f"Reading {len(self.readings_collected):2d}/{self.num_samples}: "
                          f"{distance:.3f}m at {self.target_angle}°")
                
                # Calculate result when we have enough samples
                if len(self.readings_collected) == self.num_samples:
                    self.calculate_and_display_result()
                elif len(self.readings_collected) > self.num_samples:
                    # Continue showing live readings
                    avg_distance = sum(self.readings_collected[-self.num_samples:]) / self.num_samples
                    print(f"Live reading: {distance:.3f}m | Running avg: {avg_distance:.3f}m")
    
    def find_closest_angle_index(self, scan_msg, target_angle_deg):
        """Find the LiDAR array index closest to target angle"""
        
        target_angle_rad = math.radians(target_angle_deg)
        
        # Handle angle wrapping (LiDAR angles are typically -π to π)
        if target_angle_rad > math.pi:
            target_angle_rad -= 2 * math.pi
        
        closest_index = None
        smallest_diff = float('inf')
        
        for i in range(len(scan_msg.ranges)):
            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_diff = abs(angle_rad - target_angle_rad)
            
            # Handle wrap-around (e.g., comparing -179° to +179°)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff < smallest_diff:
                smallest_diff = angle_diff
                closest_index = i
        
        return closest_index
    
    def calculate_and_display_result(self):
        """Calculate average distance and display calibration result"""
        
        # Calculate average distance
        avg_distance = sum(self.readings_collected) / len(self.readings_collected)
        
        # Calculate standard deviation for accuracy assessment
        variance = sum((x - avg_distance) ** 2 for x in self.readings_collected) / len(self.readings_collected)
        std_dev = math.sqrt(variance)
        
        # Determine boundary value (left and back edges are negative)
        if self.target_angle == 180:  # Left edge
            boundary_value = -avg_distance
        elif self.target_angle == 270:  # Back edge
            boundary_value = -avg_distance
        else:  # Right and front edges
            boundary_value = avg_distance
        
        # Display results
        print(f"\n" + "="*60)
        print(f"🎯 CALIBRATION RESULT FOR {self.target_angle}°")
        print(f"="*60)
        print(f"Direction:        {self.get_direction_name(self.target_angle)}")
        print(f"Robot edge:       {self.get_edge_name(self.target_angle)}")
        print(f"Average distance: {avg_distance:.3f}m")
        print(f"Standard dev:     {std_dev:.3f}m")
        print(f"Boundary name:    {self.get_boundary_name(self.target_angle)}")
        print(f"Boundary value:   {boundary_value:.3f}")
        print(f"="*60)
        
        # Show code to update safety_supervisor.py
        print(f"\n📝 UPDATE YOUR SAFETY_SUPERVISOR.PY:")
        print(f"   '{self.get_boundary_name(self.target_angle)}': {boundary_value:.3f},")
        
        # Show measurement quality
        if std_dev < 0.005:
            print(f"✅ EXCELLENT: Very stable readings (±{std_dev*1000:.1f}mm)")
        elif std_dev < 0.010:
            print(f"✅ GOOD: Stable readings (±{std_dev*1000:.1f}mm)")
        else:
            print(f"⚠️  WARNING: Unstable readings (±{std_dev*1000:.1f}mm)")
            print(f"   Try repositioning robot for better wall contact")
        
        print(f"\n🔄 NEXT STEPS:")
        next_angles = [0, 90, 180, 270]
        next_angles.remove(self.target_angle)
        if next_angles:
            next_angle = next_angles[0]
            print(f"   Run: python3 calibrate_boundaries.py --angle {next_angle}")
            print(f"   (Place {self.get_edge_name(next_angle)} edge against wall)")
        else:
            print(f"   All calibrations complete!")
        
        print(f"\n")

def main():
    parser = argparse.ArgumentParser(description='Calibrate robot boundaries using LiDAR')
    parser.add_argument('--angle', type=int, required=True, 
                       choices=[0, 90, 180, 270],
                       help='Angle to calibrate (0=right, 90=front, 180=left, 270=back)')
    
    args = parser.parse_args()
    
    try:
        calibrator = BoundaryCalibrator(args.angle)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print(f"\n👋 Calibration stopped by user")
    except KeyboardInterrupt:
        print(f"\n👋 Calibration stopped by user")

if __name__ == '__main__':
    main()
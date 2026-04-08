#!/usr/bin/env python3
"""
LiDAR Angular Filter Node - Tourminator Phase 2
Filters LiDAR scan data to only include readings between 90° and 270°
Author: Tourminator Team
"""

import rospy
import math
from sensor_msgs.msg import LaserScan

class LidarFilter:
    def __init__(self):
        rospy.init_node('lidar_filter', anonymous=False)
        
        # Parameters
        self.min_angle_deg = rospy.get_param('~min_angle_deg', 90.0)   # Minimum angle in degrees
        self.max_angle_deg = rospy.get_param('~max_angle_deg', 270.0)  # Maximum angle in degrees
        
        # Convert to radians
        self.min_angle_rad = math.radians(self.min_angle_deg)
        self.max_angle_rad = math.radians(self.max_angle_deg)
        
        # ROS interfaces
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.filtered_scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)
        
        rospy.loginfo(f"🔧 LiDAR Filter initialized: {self.min_angle_deg}° to {self.max_angle_deg}°")
        rospy.loginfo(f"📡 Subscribing to: /scan")
        rospy.loginfo(f"📡 Publishing to: /scan_filtered")

    def scan_callback(self, scan_msg):
        """
        Filter incoming LiDAR scan to only include specified angular range
        """
        # Create filtered scan message
        filtered_scan = LaserScan()
        
        # Copy header and timing info
        filtered_scan.header = scan_msg.header
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        
        # Calculate which indices to keep
        original_ranges = scan_msg.ranges
        original_intensities = scan_msg.intensities if scan_msg.intensities else []
        
        # Find start and end indices for our desired range
        start_index = None
        end_index = None
        
        for i, range_val in enumerate(original_ranges):
            # Calculate angle for this reading
            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Normalize angle to 0-2π
            angle_rad = angle_rad % (2 * math.pi)
            
            # Check if this angle is in our desired range
            if self.is_angle_in_range(angle_rad):
                if start_index is None:
                    start_index = i
                end_index = i
        
        # If we found valid indices, extract the data
        if start_index is not None and end_index is not None:
            # Extract ranges for our angular window
            filtered_ranges = original_ranges[start_index:end_index + 1]
            filtered_intensities = original_intensities[start_index:end_index + 1] if original_intensities else []
            
            # Update scan message with filtered data
            filtered_scan.angle_min = scan_msg.angle_min + start_index * scan_msg.angle_increment
            filtered_scan.angle_max = scan_msg.angle_min + end_index * scan_msg.angle_increment
            filtered_scan.ranges = filtered_ranges
            filtered_scan.intensities = filtered_intensities
            
            # Publish filtered scan
            self.filtered_scan_pub.publish(filtered_scan)
            
            # Debug info (throttled)
            if rospy.get_time() % 5 < 0.1:  # Every ~5 seconds
                rospy.loginfo(f"🔍 Filtered scan: {len(filtered_ranges)}/{len(original_ranges)} points "
                             f"(angles: {math.degrees(filtered_scan.angle_min):.1f}° to {math.degrees(filtered_scan.angle_max):.1f}°)")
        else:
            rospy.logwarn_throttle(1.0, "⚠️  No valid LiDAR points found in specified angular range!")

    def is_angle_in_range(self, angle_rad):
        """
        Check if angle is within our desired range (90° to 270°)
        Handles wrap-around for angles
        """
        # Convert angle to 0-2π range
        angle_rad = angle_rad % (2 * math.pi)
        
        # Convert our min/max to 0-2π range
        min_angle = self.min_angle_rad % (2 * math.pi)
        max_angle = self.max_angle_rad % (2 * math.pi)
        
        # Handle wrap-around case
        if min_angle <= max_angle:
            # Normal case: min_angle <= angle <= max_angle
            return min_angle <= angle_rad <= max_angle
        else:
            # Wrap-around case: angle >= min_angle OR angle <= max_angle
            return angle_rad >= min_angle or angle_rad <= max_angle

    def run(self):
        """
        Main execution loop
        """
        rospy.loginfo("🚀 LiDAR Filter node running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        filter_node = LidarFilter()
        filter_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 LiDAR Filter node shutdown")
    except Exception as e:
        rospy.logerr(f"❌ LiDAR Filter error: {e}")
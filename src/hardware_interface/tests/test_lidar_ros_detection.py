#!/usr/bin/env python3
"""
ROS LiDAR Object Detection with Exact Positions
Runs 10 scans and reports exact positions of objects
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from collections import defaultdict
import numpy as np

class LiDARObjectDetector:
    def __init__(self):
        rospy.init_node('lidar_object_detector', anonymous=True)
        
        # Detection parameters
        self.min_distance = 0.10  # 10cm minimum
        self.max_distance = 0.20  # 20cm maximum
        self.merge_threshold = 0.10  # Merge points within 10cm of each other
        
        # Scan control
        self.scan_count = 0
        self.max_scans = 10
        self.all_detections = []
        
        # Subscribe to scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        print("🔄 LiDAR Object Detection Started")
        print(f"🔍 Scanning {self.max_distance}m range for {self.max_scans} scans...")
        print("Objects will be reported after completion\n")

    def scan_callback(self, scan_msg):
        """Process incoming scan data"""
        if self.scan_count >= self.max_scans:
            return
            
        self.scan_count += 1
        
        # Collect all valid detections in this scan
        current_scan = []
        for i, distance in enumerate(scan_msg.ranges):
            if not (math.isnan(distance) or math.isinf(distance)):
                if self.min_distance <= distance <= self.max_distance:
                    angle = math.degrees(scan_msg.angle_min + i * scan_msg.angle_increment)
                    angle = angle % 360.0
                    current_scan.append((angle, distance))
        
        self.all_detections.extend(current_scan)
        
        # Show progress
        print(f"📊 Scan {self.scan_count}/{self.max_scans} - Found {len(current_scan)} points")
        
        # Process and exit after final scan
        if self.scan_count == self.max_scans:
            self.process_and_report()
            rospy.signal_shutdown("Scan complete")

    def process_and_report(self):
        """Cluster detections and report objects"""
        if not self.all_detections:
            print("\n❌ No objects detected in range")
            return
        
        # Convert to numpy array for processing
        points = np.array(self.all_detections)
        angles = points[:, 0]
        distances = points[:, 1]
        
        # Convert to cartesian coordinates for clustering
        x = distances * np.cos(np.radians(angles))
        y = distances * np.sin(np.radians(angles))
        
        # Simple clustering - merge nearby points
        clusters = []
        for xi, yi, angle, dist in zip(x, y, angles, distances):
            merged = False
            for cluster in clusters:
                # Calculate distance to cluster center
                cluster_x, cluster_y, cluster_points = cluster
                dx = xi - cluster_x
                dy = yi - cluster_y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < self.merge_threshold:
                    # Merge with existing cluster
                    new_x = (cluster_x * len(cluster_points) + xi) / (len(cluster_points) + 1)
                    new_y = (cluster_y * len(cluster_points) + yi) / (len(cluster_points) + 1)
                    cluster[0] = new_x
                    cluster[1] = new_y
                    cluster[2].append((angle, dist))
                    merged = True
                    break
            
            if not merged:
                # Create new cluster
                clusters.append([xi, yi, [(angle, dist)]])
        
        # Convert clusters back to polar coordinates
        objects = []
        for cluster in clusters:
            x, y, points = cluster
            avg_distance = np.mean([p[1] for p in points])
            avg_angle = np.mean([p[0] for p in points])
            
            # Calculate cluster size
            min_dist = min([p[1] for p in points])
            max_dist = max([p[1] for p in points])
            size = max_dist - min_dist
            
            objects.append({
                'angle': avg_angle,
                'distance': avg_distance,
                'points': len(points),
                'size': size
            })
        
        # Sort objects by angle
        objects.sort(key=lambda obj: obj['angle'])
        
        # Print final report
        print("\n🎯 OBJECT DETECTION REPORT")
        print("=" * 50)
        print(f"Total scans: {self.scan_count}")
        print(f"Total points detected: {len(self.all_detections)}")
        print(f"Unique objects found: {len(objects)}")
        print("=" * 50)
        
        for i, obj in enumerate(objects, 1):
            print(f"Object {i}:")
            print(f"  ↳ Angle: {obj['angle']:.1f}°")
            print(f"  ↳ Distance: {obj['distance']:.3f}m")
            print(f"  ↳ Points: {obj['points']}")
            print(f"  ↳ Size: {obj['size']:.3f}m")
            print("-" * 20)
        
        # Visual representation
        print("\n🌍 Top View (0° is front, angles increase clockwise):")
        for obj in objects:
            angle = obj['angle']
            dist = obj['distance']
            marker = '•' if obj['points'] > 3 else '◦'
            print(f"{angle:5.1f}° | {' ' * int(dist*50)}{marker} ({dist:.2f}m)")

def main():
    try:
        detector = LiDARObjectDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print("\n🛑 Detection interrupted")

if __name__ == '__main__':
    main()
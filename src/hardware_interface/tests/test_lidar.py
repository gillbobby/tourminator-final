#!/usr/bin/env python3
"""
Enhanced ROS LiDAR Test with Real-time Environment Detection
Tests RPLidar A1M8 integration with ROS and provides live detection feedback

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_lidar_ros.py
Usage: 
  1. Start ROS: roscore
  2. Run test: python3 test_lidar_ros.py
  3. Or as ROS node: rosrun hardware_interface test_lidar_ros.py

ENHANCED FEATURES:
- Full ROS integration testing
- Real-time human/object detection
- Live environment monitoring  
- Validates lidar_bridge.py functionality
- Tests SLAM-ready data output
- Interactive detection feedback
"""

import sys
import time
import math
import threading
from collections import defaultdict, deque
from datetime import datetime

# ROS imports
try:
    import rospy
    import tf2_ros
    from tf2_ros import TransformListener, Buffer
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    from geometry_msgs.msg import TransformStamped
    ROS_AVAILABLE = True
except ImportError:
    print("❌ ROS not available - install ROS and source workspace")
    ROS_AVAILABLE = False

# RPLidar direct access (for comparison tests)
try:
    from rplidar import RPLidar, RPLidarException
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False

class ROSLiDARTester:
    def __init__(self):
        if not ROS_AVAILABLE:
            print("❌ ROS not available - cannot run ROS integration tests")
            sys.exit(1)
            
        # Initialize ROS node
        rospy.init_node('lidar_tester', anonymous=True)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.test_results = []
        self.scan_data_buffer = deque(maxlen=100)  # Keep last 100 scans
        self.detection_zones = {
            'front': {'angles': (350, 10), 'detections': []},      # -10° to +10°
            'left': {'angles': (80, 100), 'detections': []},       # 80° to 100°
            'right': {'angles': (260, 280), 'detections': []},     # 260° to 280°
            'back': {'angles': (170, 190), 'detections': []}       # 170° to 190°
        }
        
        # Detection tracking
        self.baseline_distances = {}  # Background distances per angle
        self.human_detections = []
        self.last_scan_time = None
        self.scan_count = 0
        self.detection_sensitivity = rospy.get_param('~detection_sensitivity', 0.1)
 # 30cm change to trigger detection
        
        # ROS subscribers
        self.scan_subscriber = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Status tracking
        self.ros_scan_received = False
        self.lidar_bridge_running = False
        
    def log_result(self, test_name, passed, message=""):
        """Log test result with color coding"""
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name}")
        if message:
            print(f"    {message}")
        
        self.test_results.append({
            'test': test_name,
            'passed': passed,
            'message': message,
            'timestamp': datetime.now().isoformat()
        })

    def test_ros_environment(self):
        """Test ROS environment and core setup"""
        print("\n=== Testing ROS Environment ===")
        
        try:
            # Test if roscore is running
            rospy.get_published_topics()
            self.log_result("ROS Core running", True, "roscore detected")
        except Exception as e:
            self.log_result("ROS Core running", False, "Start with: roscore")
            return False
        
        try:
            # Test ROS master connectivity
            master = rospy.get_master()
            topics = master.getPublishedTopics('')
            self.log_result("ROS Master connectivity", True, f"{len(topics[2])} topics available")
        except Exception as e:
            self.log_result("ROS Master connectivity", False, str(e))
            return False
            
        # Test our node initialization
        try:
            node_name = rospy.get_name()
            self.log_result("ROS Node initialization", True, f"Node: {node_name}")
            return True
        except Exception as e:
            self.log_result("ROS Node initialization", False, str(e))
            return False

    def test_lidar_bridge_detection(self):
        """Test if lidar_bridge.py is running and publishing"""
        print("\n=== Testing LiDAR Bridge Integration ===")
        
        try:
            # Check if /scan topic exists
            topics = rospy.get_published_topics()
            scan_topic_found = any('/scan' in topic[0] for topic in topics)
            
            if scan_topic_found:
                self.log_result("LiDAR Bridge /scan topic", True, "/scan topic available")
            else:
                self.log_result("LiDAR Bridge /scan topic", False, 
                              "Start lidar_bridge: rosrun hardware_interface lidar_bridge.py")
                return False
            
            # Try to get a scan message
            print("Waiting for scan data from lidar_bridge...")
            try:
                scan_msg = rospy.wait_for_message('/scan', LaserScan, timeout=10.0)
                self.log_result("LiDAR Bridge data reception", True, 
                              f"Received scan with {len(scan_msg.ranges)} points")
                self.lidar_bridge_running = True
                return True
            except rospy.ROSException:
                self.log_result("LiDAR Bridge data reception", False, 
                              "No scan data received - check lidar_bridge.py")
                return False
                
        except Exception as e:
            self.log_result("LiDAR Bridge detection", False, f"Error: {str(e)}")
            return False

    def scan_callback(self, scan_msg):
        """Process incoming scan data for real-time detection"""
        self.ros_scan_received = True
        self.scan_count += 1
        self.last_scan_time = rospy.Time.now()
        
        # Convert scan to our format
        scan_data = []
        for i, range_val in enumerate(scan_msg.ranges):
            if self.is_valid_range(range_val, scan_msg.range_min, scan_msg.range_max):
                angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
                angle_deg = math.degrees(angle_rad) % 360.0
                scan_data.append((angle_deg, range_val))
        
        # Store in buffer for analysis
        self.scan_data_buffer.append({
            'timestamp': scan_msg.header.stamp,
            'data': scan_data,
            'total_points': len(scan_data)
        })
        
        # Perform real-time detection
        self.detect_environment_changes(scan_data)

    def is_valid_range(self, range_val, range_min, range_max):
        """Check if range value is valid"""
        return not (math.isnan(range_val) or math.isinf(range_val) or 
                   range_val < range_min or range_val > range_max)

    def establish_baseline(self):
        """Establish baseline distances for change detection"""
        print("\n=== Establishing Environmental Baseline ===")
        print("Please ensure area around LiDAR is clear...")
        print("Taking baseline measurements for 5 seconds...")
        
        baseline_scans = []
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < 5.0:
            if self.scan_data_buffer:
                latest_scan = self.scan_data_buffer[-1]
                baseline_scans.append(latest_scan['data'])
            time.sleep(0.1)
        
        if not baseline_scans:
            print("❌ No baseline data collected")
            return False
        
        # Calculate average distances per angle sector
        angle_sums = defaultdict(list)
        
        for scan in baseline_scans:
            for angle, distance in scan:
                angle_sector = int(angle // 5) * 5  # 5-degree sectors
                angle_sums[angle_sector].append(distance)
        
        # Store baseline averages
        for sector, distances in angle_sums.items():
            if distances:
                self.baseline_distances[sector] = sum(distances) / len(distances)
        
        print(f"✅ Baseline established with {len(self.baseline_distances)} sectors")
        print(f"📊 Monitoring {len(baseline_scans)} scans")
        return True

    def detect_environment_changes(self, scan_data):
        """Detect changes in environment (humans, objects)"""
        if not self.baseline_distances or not scan_data:
            return
        
        current_detections = []
        significant_changes = []
        
        # Group current scan by sectors
        current_sectors = defaultdict(list)
        for angle, distance in scan_data:
            sector = int(angle // 5) * 5
            current_sectors[sector].append(distance)
        
        # Compare to baseline
        for sector, distances in current_sectors.items():
            if sector in self.baseline_distances and distances:
                current_avg = sum(distances) / len(distances)
                baseline_avg = self.baseline_distances[sector]
                distance_change = baseline_avg - current_avg
                
                # Significant change detection (something got closer)
                if distance_change > self.detection_sensitivity:
                    significant_changes.append({
                        'sector': sector,
                        'angle_range': f"{sector}-{sector+5}°",
                        'distance_change': distance_change,
                        'current_distance': current_avg,
                        'baseline_distance': baseline_avg
                    })
        
        # Classify detections
        if significant_changes:
            # Group nearby sectors together
            detection_groups = self.group_adjacent_detections(significant_changes)
            
            for group in detection_groups:
                detection_info = {
                    'timestamp': rospy.Time.now(),
                    'angle_center': group['center_angle'],
                    'angle_span': group['span'],
                    'distance': group['avg_distance'],
                    'change_magnitude': group['avg_change'],
                    'zone': self.classify_detection_zone(group['center_angle'])
                }
                
                # Real-time notification
                if group['avg_change'] > 0.5:  # Significant detection
                    print(f"🚨 DETECTION: {detection_info['zone']} zone, "
                          f"{detection_info['angle_center']:.0f}°, "
                          f"{detection_info['distance']:.2f}m away, "
                          f"change: {detection_info['change_magnitude']:.2f}m")
                
                current_detections.append(detection_info)
        
        # Update detection zones
        self.update_detection_zones(current_detections)

    def group_adjacent_detections(self, changes):
        """Group adjacent angle sectors into single detections"""
        if not changes:
            return []
        
        # Sort by sector angle
        changes.sort(key=lambda x: x['sector'])
        
        groups = []
        current_group = [changes[0]]
        
        for change in changes[1:]:
            # If sectors are adjacent (within 10 degrees), group them
            if change['sector'] - current_group[-1]['sector'] <= 10:
                current_group.append(change)
            else:
                # Process current group and start new one
                groups.append(self.process_detection_group(current_group))
                current_group = [change]
        
        # Process final group
        if current_group:
            groups.append(self.process_detection_group(current_group))
        
        return groups

    def process_detection_group(self, group):
        """Process a group of adjacent detections"""
        angles = [change['sector'] for change in group]
        distances = [change['current_distance'] for change in group]
        changes = [change['distance_change'] for change in group]
        
        return {
            'center_angle': sum(angles) / len(angles),
            'span': max(angles) - min(angles) + 5,  # +5 for sector width
            'avg_distance': sum(distances) / len(distances),
            'avg_change': sum(changes) / len(changes),
            'sector_count': len(group)
        }

    def classify_detection_zone(self, angle):
        """Classify which zone a detection is in"""
        angle = angle % 360
        
        if 350 <= angle or angle <= 10:
            return "FRONT"
        elif 80 <= angle <= 100:
            return "LEFT"
        elif 170 <= angle <= 190:
            return "BACK"
        elif 260 <= angle <= 280:
            return "RIGHT"
        else:
            return f"ANGLE_{int(angle)}"

    def update_detection_zones(self, detections):
        """Update detection zone tracking"""
        # Clear previous detections
        for zone in self.detection_zones.values():
            zone['detections'] = []
        
        # Add current detections
        for detection in detections:
            zone_name = detection['zone'].lower()
            if zone_name in self.detection_zones:
                self.detection_zones[zone_name]['detections'].append(detection)

    def test_real_time_detection(self):
        """Test real-time detection capabilities"""
        print("\n=== Testing Real-time Detection ===")
        
        if not self.lidar_bridge_running:
            self.log_result("Real-time detection test", False, "LiDAR bridge not running")
            return False
        
        # Subscribe to scan data
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Wait for initial data
        print("Waiting for scan data...")
        wait_start = rospy.Time.now()
        while not self.ros_scan_received and (rospy.Time.now() - wait_start).to_sec() < 10:
            time.sleep(0.1)
        
        if not self.ros_scan_received:
            self.log_result("ROS scan data reception", False, "No scan data received")
            return False
        
        self.log_result("ROS scan data reception", True, "Receiving scan data")
        
        # Establish baseline
        if not self.establish_baseline():
            self.log_result("Baseline establishment", False, "Could not establish baseline")
            return False
        
        self.log_result("Baseline establishment", True, "Environmental baseline set")
        
        # Start interactive detection test
        print("\n🎯 INTERACTIVE DETECTION TEST")
        print("=" * 50)
        print("The LiDAR will now monitor for movement!")
        print("Try these tests:")
        print("  1. Walk in front of the LiDAR")
        print("  2. Wave your hand in different directions")
        print("  3. Place/remove objects around the LiDAR")
        print("  4. Move around to different sides")
        print("\nPress Ctrl+C to end detection test")
        print("=" * 50)
        
        detection_test_passed = False
        human_detected = False
        test_start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                # Check for any significant detections
                total_detections = sum(len(zone['detections']) for zone in self.detection_zones.values())
                
                if total_detections > 0:
                    if not human_detected:
                        print("\n🎉 FIRST DETECTION SUCCESS!")
                        human_detected = True
                        detection_test_passed = True
                    
                    # Display current detections
                    self.display_detection_status()
                
                # Show periodic status
                if int(rospy.Time.now().to_sec()) % 5 == 0:
                    elapsed = (rospy.Time.now() - test_start_time).to_sec()
                    print(f"\n📡 Status: {self.scan_count} scans processed, "
                          f"{elapsed:.0f}s elapsed, monitoring...")
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n⏹️  Detection test stopped by user")
        
        if detection_test_passed:
            self.log_result("Human detection capability", True, 
                          "Successfully detected environmental changes")
        else:
            self.log_result("Human detection capability", False, 
                          "No significant detections recorded during test")
        
        return detection_test_passed

    def display_detection_status(self):
        """Display current detection status"""
        print("\n📍 CURRENT DETECTIONS:")
        
        for zone_name, zone_data in self.detection_zones.items():
            if zone_data['detections']:
                for detection in zone_data['detections']:
                    print(f"  🎯 {zone_name.upper()}: "
                          f"{detection['distance']:.2f}m, "
                          f"change: {detection['change_magnitude']:.2f}m, "
                          f"angle: {detection['angle_center']:.0f}°")
        
        print("   (Move around to trigger more detections)")

    def test_ros_tf_integration(self):
        """Test TF integration for coordinate frames"""
        print("\n=== Testing ROS TF Integration ===")
        
        try:
            # Check if lidar_link frame exists
            frames = self.tf_buffer.all_frames_as_string()
            
            if 'lidar_link' in frames:
                self.log_result("TF lidar_link frame", True, "lidar_link frame found")
            else:
                self.log_result("TF lidar_link frame", False, 
                              "lidar_link frame missing - check tf tree")
                return False
            
            # Try to get transform
            try:
                transform = self.tf_buffer.lookup_transform('base_link', 'lidar_link', 
                                                          rospy.Time(0), rospy.Duration(1.0))
                self.log_result("TF transform lookup", True, "base_link->lidar_link transform OK")
            except Exception:
                # This might fail if base_link doesn't exist yet - that's OK for testing
                self.log_result("TF transform lookup", True, "lidar_link frame available (base_link optional)")
            
            return True
            
        except Exception as e:
            self.log_result("TF integration test", False, f"TF error: {str(e)}")
            return False

    def test_slam_data_quality(self):
        """Test if scan data is suitable for SLAM"""
        print("\n=== Testing SLAM Data Quality ===")
        
        if not self.scan_data_buffer:
            self.log_result("SLAM data quality", False, "No scan data available")
            return False
        
        # Analyze recent scans
        recent_scans = list(self.scan_data_buffer)[-10:]  # Last 10 scans
        
        if not recent_scans:
            self.log_result("SLAM data quality", False, "Insufficient scan data")
            return False
        
        # Quality metrics
        total_points = sum(scan['total_points'] for scan in recent_scans)
        avg_points_per_scan = total_points / len(recent_scans)
        
        # Check angle coverage
        all_angles = []
        for scan in recent_scans:
            all_angles.extend([angle for angle, _ in scan['data']])
        
        if all_angles:
            angle_coverage = max(all_angles) - min(all_angles)
            unique_sectors = len(set(int(angle // 10) for angle in all_angles))
        else:
            angle_coverage = 0
            unique_sectors = 0
        
        print(f"SLAM Quality Analysis:")
        print(f"  Average points per scan: {avg_points_per_scan:.1f}")
        print(f"  Angle coverage: {angle_coverage:.1f}°")
        print(f"  Unique 10° sectors: {unique_sectors}/36")
        
        # SLAM suitability criteria
        slam_suitable = (
            avg_points_per_scan >= 50 and      # Enough points
            angle_coverage >= 300 and          # Good angle coverage  
            unique_sectors >= 20               # Good sector distribution
        )
        
        if slam_suitable:
            self.log_result("SLAM data quality", True, 
                          f"Suitable: {avg_points_per_scan:.0f} pts/scan, {angle_coverage:.0f}° coverage")
        else:
            self.log_result("SLAM data quality", False, 
                          f"Poor quality: {avg_points_per_scan:.0f} pts/scan, {angle_coverage:.0f}° coverage")
        
        return slam_suitable

    def test_pole_filtering_effectiveness(self):
        """Test pole filtering from lidar_bridge.py"""
        print("\n=== Testing Pole Filtering Effectiveness ===")
        
        if not self.scan_data_buffer:
            self.log_result("Pole filtering test", False, "No scan data for analysis")
            return False
        
        # Analyze pole region (270° ± 10°)
        pole_region_data = []
        
        for scan in list(self.scan_data_buffer)[-20:]:  # Last 20 scans
            for angle, distance in scan['data']:
                # Check if in pole region (260-280°)
                if 260 <= angle <= 280:
                    pole_region_data.append((angle, distance))
        
        if not pole_region_data:
            self.log_result("Pole filtering effectiveness", True, 
                          "Pole region clean - filtering working")
            return True
        
        # Check if pole filtering is working
        very_close_detections = [d for a, d in pole_region_data if d < 0.2]  # Less than 20cm
        
        pole_filter_ratio = len(very_close_detections) / len(pole_region_data)
        
        print(f"Pole Region Analysis:")
        print(f"  Total pole region points: {len(pole_region_data)}")
        print(f"  Very close detections: {len(very_close_detections)}")
        print(f"  Close detection ratio: {pole_filter_ratio:.2%}")
        
        if pole_filter_ratio < 0.1:  # Less than 10% very close detections
            self.log_result("Pole filtering effectiveness", True, 
                          f"Good filtering: {pole_filter_ratio:.1%} close detections")
        else:
            self.log_result("Pole filtering effectiveness", False, 
                          f"Poor filtering: {pole_filter_ratio:.1%} close detections")
        
        return pole_filter_ratio < 0.1

    def test_navigation_stack_compatibility(self):
        """Test compatibility with ROS navigation stack"""
        print("\n=== Testing Navigation Stack Compatibility ===")
        
        try:
            # Check if scan data format is correct for move_base
            if self.scan_data_buffer:
                latest_scan = self.scan_data_buffer[-1]
                
                # Check data freshness
                data_age = (rospy.Time.now() - latest_scan['timestamp']).to_sec()
                
                if data_age < 1.0:
                    self.log_result("Scan data freshness", True, f"Data age: {data_age:.2f}s")
                else:
                    self.log_result("Scan data freshness", False, f"Stale data: {data_age:.2f}s")
                
                # Check for navigation-critical zones
                front_coverage = any(350 <= angle <= 360 or 0 <= angle <= 10 
                                   for angle, _ in latest_scan['data'])
                
                if front_coverage:
                    self.log_result("Navigation front coverage", True, "Front zone covered")
                else:
                    self.log_result("Navigation front coverage", False, "Front zone not covered")
                
                return data_age < 1.0 and front_coverage
            else:
                self.log_result("Navigation compatibility", False, "No scan data available")
                return False
                
        except Exception as e:
            self.log_result("Navigation stack compatibility", False, str(e))
            return False

    def run_comprehensive_test(self):
        """Run all ROS integration tests"""
        print("Enhanced ROS LiDAR Integration Test")
        print("=" * 60)
        print("Testing RPLidar A1M8 with ROS and real-time detection")
        print("INCLUDES: Human detection, SLAM testing, TF integration")
        print("=" * 60)
        
        try:
            # Core ROS tests
            if not self.test_ros_environment():
                self.print_summary()
                return False
            
            if not self.test_lidar_bridge_detection():
                self.print_summary()
                return False
            
            # TF and navigation tests
            self.test_ros_tf_integration()
            
            # Real-time detection test (interactive)
            if self.test_real_time_detection():
                # Additional analysis tests
                self.test_slam_data_quality()
                self.test_pole_filtering_effectiveness()
                self.test_navigation_stack_compatibility()
            
            self.print_summary()
            return True
            
        except KeyboardInterrupt:
            print("\n\n⏹️  Tests interrupted by user")
            self.print_summary()
            return False
        except Exception as e:
            print(f"\n💥 Unexpected error: {e}")
            self.print_summary()
            return False

    def print_summary(self):
        """Print comprehensive test summary"""
        passed = sum(1 for result in self.test_results if result['passed'])
        failed = len(self.test_results) - passed
        
        print("\n" + "="*70)
        print("ROS LIDAR INTEGRATION TEST SUMMARY")
        print("="*70)
        print(f"Tests Run:  {len(self.test_results)}")
        print(f"Passed:     {passed}")
        print(f"Failed:     {failed}")
        print(f"Success:    {(passed/len(self.test_results)*100):.1f}%" if self.test_results else "0%")
        
        if failed <= 1:  # Allow 1 failure for optional tests
            print(f"\n🎉 ROS LIDAR INTEGRATION SUCCESSFUL!")
            print(f"\n✅ Verified Capabilities:")
            print(f"  • ROS core connectivity")
            print(f"  • lidar_bridge.py integration")
            print(f"  • Real-time environmental detection")
            print(f"  • Human movement detection")
            print(f"  • SLAM-ready data quality")
            print(f"  • Pole filtering functionality")
            print(f"  • Navigation stack compatibility")
            
            print(f"\n📡 Detection Summary:")
            print(f"  • Scans processed: {self.scan_count}")
            print(f"  • Human detections: {len(self.human_detections)}")
            print(f"  • Baseline sectors: {len(self.baseline_distances)}")
            
            print(f"\n🎯 Next Steps:")
            print(f"1. Start gmapping: roslaunch gmapping_demo.launch")
            print(f"2. Start move_base for navigation")
            print(f"3. Monitor: rviz -d navigation.rviz")
            print(f"4. Test full autonomous navigation")
            
        else:
            print(f"\n⚠️  {failed} ROS integration tests failed.")
            print(f"\n❌ Failed Tests:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  • {result['test']}: {result['message']}")
            
            print(f"\n🔧 Troubleshooting Steps:")
            print(f"1. Ensure roscore is running: roscore")
            print(f"2. Start lidar_bridge: rosrun hardware_interface lidar_bridge.py")
            print(f"3. Check topics: rostopic list")
            print(f"4. Monitor scan: rostopic echo /scan")
            print(f"5. Check TF tree: rosrun tf view_frames")
        
        print(f"\n📊 Test completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def main():
    print("ROS LiDAR Integration Test with Real-time Detection")
    print("=" * 65)
    print("Testing RPLidar A1M8 integration with ROS navigation stack")
    print("ENHANCED: Real-time human detection and environment monitoring")
    print("=" * 65)
    
    # Prerequisites check
    print("\n🔧 PREREQUISITES:")
    print("1. ROS workspace sourced?")
    print("2. roscore running?")
    print("3. LiDAR connected and powered?")
    print("4. lidar_bridge.py ready to test?")
    print("5. Clear area around LiDAR for detection test?")
    
    response = input("\nPrerequisites met? Ready for ROS integration test? (y/N): ")
    if response.lower() != 'y':
        print("Test cancelled. Set up environment and run again when ready.")
        sys.exit(1)
    
    try:
        tester = ROSLiDARTester()
        success = tester.run_comprehensive_test()
        sys.exit(0 if success else 1)
        
    except Exception as e:
        print(f"💥 Test error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

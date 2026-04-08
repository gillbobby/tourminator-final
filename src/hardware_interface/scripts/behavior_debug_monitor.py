#!/usr/bin/env python3
"""
Tourminator Behavioral Debug Monitor
Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/behavior_debug_monitor.py

SPECIALIZED DEBUG TOOL FOR AUTONOMOUS NAVIGATION BEHAVIOR
- Shows decision-making process
- Environmental perception details
- Planning and exploration status
- Photo capture progress
- Configuration tracking
- Navigation state analysis

This replaces messy terminal output with structured behavioral insights.
"""

import rospy
import json
import os
import time
import math
from collections import defaultdict, deque
from datetime import datetime
import threading

# ROS message imports
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus

class BehavioralDebugMonitor:
    def __init__(self):
        rospy.init_node('behavior_debug_monitor', anonymous=True)
        
        # Behavioral state tracking
        self.robot_state = "UNKNOWN"
        self.previous_state = "UNKNOWN"
        self.state_duration = 0
        self.state_history = deque(maxlen=10)
        
        # Decision making tracking
        self.current_goal = None
        self.goal_status = "NO_GOAL"
        self.goal_history = deque(maxlen=5)
        self.decision_log = deque(maxlen=20)
        
        # Environmental perception
        self.obstacles_detected = []
        self.nearest_obstacle = float('inf')
        self.lidar_data = {
            'ranges_count': 0,
            'front_clear': True,
            'left_clear': True,
            'right_clear': True,
            'rear_clear': True
        }
        
        # Exploration tracking
        self.exploration_status = {
            'coverage_percent': 0.0,
            'frontiers_found': 0,
            'areas_mapped': 0,
            'exploration_complete': False,
            'stuck_count': 0,
            'recovery_attempts': 0
        }
        
        # Photo mission tracking
        self.photo_status = {
            'photos_taken': 0,
            'waypoints_total': 0,
            'current_waypoint': 0,
            'photo_mode_active': False,
            'last_photo_location': None
        }
        
        # Navigation planning
        self.navigation_status = {
            'planning_active': False,
            'path_found': False,
            'path_length': 0,
            'current_velocity': {'linear': 0.0, 'angular': 0.0},
            'stuck_detected': False,
            'recovery_behavior': False
        }
        
        # System configuration tracking
        self.config_status = {
            'config_files_loaded': [],
            'parameters_active': {},
            'safety_settings': {},
            'exploration_settings': {}
        }
        
        # Timing and performance
        self.start_time = time.time()
        self.last_update = {}
        self.update_frequency = 1.0  # Hz
        
        # Setup subscribers for behavioral monitoring
        self.setup_behavioral_subscribers()
        
        # Load configuration information
        self.load_config_info()
        
        # Start the display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        rospy.loginfo("🧠 Behavioral Debug Monitor Started - Monitoring robot decision-making")
        
    def setup_behavioral_subscribers(self):
        """Setup subscribers for behavioral analysis"""
        
        # Core behavioral state
        rospy.Subscriber('/mapper_state', String, self.mapper_state_callback)
        rospy.Subscriber('/mapper_debug', String, self.mapper_debug_callback)
        
        # Navigation and planning
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        rospy.Subscriber('/move_base/status', GoalStatus, self.status_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/cmd_vel_nav', Twist, self.cmd_vel_nav_callback)
        
        # Environmental perception
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Safety and status
        rospy.Subscriber('/safety_status', String, self.safety_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        rospy.Subscriber('/camera/photo_status', String, self.photo_callback)
        rospy.Subscriber('/map_coverage', Float32, self.coverage_callback)
        
    def load_config_info(self):
        """Load information about configuration files being used"""
        try:
            # Check for parameter files
            config_files = [
                '/opt/ros/noetic/share/indoor_mapping_robot/config/robot_params.yaml',
                '/opt/ros/noetic/share/indoor_mapping_robot/config/costmap_common_params.yaml',
                '/opt/ros/noetic/share/indoor_mapping_robot/config/local_costmap_params.yaml',
                '/opt/ros/noetic/share/indoor_mapping_robot/config/global_costmap_params.yaml',
                '/opt/ros/noetic/share/indoor_mapping_robot/config/base_local_planner_params.yaml'
            ]
            
            for config_file in config_files:
                if os.path.exists(config_file):
                    self.config_status['config_files_loaded'].append(os.path.basename(config_file))
                    
            # Get key parameters
            self.config_status['parameters_active'] = {
                'exploration_timeout': rospy.get_param('~exploration_timeout', 'NOT_SET'),
                'photo_spacing': rospy.get_param('~photo_spacing', 'NOT_SET'),
                'coverage_threshold': rospy.get_param('~coverage_threshold', 'NOT_SET'),
                'max_linear_speed': rospy.get_param('/motor_bridge/max_linear_speed', 'NOT_SET'),
                'emergency_distance': rospy.get_param('/safety_supervisor/emergency_distance', 'NOT_SET')
            }
            
        except Exception as e:
            rospy.logwarn(f"Could not load config info: {e}")
            
    def mapper_state_callback(self, msg):
        """Track high-level robot state changes"""
        new_state = msg.data
        if new_state != self.robot_state:
            self.previous_state = self.robot_state
            self.robot_state = new_state
            
            state_change = {
                'timestamp': time.time(),
                'from': self.previous_state,
                'to': new_state,
                'duration': self.state_duration
            }
            self.state_history.append(state_change)
            
            # Log the decision
            self.log_decision(f"STATE_CHANGE: {self.previous_state} → {new_state}")
            
    def mapper_debug_callback(self, msg):
        """Parse detailed behavioral debug information"""
        try:
            debug_data = json.loads(msg.data)
            
            # Extract exploration status
            if 'stats' in debug_data:
                stats = debug_data['stats']
                self.exploration_status.update({
                    'coverage_percent': stats.get('coverage_percent', 0),
                    'frontiers_found': stats.get('frontier_count', 0),
                    'photos_taken': stats.get('photos_taken', 0)
                })
                
            # Extract decision information
            if 'state_description' in debug_data:
                description = debug_data['state_description']
                if description != self.exploration_status.get('last_description'):
                    self.log_decision(f"BEHAVIOR: {description}")
                    self.exploration_status['last_description'] = description
                    
        except json.JSONDecodeError:
            pass
            
    def goal_callback(self, msg):
        """Track navigation goals being set"""
        goal_pos = msg.goal.target_pose.pose.position
        self.current_goal = {
            'x': goal_pos.x,
            'y': goal_pos.y,
            'timestamp': time.time()
        }
        self.goal_history.append(self.current_goal)
        self.log_decision(f"NEW_GOAL: Moving to ({goal_pos.x:.2f}, {goal_pos.y:.2f})")
        
    def result_callback(self, msg):
        """Track goal completion results"""
        status = msg.status.status
        status_text = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED", 
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.REJECTED: "REJECTED"
        }.get(status, f"UNKNOWN({status})")
        
        self.goal_status = status_text
        self.log_decision(f"GOAL_RESULT: {status_text}")
        
        if status == GoalStatus.ABORTED:
            self.exploration_status['stuck_count'] += 1
            self.log_decision(f"STUCK_DETECTED: Count = {self.exploration_status['stuck_count']}")
            
    def scan_callback(self, msg):
        """Analyze LiDAR data for environmental understanding"""
        # Basic obstacle detection
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.lidar_data['ranges_count'] = len(ranges)
        
        if ranges:
            self.nearest_obstacle = min(ranges)
            
            # Analyze directional clearance (simplified)
            total_ranges = len(msg.ranges)
            quarter = total_ranges // 4
            
            # Front, left, right, rear sectors
            front_ranges = ranges[:quarter] + ranges[-quarter:]
            left_ranges = ranges[quarter:quarter*2]
            right_ranges = ranges[quarter*3:quarter*4]
            rear_ranges = ranges[quarter*2:quarter*3]
            
            self.lidar_data.update({
                'front_clear': min(front_ranges) > 0.5 if front_ranges else False,
                'left_clear': min(left_ranges) > 0.3 if left_ranges else False,
                'right_clear': min(right_ranges) > 0.3 if right_ranges else False,
                'rear_clear': min(rear_ranges) > 0.3 if rear_ranges else False
            })
            
            # Detect if we're getting close to obstacles
            if self.nearest_obstacle < 0.3:
                self.log_decision(f"OBSTACLE_CLOSE: {self.nearest_obstacle:.2f}m ahead")
                
    def cmd_vel_callback(self, msg):
        """Track movement commands"""
        self.navigation_status['current_velocity'] = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        
        # Detect if robot is trying to move but can't (stuck)
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.navigation_status['planning_active'] = True
        else:
            self.navigation_status['planning_active'] = False
            
    def safety_callback(self, msg):
        """Track safety system decisions"""
        if "EMERGENCY" in msg.data.upper():
            self.log_decision(f"SAFETY_EMERGENCY: {msg.data}")
        elif "CRITICAL" in msg.data.upper():
            self.log_decision(f"SAFETY_CRITICAL: {msg.data}")
            
    def photo_callback(self, msg):
        """Track photo capture activities"""
        if "captured" in msg.data.lower():
            self.photo_status['photos_taken'] += 1
            self.log_decision(f"PHOTO_CAPTURED: Total = {self.photo_status['photos_taken']}")
        elif "moving to photo" in msg.data.lower():
            self.log_decision(f"PHOTO_NAVIGATION: {msg.data}")
            
    def coverage_callback(self, msg):
        """Track map coverage progress"""
        old_coverage = self.exploration_status['coverage_percent']
        self.exploration_status['coverage_percent'] = msg.data
        
        # Log significant coverage improvements
        if msg.data - old_coverage > 5.0:  # 5% improvement
            self.log_decision(f"COVERAGE_PROGRESS: {old_coverage:.1f}% → {msg.data:.1f}%")
            
    def log_decision(self, decision):
        """Log a decision or behavioral event"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        decision_entry = f"[{timestamp}] {decision}"
        self.decision_log.append(decision_entry)
        
    def get_current_behavior_summary(self):
        """Generate a summary of current robot behavior"""
        summary = []
        
        # Current state and what it means
        summary.append(f"🤖 STATE: {self.robot_state}")
        
        # Decision making status
        if self.current_goal:
            goal_age = time.time() - self.current_goal['timestamp']
            summary.append(f"🎯 GOAL: ({self.current_goal['x']:.1f}, {self.current_goal['y']:.1f}) - {goal_age:.0f}s ago")
        else:
            summary.append("🎯 GOAL: No active goal")
            
        # Environmental awareness
        if self.nearest_obstacle != float('inf'):
            summary.append(f"👁️ PERCEPTION: Nearest obstacle {self.nearest_obstacle:.2f}m")
        
        # Movement status
        vel = self.navigation_status['current_velocity']
        if abs(vel['linear']) > 0.01 or abs(vel['angular']) > 0.01:
            summary.append(f"🚗 MOVEMENT: Linear {vel['linear']:.2f} m/s, Angular {vel['angular']:.2f} rad/s")
        else:
            summary.append("🚗 MOVEMENT: Stopped")
            
        # Exploration progress
        summary.append(f"🗺️ EXPLORATION: {self.exploration_status['coverage_percent']:.1f}% coverage")
        
        return summary
        
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')
        
    def display_loop(self):
        """Main display loop for behavioral debugging"""
        while not rospy.is_shutdown():
            try:
                self.update_behavioral_display()
                time.sleep(1.0 / self.update_frequency)
            except Exception as e:
                rospy.logerr(f"Display error: {e}")
                time.sleep(1.0)
                
    def update_behavioral_display(self):
        """Update the behavioral debug display"""
        self.clear_screen()
        
        # Header
        print("=" * 100)
        print("🧠 TOURMINATOR BEHAVIORAL DEBUG MONITOR - Understanding Robot Decision Making".center(100))
        uptime = time.time() - self.start_time
        print(f"Uptime: {uptime/60:.1f} minutes".center(100))
        print("=" * 100)
        
        # Current Behavior Summary
        print("\n🤖 CURRENT BEHAVIOR ANALYSIS")
        print("-" * 60)
        behavior_summary = self.get_current_behavior_summary()
        for item in behavior_summary:
            print(f"  {item}")
            
        # Decision Making Log (last 10 decisions)
        print("\n🧩 RECENT DECISIONS & BEHAVIORS")
        print("-" * 60)
        recent_decisions = list(self.decision_log)[-10:]
        if recent_decisions:
            for decision in recent_decisions:
                print(f"  {decision}")
        else:
            print("  No decisions logged yet...")
            
        # Environmental Understanding
        print("\n👁️ ENVIRONMENTAL PERCEPTION")
        print("-" * 60)
        print(f"  LiDAR Points: {self.lidar_data['ranges_count']}")
        print(f"  Nearest Obstacle: {self.nearest_obstacle:.2f}m")
        print(f"  Direction Clear: Front={self.lidar_data['front_clear']}, " +
              f"Left={self.lidar_data['left_clear']}, Right={self.lidar_data['right_clear']}")
              
        # Exploration Status
        print("\n🗺️ EXPLORATION & MAPPING STATUS")
        print("-" * 60)
        exp = self.exploration_status
        print(f"  Coverage: {exp['coverage_percent']:.1f}%")
        print(f"  Frontiers Found: {exp['frontiers_found']}")
        print(f"  Stuck Count: {exp['stuck_count']}")
        print(f"  Recovery Attempts: {exp['recovery_attempts']}")
        
        # Photo Mission Status
        print("\n📸 PHOTO MISSION STATUS")
        print("-" * 60)
        photo = self.photo_status
        print(f"  Photos Taken: {photo['photos_taken']}")
        print(f"  Waypoint Progress: {photo['current_waypoint']}/{photo['waypoints_total']}")
        print(f"  Photo Mode Active: {photo['photo_mode_active']}")
        
        # Navigation Planning
        print("\n🧭 NAVIGATION & PLANNING")
        print("-" * 60)
        nav = self.navigation_status
        print(f"  Planning Active: {nav['planning_active']}")
        print(f"  Current Goal Status: {self.goal_status}")
        print(f"  Stuck Detection: {nav['stuck_detected']}")
        print(f"  Recovery Behavior: {nav['recovery_behavior']}")
        
        # Configuration & Files
        print("\n⚙️ CONFIGURATION STATUS")
        print("-" * 60)
        config = self.config_status
        print(f"  Config Files Loaded: {len(config['config_files_loaded'])}")
        for file in config['config_files_loaded'][:3]:  # Show first 3
            print(f"    - {file}")
        if len(config['config_files_loaded']) > 3:
            print(f"    ... and {len(config['config_files_loaded'])-3} more")
            
        key_params = config['parameters_active']
        print(f"  Key Parameters:")
        print(f"    - Exploration Timeout: {key_params.get('exploration_timeout', 'NOT_SET')}")
        print(f"    - Coverage Threshold: {key_params.get('coverage_threshold', 'NOT_SET')}")
        print(f"    - Emergency Distance: {key_params.get('emergency_distance', 'NOT_SET')}")
        
        # State History (compact)
        print("\n📊 STATE TRANSITION HISTORY")
        print("-" * 60)
        recent_states = list(self.state_history)[-5:]
        if recent_states:
            for state in recent_states:
                duration = state['duration']
                print(f"  {state['from']} → {state['to']} (held {duration:.0f}s)")
        else:
            print("  No state changes recorded yet...")
            
        # Help text
        print("\n" + "=" * 100)
        print("💡 This monitor shows robot's decision-making process, environmental perception,")
        print("   exploration progress, and configuration status. Press Ctrl+C to exit.")
        print("=" * 100)
        
        # Update timing
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"\nLast updated: {current_time}")
        
    # Placeholder callbacks for missing methods
    def cmd_vel_nav_callback(self, msg):
        pass
        
    def status_callback(self, msg):
        pass
        
    def odom_callback(self, msg):
        pass
        
    def map_callback(self, msg):
        pass
        
    def emergency_callback(self, msg):
        if msg.data:
            self.log_decision("EMERGENCY_STOP_ACTIVATED")

def main():
    try:
        monitor = BehavioralDebugMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("\n🛑 Behavioral debug monitor shutting down...")
    except KeyboardInterrupt:
        print("\n🛑 Behavioral debug monitor interrupted by user")
    except Exception as e:
        print(f"\n❌ Error in behavioral debug monitor: {e}")

if __name__ == '__main__':
    main()

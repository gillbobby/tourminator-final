#!/usr/bin/env python3
"""
Simple Safety Supervisor - Front LiDAR Only
Designed for robot with front-hemisphere LiDAR sensing

Simple approach:
- Find minimum distance in front LiDAR field
- Apply safety speed scaling based on distance
- Handle reverse motion conservatively (no rear sensing)
"""

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

class SimpleSafetySupervisor:
    def __init__(self):
        rospy.init_node('safety_supervisor', anonymous=True)
        
        # SAFETY ZONES - Direct LiDAR distances
        self.safety_zones = {
            'emergency': rospy.get_param('~emergency_distance', 0.05),   # 25cm - HARD STOP
            'critical': rospy.get_param('~critical_distance', 0.10),     # 40cm - Heavy braking
            'warning': rospy.get_param('~warning_distance', 0.20),       # 60cm - Slow down
            'caution': rospy.get_param('~caution_distance', 0.30)        # 80cm - Reduce speed
        }
        
        # SPEED SCALING FACTORS
        self.speed_factors = {
            'emergency': 0.0,      # Complete stop
            'critical': 0.15,      # Very slow
            'warning': 0.50,       # Half speed  
            'caution': 0.75,       # Reduced speed
            'clear': 1.0           # Full speed
        }
        
        # REVERSE MOTION SAFETY (no rear sensing)
        self.max_reverse_speed = rospy.get_param('~max_reverse_speed', 0.08)  # 8cm/s max reverse
        
        # FRONT LIDAR FIELD OF VIEW
        self.front_angle_min = rospy.get_param('~front_angle_min', -90.0)    # -90° (left)
        self.front_angle_max = rospy.get_param('~front_angle_max', 90.0)     # +90° (right)
        
        # FILTERING
        self.min_valid_range = rospy.get_param('~min_valid_range', 0.15)     # Ignore readings < 15cm
        self.max_valid_range = rospy.get_param('~max_valid_range', 8.0)      # Ignore readings > 8m
        
        # STATE TRACKING
        self.current_safety_level = 'clear'
        self.min_distance = float('inf')
        self.last_cmd_vel = Twist()
        self.scan_data_valid = False
        self.emergency_stops = 0
        
        # ROS INTERFACES
        # Input: Navigation commands
        self.cmd_nav_sub = rospy.Subscriber('/cmd_vel_nav', Twist, self.navigation_command_callback)
        # Input: LiDAR data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Output: Safe commands to motor bridge
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Status publishing
        self.safety_status_pub = rospy.Publisher('/safety_status', String, queue_size=1)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20 Hz
        
        rospy.loginfo("✅ Simple Safety Supervisor started")
        rospy.loginfo(f"🛡️  Safety zones: {self.safety_zones}")
        rospy.loginfo(f"📡 Front LiDAR field: {self.front_angle_min}° to {self.front_angle_max}°")
        rospy.loginfo(f"⏪ Max reverse speed: {self.max_reverse_speed} m/s")

    def scan_callback(self, scan_msg):
        """Process front LiDAR data to find minimum distance"""
        self.scan_data_valid = True
        min_distance_this_scan = float('inf')
        
        # Process each laser reading
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid readings
            if not self.is_valid_range(range_val):
                continue
            
            # Calculate angle for this reading
            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Only process front hemisphere
            if not self.is_in_front_field(angle_deg):
                continue
            
            # Track minimum distance
            if range_val < min_distance_this_scan:
                min_distance_this_scan = range_val
        
        # Update global minimum distance and safety level
        self.min_distance = min_distance_this_scan
        self.update_safety_level(min_distance_this_scan)

    def is_valid_range(self, range_val):
        """Check if LiDAR range reading is valid"""
        return (not math.isnan(range_val) and 
                not math.isinf(range_val) and
                self.min_valid_range <= range_val <= self.max_valid_range)

    def is_in_front_field(self, angle_deg):
        """Check if angle is in front field of view"""
        # Normalize angle to -180 to +180
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        
        return self.front_angle_min <= angle_deg <= self.front_angle_max

    def update_safety_level(self, min_distance):
        """Determine safety level based on minimum distance"""
        
        if min_distance == float('inf'):
            new_level = 'clear'
        elif min_distance <= self.safety_zones['emergency']:
            new_level = 'emergency'
        elif min_distance <= self.safety_zones['critical']:
            new_level = 'critical'
        elif min_distance <= self.safety_zones['warning']:
            new_level = 'warning'
        elif min_distance <= self.safety_zones['caution']:
            new_level = 'caution'
        else:
            new_level = 'clear'
        
        # Log safety level changes
        if new_level != self.current_safety_level:
            rospy.logwarn(f"🚨 SAFETY: {self.current_safety_level} → {new_level}")
            
            if min_distance != float('inf'):
                rospy.logwarn(f"   Closest obstacle: {min_distance:.2f}m ahead")
            
            if new_level == 'emergency':
                self.emergency_stops += 1
                self.emergency_pub.publish(Bool(True))
                rospy.logwarn(f"🛑 EMERGENCY STOP #{self.emergency_stops}")
            elif self.current_safety_level == 'emergency':
                self.emergency_pub.publish(Bool(False))
                rospy.loginfo("✅ Emergency cleared")
        
        self.current_safety_level = new_level
        
        # Publish safety status
        if min_distance != float('inf'):
            status_msg = f"{new_level}:{min_distance:.2f}:front:simple"
        else:
            status_msg = f"{new_level}:inf:clear:simple"
        
        self.safety_status_pub.publish(String(status_msg))

    def navigation_command_callback(self, cmd_msg):
        """Receive navigation commands that need safety processing"""
        self.last_cmd_vel = cmd_msg

    def control_loop(self, event):
        """Apply safety limits to navigation commands"""
        if not self.scan_data_valid:
            return
        
        # Get commanded velocities
        cmd_linear = self.last_cmd_vel.linear.x
        cmd_angular = self.last_cmd_vel.angular.z
        
        # Apply safety scaling based on motion direction
        safe_cmd = Twist()
        
        if cmd_linear >= 0:
            # FORWARD MOTION - use front LiDAR safety
            safety_factor = self.speed_factors[self.current_safety_level]
            safe_cmd.linear.x = cmd_linear * safety_factor
            safe_cmd.angular.z = cmd_angular * safety_factor
            
        else:
            # REVERSE MOTION - conservative speed (no rear sensing)
            safe_cmd.linear.x = max(cmd_linear, -self.max_reverse_speed)
            safe_cmd.angular.z = cmd_angular * 0.5  # Slower turns when reversing
            
            # Additional safety: stop reverse if emergency in front
            # (might be backing into detected obstacle)
            if self.current_safety_level == 'emergency':
                safe_cmd.linear.x = 0.0
                safe_cmd.angular.z = 0.0
        
        # Publish safe command
        self.cmd_vel_pub.publish(safe_cmd)
        
        # Log safety interventions
        if (abs(safe_cmd.linear.x - cmd_linear) > 0.01 or 
            abs(safe_cmd.angular.z - cmd_angular) > 0.01):
            rospy.logdebug(f"SAFETY OVERRIDE: cmd({cmd_linear:.2f}, {cmd_angular:.2f}) → "
                          f"safe({safe_cmd.linear.x:.2f}, {safe_cmd.angular.z:.2f})")

if __name__ == '__main__':
    try:
        safety_supervisor = SimpleSafetySupervisor()
        rospy.loginfo("🛡️  Simple Safety Supervisor active!")
        rospy.loginfo("🎯 Protecting front hemisphere with distance-based safety")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Safety Supervisor shutting down")
    except Exception as e:
        rospy.logerr(f"Safety Supervisor error: {e}")
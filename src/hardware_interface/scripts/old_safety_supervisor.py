#!/usr/bin/env python3
"""
Safety Supervisor Node - RAY-INTERSECTION METHOD
Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/src/safety_supervisor.py

COORDINATE SYSTEM:
+x = right, -x = left
+y = forward, -y = backward
angles CCW from +x axis (0° = right, 90° = forward, 180° = left, 270° = backward)

RAY-INTERSECTION APPROACH:
For each LiDAR reading, find where that same beam exits the robot chassis,
then calculate actual clearance = measured_distance - exit_distance
"""

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class SafetySupervisor:
    def __init__(self):
        rospy.init_node('safety_supervisor', anonymous=True)
        
        # SAFETY PARAMETERS - DISTANCES FROM ROBOT SURFACE
        self.safety_zones = {
            'emergency': rospy.get_param('~emergency_distance', 0.20),   # 20cm from robot surface - HARD STOP
            'critical': rospy.get_param('~critical_distance', 0.30),     # 30cm from robot surface - Heavy braking  
            'warning': rospy.get_param('~warning_distance', 0.45),       # 45cm from robot surface - Slow down
            'caution': rospy.get_param('~caution_distance', 0.60)        # 60cm from robot surface - Reduce speed
        }
        
        # SPEED CONTROL FACTORS
        self.speed_factors = {
            'emergency': 0.05,    # 
            'critical': 0.20,     # 15% of commanded speed
            'warning': 0.60,      # 45% of commanded speed
            'caution': 0.9,      # 80% of commanded speed
            'clear': 1.0         # Full speed
        }
        
        # ACCELERATION LIMITS
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 0.2)    # m/s²
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 0.2)  # rad/s²
        
        # ROBOT CHASSIS BOUNDARIES (distances from LiDAR to robot edges)
        # Coordinate system: +x=right, -x=left, +y=forward, -y=backward
        self.robot_boundaries = {
            'x_min': rospy.get_param('~robot_left_edge', -0.22),    # Left edge: 22cm left of LiDAR
            'x_max': rospy.get_param('~robot_right_edge', 0.26),    # Right edge: 26cm right of LiDAR  
            'y_min': rospy.get_param('~robot_back_edge', -0.293),    # Back edge: 29cm behind LiDAR
            'y_max': rospy.get_param('~robot_front_edge', 0.03)     # Front edge: 3cm forward of LiDAR
        }

        #emergency handling
        self.emergency_start_time = None
        self.emergency_escape_timeout = rospy.get_param('~emergency_escape_timeout', 5.0)  # 5 seconds
        self.escape_behavior_active = False

        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            rospy.loginfo("✅ Costmap clearing service connected")
        except rospy.ROSException:
            rospy.logwarn("⚠️  Costmap clearing service not available - escape may be limited")
            self.clear_costmaps = None
        
        # POLE FILTERING - Filter out robot's camera pole
        self.pole_filter_enabled = rospy.get_param('~pole_filter_enabled', True)
        self.pole_angle_center = rospy.get_param('~pole_angle_center', 280.0)    # Angle where pole appears
        self.pole_angle_range = rospy.get_param('~pole_angle_range', 20.0)       # ±10° around pole
        self.pole_min_distance = rospy.get_param('~pole_min_distance', 0.20)     # Filter readings < 15cm
        
        # Current state
        self.current_safety_level = 'clear'
        self.min_clearance_global = float('inf')
        self.last_cmd_vel = Twist()
        self.current_output_vel = Twist()
        self.last_update_time = rospy.Time.now()
        self.scan_data_valid = False
        
        # Emergency tracking
        self.emergency_stops = 0
        self.safety_overrides = 0
        
        # ROS interfaces
        self.cmd_nav_sub = rospy.Subscriber('/cmd_vel_nav', Twist, self.navigation_command_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Output safe commands to motor_bridge
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Safety status publishing
        self.safety_status_pub = rospy.Publisher('/safety_status', String, queue_size=1)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # Control timer for smooth acceleration/deceleration
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20 Hz
        
        rospy.loginfo("Safety Supervisor started with RAY-INTERSECTION method")
        rospy.loginfo(f"Safety zones (from robot surface): {self.safety_zones}")
        rospy.loginfo(f"Robot boundaries: {self.robot_boundaries}")
        rospy.loginfo("Coordinate system: +x=right, +y=forward, -x=left, -y=backward")

    def calculate_robot_clearance(self, angle_deg, measured_distance):
        """
        Calculate actual clearance from obstacle to robot surface using ray-intersection method
        
        Args:
            angle_deg: LiDAR angle in degrees (0°=right, 90°=forward, 180°=left, 270°=backward)
            measured_distance: Distance from LiDAR to obstacle in meters
        
        Returns:
            Actual clearance from obstacle to robot surface in meters
        """
        
        # Convert angle to radians
        angle_rad = math.radians(angle_deg)
        cos_theta = math.cos(angle_rad)
        sin_theta = math.sin(angle_rad)
        
        # Find where this ray first exits the robot chassis
        exit_distances = []
        
        # Check intersection with left edge (x = x_min)
        if cos_theta < 0:  # Ray pointing left (negative x direction)
            r_x = abs(self.robot_boundaries['x_min']) / abs(cos_theta)
            exit_distances.append(r_x)
        
        # Check intersection with right edge (x = x_max)  
        if cos_theta > 0:  # Ray pointing right (positive x direction)
            r_x = abs(self.robot_boundaries['x_max']) / abs(cos_theta)
            exit_distances.append(r_x)
        
        # Check intersection with back edge (y = y_min)
        if sin_theta < 0:  # Ray pointing backward (negative y direction)
            r_y = abs(self.robot_boundaries['y_min']) / abs(sin_theta)
            exit_distances.append(r_y)
        
        # Check intersection with front edge (y = y_max)
        if sin_theta > 0:  # Ray pointing forward (positive y direction)
            r_y = abs(self.robot_boundaries['y_max']) / abs(sin_theta)
            exit_distances.append(r_y)
        
        # Ray exits robot at the closest intersection
        if exit_distances:
            robot_exit_distance = min(exit_distances)
            
            # Calculate actual clearance
            clearance = measured_distance - robot_exit_distance
            
            # Debug for close obstacles
            if clearance < 0.5:
                rospy.logdebug(f"Angle {angle_deg:.0f}°: measured={measured_distance:.3f}m, "
                              f"robot_exit={robot_exit_distance:.3f}m, clearance={clearance:.3f}m")
            
            return max(0.0, clearance)  # Ensure non-negative clearance
        else:
            # Shouldn't happen, but return measured distance as fallback
            rospy.logwarn(f"No robot boundary intersection found for angle {angle_deg}°")
            return measured_distance

    def scan_callback(self, scan_msg):
        """
        Process incoming LiDAR scan data using ray-intersection method
        """
        self.scan_data_valid = True
        
        # Track minimum clearance across all directions
        min_clearance_this_scan = float('inf')
        closest_angle = None
        
        # Process each laser reading
        for i, range_val in enumerate(scan_msg.ranges):
            # Validate range reading
            if not self.is_valid_range(range_val, scan_msg.range_min, scan_msg.range_max):
                continue
            
            # Calculate angle for this reading
            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            # Apply pole filtering if enabled
            if self.pole_filter_enabled and self.is_pole_detection(angle_deg, range_val):
                continue  # Skip pole detections
            
            # Calculate actual clearance to robot surface
            actual_clearance = self.calculate_robot_clearance(angle_deg, range_val)
            
            # Track minimum clearance
            if actual_clearance < min_clearance_this_scan:
                min_clearance_this_scan = actual_clearance
                closest_angle = angle_deg
        
        # Update global minimum clearance and safety level
        self.min_clearance_global = min_clearance_this_scan
        self.update_safety_level(min_clearance_this_scan, closest_angle)

    def is_valid_range(self, range_val, range_min, range_max):
        """Check if LiDAR range reading is valid"""
        return not (math.isnan(range_val) or math.isinf(range_val) or 
                   range_val < range_min or range_val > range_max)

    def is_pole_detection(self, angle_deg, distance):
        """Filter out robot's camera pole detections"""
        angle_diff = abs(angle_deg - self.pole_angle_center)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff  # Handle wrap-around
        
        in_pole_sector = angle_diff <= (self.pole_angle_range / 2.0)
        too_close = distance <= self.pole_min_distance

        if distance < 0.13:  # Only for very close readings
            rospy.logwarn_throttle(1.0, f"🔍 POLE CHECK: angle={angle_deg:.1f}° dist={distance:.3f}m "
                                f"in_sector={in_pole_sector} too_close={too_close} "
                                f"angle_diff={angle_diff:.1f}°")
        
        return in_pole_sector and too_close

    def get_direction_name(self, angle_deg):
        """Convert angle to human-readable direction name"""
        if angle_deg is None:
            return "unknown"
        
        # Normalize angle to 0-360
        angle = angle_deg % 360
        
        if 315 <= angle or angle < 45:
            return "right"
        elif 45 <= angle < 135:
            return "front"
        elif 135 <= angle < 225:
            return "left"
        elif 225 <= angle < 315:
            return "back"
        else:
            return f"{angle:.0f}°"

    def handle_emergency_escape(self, new_level):
        """Handle emergency escape behavior when stuck too long."""
        if new_level == 'emergency':
            if self.emergency_start_time is None:
                self.emergency_start_time = rospy.Time.now()
                rospy.logwarn("🚨 Emergency detected - starting escape timer")

            elapsed = (rospy.Time.now() - self.emergency_start_time).to_sec()
            if elapsed > self.emergency_escape_timeout:
                if not self.escape_behavior_active:
                    rospy.logwarn(f"🔄 EMERGENCY ESCAPE after {elapsed:.1f}s - attempting escape")
                    self.escape_behavior_active = True

                    # clear the local costmap once
                    if self.clear_costmaps is not None:
                        try:
                            self.clear_costmaps()
                            rospy.logwarn("🗺️  Costmaps cleared - move_base can replan escape path")
                        except rospy.ServiceException as e:
                            rospy.logwarn(f"Failed to clear costmaps: {e}")
                    else:
                        rospy.logwarn("🗺️  Costmap clearing not available - relying on speed reduction only")

                # drop to "critical" so you crawl out
                return 'critical'
        else:
            if self.emergency_start_time is not None:
                duration = (rospy.Time.now() - self.emergency_start_time).to_sec()
                rospy.logwarn(f"✅ Emergency cleared after {duration:.1f}s")
            self.emergency_start_time = None
            self.escape_behavior_active = False

        return new_level

    def update_safety_level(self, min_clearance, closest_angle):
        """Determine current safety level based on minimum robot surface clearance"""
        
        if min_clearance == float('inf'):
            new_level = 'clear'
        elif min_clearance <= self.safety_zones['emergency']:
            new_level = 'emergency'
        elif min_clearance <= self.safety_zones['critical']:
            new_level = 'critical'
        elif min_clearance <= self.safety_zones['warning']:
            new_level = 'warning'
        elif min_clearance <= self.safety_zones['caution']:
            new_level = 'caution'
        else:
            new_level = 'clear'
        
        # Log safety level changes
        if new_level != self.current_safety_level:
            rospy.logwarn(f"SAFETY LEVEL: {self.current_safety_level} → {new_level}")
            
            if min_clearance != float('inf'):
                direction = self.get_direction_name(closest_angle)
                rospy.logwarn(f"  Closest obstacle: {min_clearance:.2f}m from robot surface ({direction})")
            
            if new_level == 'emergency':
                self.emergency_stops += 1
                self.emergency_pub.publish(Bool(True))
                rospy.logwarn(f"🚨 EMERGENCY STOP #{self.emergency_stops} - OBSTACLE TOO CLOSE!")
            elif self.current_safety_level == 'emergency':
                self.emergency_pub.publish(Bool(False))
                rospy.loginfo("✅ Emergency condition cleared")
        
        new_level = self.handle_emergency_escape(new_level)
        self.current_safety_level = new_level
        
        # Publish safety status
        if min_clearance != float('inf'):
            direction = self.get_direction_name(closest_angle)
            status_msg = f"{new_level}:{min_clearance:.2f}:{direction}:ray_intersection"
        else:
            status_msg = f"{new_level}:inf:clear:ray_intersection"
        
        self.safety_status_pub.publish(String(status_msg))

    def navigation_command_callback(self, cmd_msg):
        """Receive navigation commands that need safety processing"""
        self.last_cmd_vel = cmd_msg

    def control_loop(self, event):
        """Main control loop with smooth acceleration/deceleration"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        
        if dt <= 0 or not self.scan_data_valid:
            return
        
        # Get safety factor based on current safety level
        safety_factor = self.speed_factors[self.current_safety_level]
        target_linear = self.last_cmd_vel.linear.x * safety_factor
        target_angular = self.last_cmd_vel.angular.z * safety_factor
        
        # Apply acceleration limits for smooth motion
        new_linear = self.apply_acceleration_limit(
            self.current_output_vel.linear.x, 
            target_linear, 
            self.max_linear_accel, 
            dt
        )
        
        new_angular = self.apply_acceleration_limit(
            self.current_output_vel.angular.z,
            target_angular,
            self.max_angular_accel,
            dt
        )
        
        # Create safe output command
        safe_cmd = Twist()
        safe_cmd.linear.x = new_linear
        safe_cmd.angular.z = new_angular
        
        # Track safety interventions
        if abs(safe_cmd.linear.x - self.last_cmd_vel.linear.x) > 0.01 or \
           abs(safe_cmd.angular.z - self.last_cmd_vel.angular.z) > 0.01:
            self.safety_overrides += 1
            
            if self.safety_overrides % 50 == 0:  # Log every 2.5 seconds
                rospy.loginfo(f"SAFETY OVERRIDE: {self.current_safety_level} - "
                             f"cmd({self.last_cmd_vel.linear.x:.2f}, {self.last_cmd_vel.angular.z:.2f}) → "
                             f"safe({safe_cmd.linear.x:.2f}, {safe_cmd.angular.z:.2f})")
        
        # Publish safe command to motor_bridge
        self.cmd_vel_pub.publish(safe_cmd)
        self.current_output_vel = safe_cmd
        self.last_update_time = current_time

    def apply_acceleration_limit(self, current_vel, target_vel, max_accel, dt):
        """Apply acceleration limits for smooth speed changes"""
        velocity_diff = target_vel - current_vel
        max_change = max_accel * dt
        
        if abs(velocity_diff) <= max_change:
            return target_vel
        else:
            if velocity_diff > 0:
                return current_vel + max_change
            else:
                return current_vel - max_change

    def odom_callback(self, odom_msg):
        """Track current robot velocity for advanced safety calculations"""
        pass

if __name__ == '__main__':
    try:
        safety_supervisor = SafetySupervisor()
        rospy.loginfo("🛡️  Safety Supervisor with RAY-INTERSECTION method active!")
        rospy.loginfo("🎯 All safety zones now measured from robot surface using ray-intersection")
        rospy.loginfo("📐 Coordinate system: +x=right, +y=forward, -x=left, -y=backward")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Safety Supervisor shutting down")
    except Exception as e:
        rospy.logerr(f"Safety Supervisor error: {e}")
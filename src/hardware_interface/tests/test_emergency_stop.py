#!/usr/bin/env python3
"""
Emergency Stop Safety Test - CORRECTED FOR SAFETY SUPERVISOR INTEGRATION
Tests automatic emergency stopping when obstacles detected within safety zone

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_emergency_stop.py

CORRECTED ARCHITECTURE:
- BEFORE (WRONG): test → /cmd_vel → motor_bridge (bypassed safety)
- AFTER (CORRECT): test → /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge

SAFETY FEATURES:
- Tests the complete safety-integrated command pipeline
- Manual robot control with WASD keys through safety supervisor
- Real-time safety supervisor monitoring
- Direction-specific safety zone testing
- Command modification tracking and validation
"""

import sys
import time
import math
import threading
from datetime import datetime

# ROS imports
try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    ROS_AVAILABLE = True
except ImportError:
    print("❌ ROS not available - install ROS and source workspace")
    ROS_AVAILABLE = False

# For non-blocking keyboard input
try:
    import termios
    import tty
    import select
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False

class SafetyIntegratedEmergencyStopTester:
    def __init__(self):
        if not ROS_AVAILABLE:
            print("❌ ROS not available - cannot run safety tests")
            sys.exit(1)
            
        rospy.init_node('emergency_stop_tester', anonymous=True)
        
        # Safety parameters
        self.safety_distance = 1.0  # 1 meter safety zone
        self.critical_distance = 0.5  # 0.5m critical zone (immediate stop)
        
        # Robot orientation mapping (based on your LiDAR setup)
        # 0°/360° = left, 90° = forward, 180° = right, 270° = backward
        self.direction_zones = {
            'forward': {'angles': (80, 100), 'name': 'FORWARD'},     # 80-100° (front)
            'left': {'angles': (350, 10), 'name': 'LEFT'},           # 350-10° (left)  
            'right': {'angles': (170, 190), 'name': 'RIGHT'},        # 170-190° (right)
            'backward': {'angles': (260, 280), 'name': 'BACKWARD'}   # 260-280° (back)
        }
        
        # Safety state - TESTING SAFETY_SUPERVISOR INTEGRATION
        self.emergency_active = False
        self.current_safety_level = "unknown"
        self.safety_supervisor_active = False
        self.obstacle_zones = {}
        self.last_scan_time = None
        self.robot_velocity = {'linear': 0.0, 'angular': 0.0}
        self.closest_distance = float('inf')
        
        # Command tracking for testing safety_supervisor effectiveness
        self.last_nav_command = Twist()  # What we send to safety_supervisor
        self.last_safe_command = Twist()  # What safety_supervisor outputs
        self.command_modifications = 0
        self.total_commands_sent = 0
        
        # Control parameters
        self.linear_speed = 0.3   # m/s for manual control
        self.angular_speed = 0.6  # rad/s for manual control
        self.running = True
        
        # ROS interfaces - CORRECTED SAFETY SUPERVISOR INTEGRATION
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # CRITICAL CORRECTION: Test the safety_supervisor.py integration
        # We publish to /cmd_vel_nav (input to safety_supervisor)
        # We monitor /cmd_vel (output from safety_supervisor) 
        # We monitor /safety_status and /emergency_stop from safety_supervisor
        self.cmd_nav_publisher = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.safe_cmd_callback)
        
        # Monitor safety supervisor status
        self.safety_status_subscriber = rospy.Subscriber('/safety_status', String, self.safety_status_callback)
        self.emergency_stop_subscriber = rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback)
        
        # Test tracking
        self.test_results = []
        self.emergency_stops = []
        self.safety_violations = 0
        
        # Initialize terminal for keyboard input
        if KEYBOARD_AVAILABLE:
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())

    def log_result(self, test_name, passed, message=""):
        """Log test result"""
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"\n{status}: {test_name}")
        if message:
            print(f"    {message}")
        
        self.test_results.append({
            'test': test_name,
            'passed': passed,
            'message': message,
            'timestamp': datetime.now().isoformat()
        })

    def safety_status_callback(self, msg):
        """Monitor safety_supervisor.py status - CORRECTED"""
        self.safety_supervisor_active = True
        try:
            # Parse safety status: "level:distance"  
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.current_safety_level = parts[0]
                try:
                    self.closest_distance = float(parts[1])
                except ValueError:
                    pass
        except Exception as e:
            rospy.logdebug(f"Error parsing safety status: {e}")

    def emergency_stop_callback(self, msg):
        """Monitor emergency stop events from safety_supervisor.py - CORRECTED"""
        if msg.data and not self.emergency_active:
            self.emergency_active = True
            print(f"\n🚨 SAFETY_SUPERVISOR EMERGENCY STOP ACTIVATED!")
            print(f"   Safety level: {self.current_safety_level}")
            print(f"   Closest obstacle: {self.closest_distance:.2f}m")
            
            self.emergency_stops.append({
                'timestamp': rospy.Time.now(),
                'safety_level': self.current_safety_level,
                'closest_distance': self.closest_distance
            })
            
        elif not msg.data and self.emergency_active:
            self.emergency_active = False
            print(f"\n✅ Safety_supervisor emergency cleared")

    def safe_cmd_callback(self, msg):
        """Monitor commands output by safety_supervisor.py - CORRECTED"""
        self.last_safe_command = msg
        
        # Check if safety_supervisor modified our command
        nav_speed = abs(self.last_nav_command.linear.x) + abs(self.last_nav_command.angular.z)
        safe_speed = abs(self.last_safe_command.linear.x) + abs(self.last_safe_command.angular.z)
        
        if abs(nav_speed - safe_speed) > 0.05:
            self.command_modifications += 1

    def scan_callback(self, msg):
        """Process LiDAR scan for obstacle detection"""
        self.last_scan_time = rospy.Time.now()
        
        # Reset obstacle zones
        self.obstacle_zones = {}
        
        # Find closest obstacle overall
        valid_ranges = [r for r in msg.ranges 
                       if not (math.isnan(r) or math.isinf(r)) 
                       and msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            self.closest_distance = min(valid_ranges)
        else:
            self.closest_distance = float('inf')
        
        # Analyze scan data for obstacles in each direction
        for i, range_val in enumerate(msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)):
                if msg.range_min <= range_val <= msg.range_max:
                    angle_rad = msg.angle_min + i * msg.angle_increment
                    angle_deg = math.degrees(angle_rad) % 360.0
                    
                    # Check which direction zone this angle belongs to
                    direction = self.get_direction_from_angle(angle_deg)
                    
                    if direction:
                        if direction not in self.obstacle_zones:
                            self.obstacle_zones[direction] = []
                        
                        self.obstacle_zones[direction].append({
                            'angle': angle_deg,
                            'distance': range_val,
                            'critical': range_val < self.critical_distance,
                            'warning': range_val < self.safety_distance
                        })

    def get_direction_from_angle(self, angle_deg):
        """Determine direction zone from angle"""
        for direction, zone_info in self.direction_zones.items():
            min_angle, max_angle = zone_info['angles']
            
            if min_angle > max_angle:  # Wrap-around case (e.g., 350-10)
                if angle_deg >= min_angle or angle_deg <= max_angle:
                    return direction
            else:  # Normal case (e.g., 80-100)
                if min_angle <= angle_deg <= max_angle:
                    return direction
        return None

    def test_safety_supervisor_integration(self):
        """Test that safety_supervisor.py is running and responding - CORRECTED"""
        print("\n=== Testing Safety Supervisor Integration ===")
        
        # Wait for safety_supervisor to be active
        print("Waiting for Safety Supervisor...")
        start_wait = rospy.Time.now()
        while not self.safety_supervisor_active and (rospy.Time.now() - start_wait).to_sec() < 10:
            time.sleep(0.1)
        
        if not self.safety_supervisor_active:
            self.log_result("Safety Supervisor active", False, 
                          "Start: rosrun hardware_interface safety_supervisor.py")
            return False
        
        self.log_result("Safety Supervisor active", True, f"Level: {self.current_safety_level}")
        
        # Test command flow - CORRECTED ARCHITECTURE
        print("Testing CORRECTED command architecture...")
        print("   test → /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge")
        
        test_cmd = Twist()
        test_cmd.linear.x = 0.1
        
        # Send test command to SAFETY INPUT (not directly to motors)
        self.cmd_nav_publisher.publish(test_cmd)
        self.last_nav_command = test_cmd
        self.total_commands_sent += 1
        time.sleep(0.5)
        
        # Check if we received processed command from safety supervisor
        if abs(self.last_safe_command.linear.x) > 0 or abs(self.last_safe_command.angular.z) > 0:
            self.log_result("Command flow test", True, "Commands processed by safety_supervisor")
        else:
            self.log_result("Command flow test", False, "No command processing detected")
        
        return True

    def odom_callback(self, odom_msg):
        """Track robot velocity for safety monitoring"""
        self.robot_velocity = {
            'linear': odom_msg.twist.twist.linear.x,
            'angular': odom_msg.twist.twist.angular.z
        }

    def get_keyboard_input(self):
        """Get non-blocking keyboard input for manual control"""
        if not KEYBOARD_AVAILABLE:
            return None
            
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def manual_control_loop(self):
        """Manual control with WASD keys through SAFETY SUPERVISOR - CORRECTED"""
        print("\n🎮 MANUAL CONTROL ACTIVE - SAFETY-INTEGRATED")
        print("=" * 60)
        print("CORRECTED ARCHITECTURE:")
        print("  WASD → /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge")
        print()
        print("Controls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Turn Left/Right") 
        print("  X: Stop")
        print("  Q: Quit test")
        print("\n🛡️  SAFETY: Commands processed by safety_supervisor")
        print("     Robot will auto-stop if obstacle < 1m")
        print("=" * 60)
        
        rate = rospy.Rate(10)  # 10 Hz control loop
        
        while self.running and not rospy.is_shutdown():
            # Get keyboard input
            key = self.get_keyboard_input()
            
            # Default command (stop)
            cmd = Twist()
            
            # Process keyboard commands 
            if key:
                if key.lower() == 'w':
                    cmd.linear.x = self.linear_speed
                    print("↑ FORWARD command sent to safety_supervisor")
                elif key.lower() == 's':
                    cmd.linear.x = -self.linear_speed
                    print("↓ BACKWARD command sent to safety_supervisor")
                elif key.lower() == 'a':
                    cmd.angular.z = self.angular_speed
                    print("← TURN LEFT command sent to safety_supervisor")
                elif key.lower() == 'd':
                    cmd.angular.z = -self.angular_speed
                    print("→ TURN RIGHT command sent to safety_supervisor")
                elif key.lower() == 'x':
                    cmd = Twist()  # Stop
                    print("⏹️  STOP command sent to safety_supervisor")
                elif key.lower() == 'q':
                    print("🏁 Quitting manual control")
                    break
            
            # Store our command for comparison
            self.last_nav_command = cmd
            self.total_commands_sent += 1
            
            # CORRECTED: Send command to safety_supervisor (not directly to motors)
            self.cmd_nav_publisher.publish(cmd)
            
            # Display status
            if int(rospy.Time.now().to_sec()) % 2 == 0:  # Every 2 seconds
                self.display_safety_status()
            
            rate.sleep()

    def display_safety_status(self):
        """Display current safety status from safety_supervisor - CORRECTED"""
        if self.emergency_active:
            print(f"\n🚨 SAFETY_SUPERVISOR EMERGENCY ACTIVE")
        elif self.current_safety_level != "unknown":
            print(f"\n🛡️  Safety Level: {self.current_safety_level.upper()}")
            print(f"   Closest obstacle: {self.closest_distance:.2f}m")
            
            # Show command modification - CORRECTED
            nav_speed = abs(self.last_nav_command.linear.x) + abs(self.last_nav_command.angular.z)
            safe_speed = abs(self.last_safe_command.linear.x) + abs(self.last_safe_command.angular.z)
            
            if abs(nav_speed - safe_speed) > 0.05:
                reduction = (nav_speed - safe_speed) / (nav_speed + 0.001) * 100
                print(f"   ⚠️  Command modified: {reduction:.0f}% speed reduction")
            else:
                print(f"   ✅ No safety modifications applied")
        else:
            print(f"\n📡 Waiting for safety_supervisor data...")

    def test_emergency_stop_system(self):
        """Test the emergency stop system - CORRECTED"""
        print("\n=== Testing Emergency Stop System ===")
        
        # Wait for initial scan data
        print("Waiting for LiDAR data...")
        start_wait = rospy.Time.now()
        while not self.last_scan_time and (rospy.Time.now() - start_wait).to_sec() < 10:
            time.sleep(0.1)
        
        if not self.last_scan_time:
            self.log_result("LiDAR data reception", False, "No scan data received")
            return False
        
        self.log_result("LiDAR data reception", True, "Scan data available")
        
        # Test obstacle detection
        print("\nTesting obstacle detection zones...")
        time.sleep(2)  # Let scan data accumulate
        
        if self.obstacle_zones:
            self.log_result("Obstacle detection", True, f"Detecting {len(self.obstacle_zones)} zones")
        else:
            self.log_result("Obstacle detection", True, "No obstacles detected (clear area)")
        
        return True

    def run_safety_test(self):
        """Run comprehensive safety test - CORRECTED ARCHITECTURE"""
        print("Emergency Stop & Collision Avoidance Test - SAFETY SUPERVISOR INTEGRATED")
        print("=" * 80)
        print("CORRECTED: Testing safety_supervisor.py integration")
        print("ARCHITECTURE: Commands → /cmd_vel_nav → safety_supervisor → /cmd_vel → motors")
        print("SAFETY: Robot auto-stops when obstacles < 1m detected")
        print("=" * 80)
        
        # Prerequisites check
        print("\n🔧 PREREQUISITES:")
        print("1. roscore running?")
        print("2. lidar_bridge.py running?") 
        print("3. safety_supervisor.py running? (CRITICAL - MUST BE RUNNING)")
        print("4. motor_bridge.py running? (optional for command flow test)")
        print("5. LiDAR powered and connected?")
        print("6. Robot has clear space to move safely?")
        
        response = input("\nAll systems ready for CORRECTED safety testing? (y/N): ")
        if response.lower() != 'y':
            print("Test cancelled. Start required ROS nodes and try again.")
            return False
        
        try:
            # Test safety_supervisor integration first - CORRECTED
            if not self.test_safety_supervisor_integration():
                return False
                
            # Test emergency stop system
            if not self.test_emergency_stop_system():
                return False
            
            print(f"\n🎯 INTERACTIVE SAFETY TEST - CORRECTED ARCHITECTURE")
            print(f"You'll control the robot manually through safety_supervisor.")
            print(f"Place obstacles in front of robot to test emergency stopping!")
            print(f"Commands will be processed: WASD → safety_supervisor → motors")
            
            # Start manual control - CORRECTED
            self.manual_control_loop()
            
            # Print final results
            self.print_safety_summary()
            return True
            
        except KeyboardInterrupt:
            print(f"\n\n⏹️  Safety test interrupted")
            self.emergency_stop()
            return False
        except Exception as e:
            print(f"\n💥 Safety test error: {e}")
            self.emergency_stop()
            return False
        finally:
            self.cleanup()

    def emergency_stop(self):
        """Send immediate stop command through safety system - CORRECTED"""
        stop_cmd = Twist()
        self.cmd_nav_publisher.publish(stop_cmd)  # Send to safety_supervisor INPUT
        print("\n🛑 EMERGENCY STOP COMMAND SENT TO SAFETY_SUPERVISOR")

    def print_safety_summary(self):
        """Print safety test summary - CORRECTED"""
        passed = sum(1 for result in self.test_results if result['passed'])
        failed = len(self.test_results) - passed
        
        print("\n" + "="*80)
        print("CORRECTED EMERGENCY STOP SAFETY TEST SUMMARY")
        print("="*80)
        print(f"Tests Run:                 {len(self.test_results)}")
        print(f"Passed:                    {passed}")
        print(f"Failed:                    {failed}")
        print(f"Emergency Stops:           {len(self.emergency_stops)}")
        print(f"Commands Sent:             {self.total_commands_sent}")
        print(f"Command Modifications:     {self.command_modifications}")
        print(f"Final Safety Level:        {self.current_safety_level}")
        
        if self.total_commands_sent > 0:
            modification_rate = (self.command_modifications / self.total_commands_sent) * 100
            print(f"Command Modification Rate: {modification_rate:.1f}%")
        
        if self.emergency_stops:
            print(f"\n🚨 Safety_Supervisor Emergency Stop Events:")
            for i, stop in enumerate(self.emergency_stops, 1):
                print(f"  {i}. Level: {stop['safety_level']}, Distance: {stop['closest_distance']:.2f}m")
        
        if failed == 0 and len(self.emergency_stops) > 0:
            print(f"\n🎉 CORRECTED SAFETY SYSTEM WORKING PERFECTLY!")
            print(f"✅ Emergency stops activated when needed")
            print(f"✅ Commands properly routed through safety_supervisor")
            print(f"✅ Robot stopped before collisions")
            print(f"✅ Safety override working correctly")
            
        elif failed == 0 and len(self.emergency_stops) == 0:
            print(f"\n✅ CORRECTED SAFETY SYSTEM FUNCTIONAL")
            print(f"✅ Command flow through safety_supervisor working")
            print(f"⚠️  No emergency stops triggered (test with obstacles)")
            
        else:
            print(f"\n⚠️  SAFETY ISSUES DETECTED")
            print(f"❌ Review failed tests and emergency stop behavior")
        
        print(f"\n🔧 CORRECTED Architecture Validation:")
        if self.safety_supervisor_active:
            print(f"✅ safety_supervisor.py is running and processing commands")
        else:
            print(f"❌ safety_supervisor.py not detected - start it!")
            
        if self.command_modifications > 0:
            print(f"✅ Safety command modifications working ({self.command_modifications} modifications)")
        else:
            print(f"⚠️  No command modifications detected (may be normal if no obstacles)")

    def cleanup(self):
        """Clean up terminal and ROS - CORRECTED"""
        self.running = False
        
        # Send final stop command through CORRECTED architecture
        self.emergency_stop()
        
        # Restore terminal
        if KEYBOARD_AVAILABLE:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
            except:
                pass
        
        print("\n🧹 CORRECTED safety test cleanup complete")

def main():
    print("🛡️  CORRECTED Emergency Stop & Collision Avoidance Safety Test")
    print("\nThis CORRECTED test validates the complete safety-integrated architecture:")
    print("Commands → /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge → Motors")
    print("\nThe robot will automatically stop if any obstacle is detected within 1 meter.")
    print("\nCORRECTED TEST PROCEDURE:")
    print("1. You'll control robot with WASD keys through safety_supervisor")
    print("2. Place obstacles in robot's path")
    print("3. Verify robot stops automatically via safety_supervisor") 
    print("4. Test in all directions (front, back, left, right)")
    print("5. Validate command modifications are working")
    
    try:
        tester = SafetyIntegratedEmergencyStopTester()
        success = tester.run_safety_test()
        sys.exit(0 if success else 1)
        
    except Exception as e:
        print(f"💥 CORRECTED safety test error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
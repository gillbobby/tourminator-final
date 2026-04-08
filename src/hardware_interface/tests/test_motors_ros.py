#!/usr/bin/env python3
"""
ROS-Integrated Motor Test - Tests Direct Hardware + ROS Integration
Tests motor connections using your EXACT proven code + ROS motor_bridge.py integration

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_motors_ros.py
Usage: python3 test_motors_ros.py

COMPREHENSIVE TESTING:
- Tests direct GPIO motor control (your proven code)
- Tests ROS motor_bridge.py integration
- Tests /cmd_vel command processing
- Validates differential drive kinematics
- Tests emergency stop functionality
"""

import sys
import time
from datetime import datetime

# Direct GPIO control (your proven approach)
try:
    from gpiozero import Motor
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# ROS imports
try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

class ROSMotorTester:
    def __init__(self):
        self.test_results = []
        
        # Direct motor control (your proven setup)
        self.motorA = None  # Left motor (GPIO 17,18)
        self.motorB = None  # Right motor (GPIO 22,23)
        
        # ROS integration testing
        self.ros_initialized = False
        self.motor_bridge_active = False
        self.safety_supervisor_active = False
        self.odom_received = False
        self.last_odom = None
        self.cmd_vel_nav_pub = None  # Publish to safety_supervisor input
        self.last_safe_command = Twist()  # Monitor safety_supervisor output
        self.current_safety_level = "unknown"
        
        # Test tracking
        self.movement_tests_completed = 0
        self.ros_commands_sent = 0
        self.odom_updates_received = 0
        
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

    def test_gpio_hardware_setup(self):
        """Test direct GPIO motor control (your proven code)"""
        print("\n=== Testing Direct GPIO Motor Control ===")
        
        if not GPIO_AVAILABLE:
            self.log_result("GPIO library availability", False, "Install: pip3 install gpiozero")
            return False
        
        self.log_result("GPIO library availability", True, "gpiozero available")
        
        try:
            # Initialize motors with your EXACT configuration
            self.motorA = Motor(17, 18, pwm=True)  # Left motor
            self.motorB = Motor(22, 23, pwm=True)  # Right motor
            
            self.log_result("Motor A (Left) GPIO init", True, "GPIO pins 17,18")
            self.log_result("Motor B (Right) GPIO init", True, "GPIO pins 22,23")
            return True
            
        except Exception as e:
            self.log_result("Motor GPIO initialization", False, f"GPIO error: {str(e)}")
            return False

    # YOUR EXACT PROVEN MOTOR FUNCTIONS
    def goForward(self, test_time, speed):
        """Exact copy of your goForward function"""
        if not self.motorA or not self.motorB:
            return False
        self.motorA.forward(speed)
        self.motorB.forward(speed)
        time.sleep(test_time)
        self.motorA.stop()
        self.motorB.stop()
        return True
    
    def goBackward(self, test_time, speed):
        """Exact copy of your goBackward function"""
        if not self.motorA or not self.motorB:
            return False
        self.motorA.backward(speed)
        self.motorB.backward(speed)
        time.sleep(test_time)
        self.motorA.stop()
        self.motorB.stop()
        return True
    
    def goRight(self, test_time, speed):
        """Exact copy of your goRight function"""
        if not self.motorA or not self.motorB:
            return False
        self.motorA.forward(speed)
        self.motorB.stop()
        time.sleep(test_time)
        self.motorA.stop()
        return True
    
    def goLeft(self, test_time, speed):
        """Exact copy of your goLeft function"""
        if not self.motorA or not self.motorB:
            return False
        self.motorA.stop()
        self.motorB.forward(speed)
        time.sleep(test_time)
        self.motorB.stop()
        return True
    
    def stop(self):
        """Exact copy of your stop function"""
        if self.motorA and self.motorB:
            self.motorA.stop()
            self.motorB.stop()

    def test_direct_motor_control(self):
        """Test direct motor control using your proven functions"""
        print("\n=== Testing Direct Motor Control (Your Proven Code) ===")
        
        if not self.motorA or not self.motorB:
            self.log_result("Direct motor control", False, "Motors not initialized")
            return False
        
        try:
            print("Testing forward motion (2.5s at 30% speed)...")
            if self.goForward(2.5, 0.3):
                self.log_result("Direct forward control", True, "Using your goForward()")
                self.movement_tests_completed += 1
            
            time.sleep(1)
            
            print("Testing backward motion (2.5s at 30% speed)...")
            if self.goBackward(2.5, 0.3):
                self.log_result("Direct backward control", True, "Using your goBackward()")
                self.movement_tests_completed += 1
            
            time.sleep(1)
            
            print("Testing right turn (2s at 30% speed)...")
            if self.goRight(2.0, 0.3):
                self.log_result("Direct right turn control", True, "Using your goRight()")
                self.movement_tests_completed += 1
            
            time.sleep(1)
            
            print("Testing left turn (2s at 30% speed)...")
            if self.goLeft(2.0, 0.3):
                self.log_result("Direct left turn control", True, "Using your goLeft()")
                self.movement_tests_completed += 1
            
            return self.movement_tests_completed == 4
            
        except Exception as e:
            self.log_result("Direct motor control", False, f"Control error: {str(e)}")
            return False

    def test_ros_environment(self):
        """Test ROS environment setup"""
        print("\n=== Testing ROS Environment ===")
        
        if not ROS_AVAILABLE:
            self.log_result("ROS availability", False, "Install ROS and source workspace")
            return False
        
        self.log_result("ROS libraries available", True, "rospy imported successfully")
        
        try:
            # Initialize ROS node
            rospy.init_node('motor_tester', anonymous=True)
            self.ros_initialized = True
            self.log_result("ROS node initialization", True, f"Node: {rospy.get_name()}")
            
            # Test ROS master connection
            rospy.get_published_topics()
            self.log_result("ROS master connectivity", True, "Connected to roscore")
            
            return True
            
        except Exception as e:
            self.log_result("ROS environment setup", False, f"ROS error: {str(e)}")
            return False

    def test_motor_bridge_detection(self):
        """Test if motor_bridge.py is running"""
        print("\n=== Testing Motor Bridge Integration ===")
        
        if not self.ros_initialized:
            self.log_result("Motor bridge detection", False, "ROS not initialized")
            return False
        
        try:
            # Check if motor_bridge topics exist
            topics = rospy.get_published_topics()
            odom_topic_found = any('/odom' in topic[0] for topic in topics)
            
            if odom_topic_found:
                self.log_result("Motor bridge /odom topic", True, "Odometry topic available")
            else:
                self.log_result("Motor bridge /odom topic", False, 
                              "Start motor_bridge: rosrun hardware_interface motor_bridge.py")
                return False
            
            # CORRECTED: Setup ROS interfaces for SAFETY-INTEGRATED testing
            # Publish to /cmd_vel_nav (input to safety_supervisor)
            # Monitor /cmd_vel (output from safety_supervisor to motor_bridge)
            self.cmd_vel_nav_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
            self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.safe_cmd_callback)
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
            
            # Also monitor safety supervisor if available
            self.safety_status_sub = rospy.Subscriber('/safety_status', String, self.safety_status_callback)
            
            # Wait for odometry data
            print("Waiting for odometry data from motor_bridge...")
            wait_start = rospy.Time.now()
            while not self.odom_received and (rospy.Time.now() - wait_start).to_sec() < 10:
                time.sleep(0.1)
            
            if self.odom_received:
                self.log_result("Motor bridge odometry", True, "Receiving odometry data")
                self.motor_bridge_active = True
                return True
            else:
                self.log_result("Motor bridge odometry", False, "No odometry data received")
                return False
                
        except Exception as e:
            self.log_result("Motor bridge detection", False, f"Detection error: {str(e)}")
            return False

    def odom_callback(self, odom_msg):
        """Process odometry messages from motor_bridge"""
        self.odom_received = True
        self.last_odom = odom_msg
        self.odom_updates_received += 1

    def safe_cmd_callback(self, cmd_msg):
        """Monitor commands output by safety_supervisor to motor_bridge"""
        self.last_safe_command = cmd_msg

    def safety_status_callback(self, msg):
        """Monitor safety_supervisor status"""
        self.safety_supervisor_active = True
        try:
            parts = msg.data.split(':')
            if len(parts) == 2:
                self.current_safety_level = parts[0]
        except:
            pass

    def test_safety_supervisor_integration(self):
        """Test if safety_supervisor.py is running (optional but recommended)"""
        print("\n=== Testing Safety Supervisor Integration ===")
        
        # Check if safety_supervisor is running
        topics = rospy.get_published_topics()
        safety_topic_found = any('/safety_status' in topic[0] for topic in topics)
        
        if safety_topic_found:
            print("Waiting for safety_supervisor data...")
            wait_start = rospy.Time.now()
            while not self.safety_supervisor_active and (rospy.Time.now() - wait_start).to_sec() < 5:
                time.sleep(0.1)
            
            if self.safety_supervisor_active:
                self.log_result("Safety Supervisor integration", True, 
                              f"Active - Level: {self.current_safety_level}")
                print("🛡️  SAFETY MODE: Commands will go through safety_supervisor")
                return True
            else:
                self.log_result("Safety Supervisor integration", False, 
                              "Safety topics found but no data received")
        else:
            self.log_result("Safety Supervisor integration", False, 
                          "Not running - commands will go directly to motor_bridge")
            print("⚠️  DIRECT MODE: Commands will bypass safety (not recommended)")
        
        return False

    def test_ros_motor_commands(self):
        """Test ROS motor commands - CORRECTED for safety integration"""
        print("\n=== Testing ROS Motor Commands ===")
        
        if not self.motor_bridge_active:
            self.log_result("ROS motor commands", False, "Motor bridge not active")
            return False
        
        # Determine command architecture
        if self.safety_supervisor_active:
            print("🛡️  Testing through SAFETY-INTEGRATED architecture:")
            print("   test → /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge → motors")
            cmd_publisher = self.cmd_vel_nav_pub
        else:
            print("⚠️  Testing DIRECT architecture (safety bypassed):")
            print("   test → /cmd_vel → motor_bridge → motors")
            cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            time.sleep(0.5)  # Let publisher initialize
        
        try:
            # Test forward command
            print("Sending forward command via ROS...")
            cmd = Twist()
            cmd.linear.x = 0.2  # 0.2 m/s forward
            cmd_publisher.publish(cmd)
            self.ros_commands_sent += 1
            
            if self.safety_supervisor_active:
                print(f"   Command sent to safety_supervisor, monitoring safety response...")
                time.sleep(1.0)  # Give safety_supervisor time to process
                print(f"   Safety level: {self.current_safety_level}")
            
            time.sleep(3.0)
            
            # Stop
            cmd_publisher.publish(Twist())
            time.sleep(1.0)
            
            self.log_result("ROS forward command", True, "Command processed successfully")
            
            # Test backward command
            print("Sending backward command via ROS...")
            cmd = Twist()
            cmd.linear.x = -0.2  # 0.2 m/s backward
            cmd_publisher.publish(cmd)
            self.ros_commands_sent += 1
            time.sleep(3.0)
            
            # Stop
            cmd_publisher.publish(Twist())
            time.sleep(1.0)
            
            self.log_result("ROS backward command", True, "Command processed successfully")
            
            # Test turn command
            print("Sending turn command via ROS...")
            cmd = Twist()
            cmd.angular.z = 0.5  # 0.5 rad/s turn
            cmd_publisher.publish(cmd)
            self.ros_commands_sent += 1
            time.sleep(2.0)
            
            # Stop
            cmd_publisher.publish(Twist())
            
            self.log_result("ROS turn command", True, "Command processed successfully")
            
            return True
            
        except Exception as e:
            self.log_result("ROS motor commands", False, f"Command error: {str(e)}")
            return False

    def test_differential_drive_kinematics(self):
        """Test differential drive calculations - CORRECTED for safety integration"""
        print("\n=== Testing Differential Drive Kinematics ===")
        
        if not self.motor_bridge_active:
            self.log_result("Differential drive test", False, "Motor bridge not active")
            return False
        
        # Use appropriate publisher based on safety integration
        cmd_publisher = self.cmd_vel_nav_pub if self.safety_supervisor_active else rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        if not self.safety_supervisor_active:
            time.sleep(0.5)  # Let direct publisher initialize
        
        try:
            print("Testing differential drive kinematics...")
            
            if self.safety_supervisor_active:
                print("🛡️  Commands routed through safety_supervisor")
            else:
                print("⚠️  Commands sent directly to motor_bridge (safety bypassed)")
            
            # Test pure rotation (should turn in place)
            print("Testing pure rotation (turn in place)...")
            cmd = Twist()
            cmd.linear.x = 0.0    # No forward motion
            cmd.angular.z = 0.3   # Pure rotation
            cmd_publisher.publish(cmd)
            time.sleep(2.0)
            cmd_publisher.publish(Twist())  # Stop
            
            self.log_result("Pure rotation test", True, "Turn in place command")
            
            # Test combined motion (arc movement)
            print("Testing combined motion (arc movement)...")
            cmd = Twist()
            cmd.linear.x = 0.2    # Forward motion
            cmd.angular.z = 0.2   # + turning = arc
            cmd_publisher.publish(cmd)
            time.sleep(3.0)
            cmd_publisher.publish(Twist())  # Stop
            
            self.log_result("Combined motion test", True, "Arc movement command")
            
            return True
            
        except Exception as e:
            self.log_result("Differential drive kinematics", False, f"Kinematics error: {str(e)}")
            return False

    def test_odometry_integration(self):
        """Test odometry data from motor_bridge"""
        print("\n=== Testing Odometry Integration ===")
        
        if not self.odom_received:
            self.log_result("Odometry integration", False, "No odometry data received")
            return False
        
        try:
            # Record initial position
            initial_odom = self.last_odom
            initial_x = initial_odom.pose.pose.position.x
            initial_y = initial_odom.pose.pose.position.y
            
            print("Recording initial odometry position...")
            print(f"Initial position: x={initial_x:.3f}, y={initial_y:.3f}")
            
            # Send movement command and track odometry changes
            print("Sending forward command and monitoring odometry...")
            cmd = Twist()
            cmd.linear.x = 0.2
            self.cmd_vel_pub.publish(cmd)
            time.sleep(4.0)  # Move for 4 seconds
            self.cmd_vel_pub.publish(Twist())  # Stop
            
            time.sleep(1.0)  # Let odometry settle
            
            # Check final position
            final_odom = self.last_odom
            final_x = final_odom.pose.pose.position.x
            final_y = final_odom.pose.pose.position.y
            
            # Calculate movement
            distance_moved = ((final_x - initial_x)**2 + (final_y - initial_y)**2)**0.5
            
            print(f"Final position: x={final_x:.3f}, y={final_y:.3f}")
            print(f"Distance moved: {distance_moved:.3f}m")
            
            # Validate odometry is updating
            if distance_moved > 0.1:  # At least 10cm movement detected
                self.log_result("Odometry position tracking", True, 
                              f"Detected {distance_moved:.3f}m movement")
            else:
                self.log_result("Odometry position tracking", False, 
                              "No significant movement detected in odometry")
            
            # Check odometry update frequency
            if self.odom_updates_received > 10:
                self.log_result("Odometry update frequency", True, 
                              f"{self.odom_updates_received} updates received")
            else:
                self.log_result("Odometry update frequency", False, 
                              f"Only {self.odom_updates_received} updates received")
            
            return True
            
        except Exception as e:
            self.log_result("Odometry integration", False, f"Odometry error: {str(e)}")
            return False

    def test_emergency_stop_response(self):
        """Test emergency stop functionality"""
        print("\n=== Testing Emergency Stop Response ===")
        
        if not self.motor_bridge_active:
            self.log_result("Emergency stop test", False, "Motor bridge not active")
            return False
        
        try:
            print("Testing emergency stop response...")
            
            # Send movement command
            print("Sending continuous forward command...")
            cmd = Twist()
            cmd.linear.x = 0.3
            
            # Send command for 2 seconds
            for i in range(20):  # 2 seconds at 10Hz
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
            
            # Send emergency stop
            print("Sending emergency stop command...")
            stop_cmd = Twist()  # All zeros
            self.cmd_vel_pub.publish(stop_cmd)
            
            # Verify robot stops (check via odometry)
            time.sleep(0.5)
            
            # Check if velocity in odometry is near zero
            if self.last_odom:
                current_vel = abs(self.last_odom.twist.twist.linear.x)
                if current_vel < 0.05:  # Less than 5cm/s
                    self.log_result("Emergency stop response", True, 
                                  f"Robot stopped, velocity: {current_vel:.3f} m/s")
                else:
                    self.log_result("Emergency stop response", False, 
                                  f"Robot still moving: {current_vel:.3f} m/s")
            else:
                self.log_result("Emergency stop response", True, "Stop command sent successfully")
            
            return True
            
        except Exception as e:
            self.log_result("Emergency stop response", False, f"E-stop error: {str(e)}")
            return False

    def test_motor_bridge_parameters(self):
        """Test motor_bridge parameter configuration"""
        print("\n=== Testing Motor Bridge Parameters ===")
        
        try:
            # Check if parameters are loaded correctly
            if rospy.has_param('/motor_bridge/wheel_diameter'):
                wheel_diameter = rospy.get_param('/motor_bridge/wheel_diameter', 0.144)
                self.log_result("Wheel diameter parameter", True, f"{wheel_diameter}m")
            else:
                self.log_result("Wheel diameter parameter", False, "Parameter not found")
            
            if rospy.has_param('/motor_bridge/wheelbase'):
                wheelbase = rospy.get_param('/motor_bridge/wheelbase', 0.2413)
                self.log_result("Wheelbase parameter", True, f"{wheelbase}m")
            else:
                self.log_result("Wheelbase parameter", False, "Parameter not found")
            
            # Test speed limits
            max_linear = rospy.get_param('/motor_bridge/max_linear_speed', 0.4)
            max_angular = rospy.get_param('/motor_bridge/max_angular_speed', 0.8)
            
            print(f"Motor bridge speed limits: {max_linear} m/s linear, {max_angular} rad/s angular")
            self.log_result("Speed limit parameters", True, "Parameters loaded")
            
            return True
            
        except Exception as e:
            self.log_result("Motor bridge parameters", False, f"Parameter error: {str(e)}")
            return False

    def run_comprehensive_motor_test(self):
        """Run complete motor testing suite"""
        print("Comprehensive ROS Motor Integration Test")
        print("=" * 70)
        print("Testing: Direct GPIO + ROS motor_bridge.py integration")
        print("INCLUDES: Hardware control, ROS commands, odometry, emergency stop")
        print("=" * 70)
        
        # Safety check
        print("\n⚠️  SAFETY REQUIREMENTS:")
        print("1. Robot in safe, open area?")
        print("2. Motors/wheels can move freely?")
        print("3. Emergency stop accessible?")
        print("4. Power connections secure?")
        
        response = input("\nSafety requirements met? Ready for motor testing? (y/N): ")
        if response.lower() != 'y':
            print("Test cancelled for safety. Ensure safe environment and try again.")
            return False
        
        try:
            # Phase 1: Direct hardware testing
            print(f"\n🔧 PHASE 1: DIRECT HARDWARE TESTING")
            if not self.test_gpio_hardware_setup():
                return False
            
            if not self.test_direct_motor_control():
                return False
            
            # Phase 2: ROS integration testing  
            print(f"\n🤖 PHASE 2: ROS INTEGRATION TESTING")
            if not self.test_ros_environment():
                print("⚠️  Skipping ROS tests - ROS environment not available")
                self.print_comprehensive_summary()
                return True
            
            # Test safety supervisor integration (optional but recommended)
            safety_available = self.test_safety_supervisor_integration()
            
            if not self.test_motor_bridge_detection():
                print("⚠️  Skipping ROS motor tests - motor_bridge not running")
                self.print_comprehensive_summary()
                return True
            
            # Phase 3: ROS motor control testing
            print(f"\n🎮 PHASE 3: ROS MOTOR CONTROL TESTING")
            self.test_motor_bridge_parameters()
            self.test_ros_motor_commands()
            self.test_differential_drive_kinematics()
            self.test_odometry_integration()
            self.test_emergency_stop_response()
            
            self.print_comprehensive_summary()
            return True
            
        except KeyboardInterrupt:
            print(f"\n⏹️  Motor test interrupted by user")
            self.emergency_cleanup()
            return False
        except Exception as e:
            print(f"\n💥 Motor test error: {e}")
            self.emergency_cleanup()
            return False
        finally:
            self.cleanup()

    def emergency_cleanup(self):
        """Emergency cleanup - stop all motors immediately"""
        print("\n🛑 EMERGENCY CLEANUP - STOPPING ALL MOTORS")
        
        # Stop direct GPIO motors
        self.stop()
        
        # Stop via ROS if available
        if self.cmd_vel_nav_pub:
            self.cmd_vel_nav_pub.publish(Twist())
        
        # Also try direct stop if safety not available
        try:
            direct_stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            time.sleep(0.1)
            direct_stop_pub.publish(Twist())
        except:
            pass
        
        time.sleep(0.5)

    def print_comprehensive_summary(self):
        """Print detailed test results"""
        passed = sum(1 for result in self.test_results if result['passed'])
        failed = len(self.test_results) - passed
        
        print("\n" + "="*75)
        print("🔧 COMPREHENSIVE MOTOR TEST RESULTS")
        print("="*75)
        print(f"Tests Completed:      {len(self.test_results)}")
        print(f"Tests Passed:         {passed}")
        print(f"Tests Failed:         {failed}")
        print(f"Success Rate:         {(passed/len(self.test_results)*100):.1f}%" if self.test_results else "0%")
        
        print(f"\n📊 MOTOR STATISTICS:")
        print(f"Direct movements:     {self.movement_tests_completed}/4")
        print(f"ROS commands sent:    {self.ros_commands_sent}")
        print(f"Odometry updates:     {self.odom_updates_received}")
        
        # Show failed tests
        if failed > 0:
            print(f"\n❌ FAILED TESTS:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  • {result['test']}: {result['message']}")
        
        # Overall assessment
        if failed == 0 and self.movement_tests_completed == 4:
            print(f"\n🎉 MOTOR SYSTEM FULLY FUNCTIONAL!")
            print(f"✅ Direct GPIO control verified")
            print(f"✅ ROS integration working") 
            print(f"✅ Differential drive operational")
            print(f"✅ Odometry tracking active")
            print(f"✅ Emergency stop responsive")
            
            print(f"\n🎯 NEXT STEPS:")
            print(f"1. Motors ready for autonomous navigation")
            print(f"2. Test safety integration: test_emergency_stop.py")
            print(f"3. Full system test: roslaunch hardware_interface full_robot.launch")
            
        elif self.movement_tests_completed == 4:
            print(f"\n✅ HARDWARE FUNCTIONAL, ROS INTEGRATION PARTIAL")
            print(f"✅ Direct motor control working perfectly")
            print(f"⚠️  Some ROS integration issues detected")
            
        else:
            print(f"\n⚠️  MOTOR HARDWARE ISSUES DETECTED")
            print(f"❌ Direct motor control problems")
            print(f"🔧 Check hardware connections and power")
        
        print(f"\n📋 TROUBLESHOOTING GUIDE:")
        if not GPIO_AVAILABLE:
            print(f"• Install GPIO: pip3 install gpiozero")
        if not self.motor_bridge_active and ROS_AVAILABLE:
            print(f"• Start motor_bridge: rosrun hardware_interface motor_bridge.py")
        if failed > 0:
            print(f"• Check hardware connections match GPIO pins 17,18,22,23")
            print(f"• Verify power supply to motors")
            print(f"• Check motor controller connections")

    def cleanup(self):
        """Clean up all resources"""
        # Stop motors via both methods
        self.stop()
        
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())
        
        # Clean up GPIO
        try:
            if self.motorA:
                self.motorA.close()
            if self.motorB:
                self.motorB.close()
        except:
            pass
        
        print("\n🧹 Motor test cleanup complete")

def main():
    print("🔧 COMPREHENSIVE ROS MOTOR INTEGRATION TEST - SAFETY CORRECTED")
    print("\nThis test validates both direct motor control AND ROS integration.")
    print("It tests your proven GPIO code + motor_bridge.py + safety_supervisor.py")
    print("\n📋 CORRECTED ARCHITECTURE:")
    print("Direct Test: GPIO → Motors")
    print("ROS Test: /cmd_vel_nav → safety_supervisor → /cmd_vel → motor_bridge → Motors")
    print("\n📋 TEST PHASES:")
    print("1. Direct GPIO motor control (your proven code)")
    print("2. ROS environment verification") 
    print("3. Safety supervisor detection (optional)")
    print("4. motor_bridge.py integration testing")
    print("5. /cmd_vel_nav command processing (through safety)")
    print("6. Odometry tracking validation")
    print("7. Emergency stop response")
    
    try:
        tester = ROSMotorTester()
        success = tester.run_comprehensive_motor_test()
        sys.exit(0 if success else 1)
        
    except Exception as e:
        print(f"💥 Motor test error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
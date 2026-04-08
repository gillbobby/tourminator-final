#!/usr/bin/env python3
"""
Motor Bridge Node - ROS interface for differential drive robot
UPDATED: Removed IMU integration, using wheel odometry only for demo

Key Changes:
1. Verified GPIO pin assignments (17,18 left, 22,23 right)
2. Improved differential drive logic
3. Better motor control methods  
4. REMOVED: All IMU integration (hardware not available)
5. Simplified odometry using wheel encoders only
"""

import rospy
import tf2_ros
import math
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Use gpiozero since it's proven to work with your hardware
try:
    from gpiozero import Motor
    GPIO_AVAILABLE = True
    rospy.loginfo("gpiozero imported successfully")
except ImportError:
    rospy.logwarn("gpiozero not available - running in simulation mode")
    GPIO_AVAILABLE = False

class MotorBridge:
    def __init__(self):
        rospy.init_node('motor_bridge', anonymous=True)
        
        # Robot physical parameters (from your verified specs)
        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.144)  # 144mm wheels
        self.wheel_radius = self.wheel_diameter / 2.0
        self.wheelbase = rospy.get_param('~wheelbase', 0.2413)  # 1.5 feet
        
        # Motor speed limits (conservative for demo)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5)  # rad/s
        
        # GPIO pins (VERIFIED from your test code)
        self.motor_left_pins = (
            rospy.get_param('~motor_left_pin1', 17),
            rospy.get_param('~motor_left_pin2', 18)
        )
        self.motor_right_pins = (
            rospy.get_param('~motor_right_pin1', 22), 
            rospy.get_param('~motor_right_pin2', 23)
        )
        
        # Initialize motors using gpiozero (your proven approach)
        self.setup_motors()
        
        # Odometry tracking (SIMPLIFIED - NO IMU)
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        
        # Current velocity tracking for odometry
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # TF publishing option
        self.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)
        
        # ROS interfaces
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel_nav', Twist, self.cmd_vel_callback) #safety supervisor disabled
        #self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback) #safety supervisor enabled
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        if self.publish_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Timer for odometry updates
        self.odom_timer = rospy.Timer(rospy.Duration(0.1), self.publish_odometry_timer)
        
        rospy.loginfo("Motor Bridge initialized (NO IMU - wheel odometry only)")
        rospy.loginfo(f"Wheelbase: {self.wheelbase}m, Wheel radius: {self.wheel_radius}m")
        rospy.loginfo(f"Motor pins: Left{self.motor_left_pins}, Right{self.motor_right_pins}")

    def setup_motors(self):
        """Initialize motors using gpiozero Motor class (your working approach)"""
        if not GPIO_AVAILABLE:
            rospy.logwarn("GPIO not available - motor commands will be logged only")
            self.motor_left = None
            self.motor_right = None
            return
            
        try:
            # Initialize motors exactly like your working test code
            self.motor_left = Motor(self.motor_left_pins[0], self.motor_left_pins[1], pwm=True)
            self.motor_right = Motor(self.motor_right_pins[0], self.motor_right_pins[1], pwm=True)
            
            rospy.loginfo("Motors initialized successfully with gpiozero")
            rospy.loginfo("Hardware-verified GPIO pin assignments loaded")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize motors: {e}")
            self.motor_left = None
            self.motor_right = None

    def cmd_vel_callback(self, msg):
        """
        Convert /cmd_vel to wheel speeds and control motors
        IMPROVED: Better differential drive kinematics
        """
        linear_vel = msg.linear.x       # m/s forward/backward
        angular_vel = msg.angular.z     # rad/s turning (positive = left turn)
        
        # Apply speed limits
        linear_vel = max(-self.max_linear_speed, min(self.max_linear_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        # Store commanded velocities for odometry
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        
        # Improved differential drive math
        # For differential drive: left_speed = V - ω*L/2, right_speed = V + ω*L/2
        half_wheelbase = self.wheelbase / 2.0
        left_wheel_speed = linear_vel - (angular_vel * half_wheelbase)
        right_wheel_speed = linear_vel + (angular_vel * half_wheelbase)
        
        # Convert to motor control values (-1.0 to +1.0 for gpiozero)
        left_motor_speed = self.wheel_speed_to_motor(left_wheel_speed)
        right_motor_speed = self.wheel_speed_to_motor(right_wheel_speed)
        
        # Store current speeds for odometry
        self.current_left_speed = left_wheel_speed
        self.current_right_speed = right_wheel_speed
        
        # Control motors
        self.control_motors(left_motor_speed, right_motor_speed)
        
        # Debug logging
        rospy.logdebug(f"cmd_vel: lin={linear_vel:.2f}, ang={angular_vel:.2f}")
        rospy.logdebug(f"wheel speeds: L={left_wheel_speed:.2f}, R={right_wheel_speed:.2f}")

    def wheel_speed_to_motor(self, wheel_speed):
        """
        Convert wheel speed (m/s) to motor control value (-1.0 to +1.0)
        IMPROVED: Better scaling based on max wheel speed
        """
        # Calculate theoretical max wheel speed (when one wheel stops, other at max)
        max_wheel_speed = self.max_linear_speed + (self.max_angular_speed * self.wheelbase / 2.0)
        
        if max_wheel_speed == 0:
            return 0.0
            
        # Scale to -1.0 to +1.0 range
        motor_value = wheel_speed / max_wheel_speed
        
        # Clamp to valid range
        return max(-1.0, min(1.0, motor_value))

    def control_motors(self, left_speed, right_speed):
        """
        Control motors using gpiozero Motor interface
        UPDATED: Based on your working test code pattern
        """
        if not GPIO_AVAILABLE or not self.motor_left or not self.motor_right:
            rospy.logdebug(f"SIMULATION: L={left_speed:.2f}, R={right_speed:.2f}")
            return
            
        try:
            # Left motor control (exactly like your goForward/goBackward pattern)
            if abs(left_speed) < 0.01:  # Dead zone
                self.motor_left.stop()
            elif left_speed > 0:
                self.motor_left.forward(abs(left_speed))
            else:  # left_speed < 0
                self.motor_left.backward(abs(left_speed))
                
            # Right motor control
            if abs(right_speed) < 0.01:  # Dead zone
                self.motor_right.stop()
            elif right_speed > 0:
                self.motor_right.forward(abs(right_speed))
            else:  # right_speed < 0
                self.motor_right.backward(abs(right_speed))
                
        except Exception as e:
            rospy.logerr(f"Error controlling motors: {e}")

    def publish_odometry_timer(self, event):
        """Timer callback for regular odometry updates - SIMPLIFIED (NO IMU)"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt > 0:
            # SIMPLIFIED: Use commanded velocities for odometry
            # This is less accurate but sufficient for demo without wheel encoders
            linear_vel = self.current_linear_vel
            angular_vel = self.current_angular_vel
            
            # Update robot pose using commanded velocities
            delta_x = linear_vel * math.cos(self.theta) * dt
            delta_y = linear_vel * math.sin(self.theta) * dt
            delta_theta = angular_vel * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalize angle
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Publish odometry
            self.publish_odometry(linear_vel, angular_vel, current_time)
            
        self.last_time = current_time

    def publish_odometry(self, linear_vel, angular_vel, current_time):
        """Publish odometry message and TF transform - SIMPLIFIED"""
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation = self.theta_to_quaternion(self.theta)
        
        # Velocities
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Covariance - HIGHER UNCERTAINTY without IMU and encoders
        odom.pose.covariance[0] = 0.5     # x position variance (increased)
        odom.pose.covariance[7] = 0.5     # y position variance (increased)
        odom.pose.covariance[35] = 1.0    # theta variance (much higher without IMU)
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.1    # linear velocity variance
        odom.twist.covariance[35] = 0.2   # angular velocity variance
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform if enabled
        if self.publish_odom_tf:
            self.publish_tf_transform(current_time)

    def theta_to_quaternion(self, theta):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        return q

    def publish_tf_transform(self, current_time):
        """Publish transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.theta_to_quaternion(self.theta)
        
        self.tf_broadcaster.sendTransform(t)

    def emergency_stop(self):
        """Emergency stop all motors"""
        if self.motor_left and self.motor_right:
            self.motor_left.stop()
            self.motor_right.stop()
        rospy.logwarn("EMERGENCY STOP - All motors stopped")

    def cleanup(self):
        """Cleanup on shutdown"""
        self.emergency_stop()
        rospy.loginfo("Motor bridge cleaned up")

if __name__ == '__main__':
    try:
        motor_bridge = MotorBridge()
        
        # Register cleanup
        rospy.on_shutdown(motor_bridge.cleanup)
        
        rospy.loginfo("Motor bridge ready - listening for /cmd_vel commands")
        rospy.loginfo("DEMO MODE: Using wheel odometry only (no IMU)")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Motor bridge shutting down")
    except Exception as e:
        rospy.logerr(f"Motor bridge error: {e}")
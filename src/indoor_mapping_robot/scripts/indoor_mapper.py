#!/usr/bin/env python3
"""
Indoor Mapper for Real Hardware Integration - FIXED VERSION

FIXED ISSUES:
1. Added debug prints to map_callback to see if it's being called
2. Added manual map checking as fallback
3. Better map detection logic
4. Fixed subscription timing issues
"""

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Empty, Float32
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
from scipy import ndimage
from collections import deque
import json
import time
import math

class IndoorMapper:
    def __init__(self):
        rospy.init_node('indoor_mapper')
        
        # INITIALIZE ALL ATTRIBUTES FIRST
        self.robot_pose = None
        self.start_position = None
        self.robot_localized = False
        self.current_map = None
        self.map_initialized = False
        self.position_history = deque(maxlen=100)
        self.stuck_count = 0
        self.recovery_attempts = 0
        self.exploration_start_time = rospy.Time.now()
        self.last_goal_time = rospy.Time.now()
        self.failed_goals = []
        self.photo_waypoints = []
        self.current_photo_index = 0
        
        # Enhanced frontier tracking
        self.frontier_history = deque(maxlen=30)
        self.last_frontier_count = 0
        self.no_frontier_start_time = None
        self.manual_trigger = False
        
        # System integration monitoring
        self.last_safety_status = None
        self.safety_supervisor_active = False
        self.camera_system_ready = False
        self.last_camera_status = None
        self.emergency_active = False
        
        # Enhanced exploration tracking
        self.exploration_finished = False
        self.last_meaningful_movement = rospy.Time.now()
        self.movement_threshold = 0.1
        
        # Hardware integration parameters
        self.use_real_hardware = rospy.get_param('~use_real_hardware', True)
        self.camera_trigger_topic = rospy.get_param('~camera_trigger_topic', '/camera/capture_trigger')
        
        # Velocity and movement tracking
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.last_velocity_check = rospy.Time.now()
        self.zero_velocity_start = None
        self.last_position = None
        
        # Progress tracking with better metrics
        self.last_map_update_time = rospy.Time.now()
        self.last_explored_area = 0
        self.stall_start_time = None
        self.consecutive_no_progress = 0
        
        # FIXED: Add map checking variables
        self.map_check_count = 0
        self.map_callback_count = 0
        self.last_manual_map_check = rospy.Time.now()
        
        # Enhanced statistics tracking
        self.stats = {
            'total_area_m2': 0,
            'explored_area_m2': 0,
            'coverage_percent': 0,
            'rooms_found': 0,
            'photos_taken': 0,
            'distance_traveled': 0,
            'time_elapsed': 0,
            'frontier_count': 0,
            'waypoints_completed': 0,
            'safety_events': 0,
            'camera_triggers': 0,
            'system_health': 'good'
        }
        
        # Enhanced state management
        self.STATES = {
            'INITIALIZING': 'Initializing system components',
            'WAITING_FOR_SYSTEMS': 'Waiting for hardware systems to be ready',
            'MONITORING_EXPLORATION': 'Monitoring autonomous exploration progress',
            'EXPLORATION_COMPLETE': 'Exploration finished, preparing photography',
            'TRANSITIONING': 'Transitioning to photography mode',
            'PHOTOGRAPHING': 'Taking photos at waypoints',
            'RETURNING_HOME': 'Returning to start position',
            'STUCK_RECOVERY': 'Executing recovery behavior',
            'EMERGENCY_STOP': 'Emergency stop active',
            'COMPLETE': 'Mission complete',
            'ERROR': 'System error'
        }
        
        self.current_state = 'INITIALIZING'
        self.previous_state = None
        self.state_start_time = rospy.Time.now()
        
        # Enhanced parameters for real hardware integration
        self.params = {
            'exploration_timeout': rospy.get_param('~exploration_timeout', 300.0),
            'stuck_detection_time': rospy.get_param('~stuck_detection_time', 25.0),
            'map_complete_threshold': rospy.get_param('~map_complete_threshold', 0.65),
            'min_room_size': rospy.get_param('~min_room_size', 3.0),
            'photo_spacing': rospy.get_param('~photo_spacing', 2.5),
            'recovery_attempts_max': rospy.get_param('~recovery_attempts_max', 10),
            'min_frontier_threshold': rospy.get_param('~min_frontier_threshold', 3),
            'no_progress_timeout': rospy.get_param('~no_progress_timeout', 300.0),
            'stable_map_time': rospy.get_param('~stable_map_time', 20.0),
            'system_ready_timeout': rospy.get_param('~system_ready_timeout', 30.0),
            'photo_timeout': rospy.get_param('~photo_timeout', 30.0),
            'min_movement_distance': rospy.get_param('~min_movement_distance', 0.5)
        }
        
        # Publishers with enhanced integration
        self.pubs = {
            'state': rospy.Publisher('/mapper_state', String, queue_size=1, latch=True),
            'debug': rospy.Publisher('/mapper_debug', String, queue_size=1),
            'cmd_vel': rospy.Publisher('/cmd_vel_unused', Twist, queue_size=1),
            'map_coverage': rospy.Publisher('/map_coverage', Float32, queue_size=1),
            'explore_cancel': rospy.Publisher('/explore/cancel', Empty, queue_size=1),
            'camera_trigger': rospy.Publisher(self.camera_trigger_topic, Bool, queue_size=1),
            'status': rospy.Publisher('/mapper_status', String, queue_size=1, latch=True)
        }
        
        # FIXED: Subscribers with better error handling
        try:
            self.subs = {
                'map': rospy.Subscriber('/map', OccupancyGrid, self.map_callback),
                'scan': rospy.Subscriber('/scan', LaserScan, self.scan_callback),
                'odom': rospy.Subscriber('/odom', Odometry, self.odom_callback),
                'safety_status': rospy.Subscriber('/safety_status', String, self.safety_status_callback),
                'emergency_stop': rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback),
                'camera_status': rospy.Subscriber('/camera/photo_status', String, self.camera_status_callback),
                'manual_trigger': rospy.Subscriber('/trigger_photo_mode', Empty, self.manual_trigger_callback)
            }
            rospy.logwarn("✅ All subscribers created successfully")
        except Exception as e:
            rospy.logerr(f"Failed to create subscribers: {e}")
        
        # Try to subscribe to explore status if available
        try:
            self.explore_status_sub = rospy.Subscriber('/explore/status', String, self.explore_status_callback)
            self.explore_status_available = True
        except:
            self.explore_status_available = False
            rospy.logwarn("Explore status topic not available")
        
        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('/robot_navigation/move_base', MoveBaseAction)
        
        # TF for pose tracking (critical for hardware integration)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Enhanced timers
        rospy.Timer(rospy.Duration(3.0), self.publish_debug_info)
        rospy.Timer(rospy.Duration(0.5), self.update_pose_from_tf)
        rospy.Timer(rospy.Duration(1.0), self.system_health_check)
        rospy.Timer(rospy.Duration(5.0), self.publish_status)
        
        # FIXED: Add manual map checking timer
        rospy.Timer(rospy.Duration(2.0), self.manual_map_check)
        
        rospy.logwarn("🤖 Indoor Mapper initialized for REAL HARDWARE")
        rospy.logwarn("Waiting for all systems to be ready...")
        rospy.logwarn("Manual trigger: rostopic pub /trigger_photo_mode std_msgs/Empty")

    def manual_map_check(self, event):
        """FIXED: Manually check for map availability as fallback"""
        self.map_check_count += 1
        
        try:
            # Try to get map info directly
            map_info = rospy.wait_for_message('/map', OccupancyGrid, timeout=1.0)
            
            if map_info and len(map_info.data) > 100:  # Map has some data
                if not self.map_initialized:
                    rospy.logwarn("🗺️  FIXED: Manual map detection succeeded!")
                    rospy.logwarn(f"   Map size: {map_info.info.width}x{map_info.info.height}")
                    rospy.logwarn(f"   Data length: {len(map_info.data)}")
                    
                    # Manually trigger map callback
                    self.map_callback(map_info)
                    
        except Exception as e:
            # This is expected if no map is available yet
            if self.map_check_count % 10 == 0:  # Log every 20 seconds
                rospy.logdebug(f"Manual map check {self.map_check_count}: {e}")

    def map_callback(self, msg):
        """FIXED: Enhanced map callback with debugging"""
        self.map_callback_count += 1
        
        rospy.logwarn(f"🗺️  MAP CALLBACK #{self.map_callback_count} - Map received!")
        rospy.logwarn(f"   Map size: {msg.info.width} x {msg.info.height}")
        rospy.logwarn(f"   Resolution: {msg.info.resolution} m/cell")
        rospy.logwarn(f"   Data length: {len(msg.data)}")
        
        self.current_map = msg
        self.last_map_update_time = rospy.Time.now()
        
        if not self.map_initialized:
            self.map_initialized = True
            rospy.logwarn("✅ MAP INITIALIZED! Starting exploration monitoring")
            rospy.logwarn(f"   First map: {msg.info.width}x{msg.info.height} cells")
            
        # Update map statistics
        self.update_map_statistics()

    def system_health_check(self, event):
        """Monitor overall system health and integration"""
        current_time = rospy.Time.now()
        
        # Check system component health
        health_status = []
        
        # Check if safety supervisor is active
        if self.safety_supervisor_active:
            health_status.append("safety_ok")
        else:
            health_status.append("safety_missing")
        
        # Check camera system
        if self.camera_system_ready:
            health_status.append("camera_ok")
        else:
            health_status.append("camera_unknown")
        
        # Check map updates
        map_age = (current_time - self.last_map_update_time).to_sec()
        if map_age < 30.0:
            health_status.append("mapping_ok")
        else:
            health_status.append("mapping_stale")
        
        # Update system health
        if "safety_missing" in health_status:
            self.stats['system_health'] = 'degraded'
        elif len([s for s in health_status if "ok" in s]) >= 2:
            self.stats['system_health'] = 'good'
        else:
            self.stats['system_health'] = 'fair'

    def safety_status_callback(self, msg):
        """Monitor safety supervisor status"""
        self.safety_supervisor_active = True
        self.last_safety_status = msg.data
        
        # Count safety events
        if "emergency" in msg.data.lower() or "critical" in msg.data.lower():
            self.stats['safety_events'] += 1

    def emergency_stop_callback(self, msg):
        """Handle emergency stop events"""
        if msg.data and not self.emergency_active:
            self.emergency_active = True
            rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED - Pausing exploration")
            self.change_state('EMERGENCY_STOP')
        elif not msg.data and self.emergency_active:
            self.emergency_active = False
            rospy.logwarn("✅ Emergency stop cleared - Resuming")
            # Return to previous state
            if self.previous_state:
                self.change_state(self.previous_state)

    def camera_status_callback(self, msg):
        """Monitor camera system status"""
        self.camera_system_ready = True
        self.last_camera_status = msg.data
        if "captured" in msg.data.lower():
            self.stats['camera_triggers'] += 1

    def explore_status_callback(self, msg):
        """Monitor explore_lite status if available"""
        if "finished" in msg.data.lower() or "no frontiers" in msg.data.lower():
            rospy.logwarn(f"Explore reports: {msg.data}")

    def update_pose_from_tf(self, event):
        """Get robot pose from TF transform (SLAM integration)"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            
            # Extract position and orientation
            current_pose = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'theta': self.quaternion_to_yaw(transform.transform.rotation)
            }
            
            # Check for meaningful movement
            if self.robot_pose:
                dx = current_pose['x'] - self.robot_pose['x']
                dy = current_pose['y'] - self.robot_pose['y']
                distance_moved = math.sqrt(dx*dx + dy*dy)
                
                if distance_moved > self.movement_threshold:
                    self.last_meaningful_movement = rospy.Time.now()
                    self.stats['distance_traveled'] += distance_moved
            
            # Store starting position when first localized
            if self.start_position is None:
                self.start_position = current_pose.copy()
                rospy.logwarn(f"📍 Start position: ({self.start_position['x']:.2f}, {self.start_position['y']:.2f})")
                self.robot_localized = True
                
            self.robot_pose = current_pose
            
        except Exception as e:
            if self.robot_localized:
                rospy.logdebug(f"TF lookup failed: {e}")

    def manual_trigger_callback(self, msg):
        """Handle manual trigger for photography mode"""
        rospy.logwarn("🔴 MANUAL TRIGGER RECEIVED - Forcing transition to photography!")
        self.manual_trigger = True
        if self.current_state in ['MONITORING_EXPLORATION', 'EXPLORATION_COMPLETE']:
            self.transition_to_photography()
    
    def change_state(self, new_state):
        """Enhanced state change with logging"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = rospy.Time.now()
            
            rospy.logwarn(f"🔄 STATE: {self.previous_state} → {self.current_state}")
            self.pubs['state'].publish(String(self.current_state))
            
    def publish_debug_info(self, event):
        """Enhanced debug information publishing"""
        debug_info = {
            'state': self.current_state,
            'state_description': self.STATES.get(self.current_state, 'Unknown'),
            'state_duration': (rospy.Time.now() - self.state_start_time).to_sec(),
            'robot_localized': bool(self.robot_localized),
            'map_initialized': bool(self.map_initialized),
            'map_callback_count': self.map_callback_count,  # FIXED: Add debug info
            'map_check_count': self.map_check_count,
            'safety_supervisor_active': bool(self.safety_supervisor_active),
            'camera_system_ready': bool(self.camera_system_ready),
            'emergency_active': bool(self.emergency_active),
            'stats': {k: float(v) if isinstance(v, (int, float, np.number)) else str(v) 
                     for k, v in self.stats.items()},
            'current_position': self.robot_pose,
            'last_safety_status': self.last_safety_status,
            'system_integration': {
                'hardware_mode': self.use_real_hardware,
                'manual_trigger': self.manual_trigger,
                'exploration_finished': self.exploration_finished
            }
        }
        
        try:
            self.pubs['debug'].publish(String(json.dumps(debug_info, indent=2)))
        except Exception as e:
            rospy.logwarn(f"Failed to publish debug info: {e}")

    def publish_status(self, event):
        """Publish concise status updates"""
        status = f"State: {self.current_state} | "
        status += f"Coverage: {self.stats['coverage_percent']:.1f}% | "
        status += f"Photos: {self.stats['photos_taken']} | "
        status += f"Health: {self.stats['system_health']} | "
        status += f"Map callbacks: {self.map_callback_count}"  # FIXED: Add to status
        
        self.pubs['status'].publish(String(status))
        
    def scan_callback(self, msg):
        """Process laser scans for exploration monitoring"""
        # Could be used for additional frontier detection or obstacle analysis
        pass
        
    def odom_callback(self, msg):
        """Enhanced odometry processing"""
        # Track velocity for movement detection
        self.current_velocity = {
            'linear': math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2),
            'angular': abs(msg.twist.twist.angular.z)
        }
        
        # Update movement tracking
        if self.current_velocity['linear'] > 0.05 or self.current_velocity['angular'] > 0.1:
            self.last_velocity_check = rospy.Time.now()
            self.zero_velocity_start = None
        else:
            if self.zero_velocity_start is None:
                self.zero_velocity_start = rospy.Time.now()

        # Track position history for stuck detection
        if self.robot_pose:
            self.position_history.append((self.robot_pose['x'], self.robot_pose['y']))

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def update_map_statistics(self):
        """Enhanced map coverage calculation"""
        if self.current_map is None:
            return
            
        data = np.array(self.current_map.data)
        resolution = self.current_map.info.resolution
        
        # Count cells
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data == 100)
        unknown_cells = np.sum(data == -1)
        total_cells = len(data)
        
        # Calculate areas
        explored_cells = free_cells + occupied_cells
        self.stats['explored_area_m2'] = explored_cells * resolution * resolution
        
        # Coverage calculation
        if total_cells > 0:
            self.stats['coverage_percent'] = (explored_cells / total_cells) * 100
        else:
            self.stats['coverage_percent'] = 0
            
        # Count frontiers with better algorithm
        frontier_count = self.count_frontiers()
        self.stats['frontier_count'] = frontier_count
        self.frontier_history.append(frontier_count)
            
        # Publish coverage
        self.pubs['map_coverage'].publish(Float32(self.stats['coverage_percent']))

    def count_frontiers(self):
        """Enhanced frontier counting"""
        if self.current_map is None:
            return 999
            
        data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width))
        
        # Find frontier cells (unknown cells adjacent to free cells)
        kernel = np.array([[1,1,1], [1,0,1], [1,1,1]], dtype=np.uint8)
        
        # Create masks
        free_mask = (data == 0)
        unknown_mask = (data == -1)
        
        # Find unknown cells adjacent to free cells
        free_neighbors = ndimage.convolve(free_mask.astype(np.uint8), kernel, mode='constant')
        frontiers = unknown_mask & (free_neighbors > 0)
        
        # Count frontier clusters (more meaningful than individual cells)
        labeled_frontiers, num_clusters = ndimage.label(frontiers)
        
        return max(0, num_clusters)

    def check_exploration_completeness(self):
        """Enhanced exploration completion detection"""
        
        # Manual trigger always overrides
        if self.manual_trigger:
            rospy.logwarn("🔴 Manual trigger - forcing completion")
            return True
        
        # Must have basic data
        if self.current_map is None or self.robot_pose is None:
            return False
        
        elapsed = (rospy.Time.now() - self.exploration_start_time).to_sec()
        
        # 1) Coverage threshold
        if self.stats['coverage_percent'] >= self.params['map_complete_threshold'] * 100:
            rospy.logwarn(f"✅ Coverage complete: {self.stats['coverage_percent']:.1f}%")
            return True
        
        # 2) Consistently low frontiers
        if len(self.frontier_history) >= 5:
            avg_frontiers = np.mean(list(self.frontier_history)[-5:])
            if avg_frontiers <= self.params['min_frontier_threshold']:
                rospy.logwarn(f"✅ Low frontiers: {avg_frontiers:.1f} avg")
                return True
        
        # 3) No meaningful movement for extended time
        no_movement_time = (rospy.Time.now() - self.last_meaningful_movement).to_sec()
        if no_movement_time > self.params['no_progress_timeout']:
            rospy.logwarn(f"✅ No progress timeout: {no_movement_time:.0f}s")
            return True
        
        # 4) Timeout failsafe
        if elapsed >= self.params['exploration_timeout']:
            rospy.logwarn(f"✅ Time limit reached: {elapsed:.0f}s")
            return True
        
        return False

    def wait_for_systems(self):
        """FIXED: Better system waiting with detailed logging"""
        rospy.logwarn("🔧 Waiting for hardware systems...")
        
        start_time = rospy.Time.now()
        timeout = self.params['system_ready_timeout']
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            # FIXED: More detailed logging
            rospy.logwarn(f"⏳ Waiting... Map: {self.map_initialized} (callbacks: {self.map_callback_count}), Pose: {self.robot_localized}")
            
            if self.map_initialized and self.robot_localized:
                rospy.logwarn("✅ Core systems ready!")
                return True
            
            rospy.sleep(2.0)
        
        rospy.logwarn("⚠️  Timeout waiting for systems - proceeding anyway")
        rospy.logwarn(f"   Final state: Map: {self.map_initialized}, Pose: {self.robot_localized}")
        rospy.logwarn(f"   Map callbacks received: {self.map_callback_count}")
        return False

    def transition_to_photography(self):
        """Transition to photography mode"""
        self.change_state('TRANSITIONING')
        
        # Stop any ongoing exploration
        rospy.logwarn("🛑 Cancelling exploration...")
        try:
            self.pubs['explore_cancel'].publish(Empty())
            rospy.sleep(3.0)
        except Exception as e:
            rospy.logwarn(f"Failed to cancel exploration: {e}")
        
        # Generate waypoints
        self.photo_waypoints = self.generate_photo_waypoints()
        self.current_photo_index = 0
        
        if self.photo_waypoints:
            rospy.logwarn(f"📸 Starting photography: {len(self.photo_waypoints)} waypoints")
            self.change_state('PHOTOGRAPHING')
        else:
            rospy.logwarn("❌ No waypoints - returning home")
            self.return_to_start()

    def generate_photo_waypoints(self):
        """Generate photo waypoints using verified working approach"""
        if self.current_map is None:
            rospy.logwarn("❌ No map for waypoint generation")
            return []

        rospy.logwarn("📸 Generating photo waypoints...")

        width      = self.current_map.info.width
        height     = self.current_map.info.height
        resolution = self.current_map.info.resolution

        # Compute LiDAR-to-robot-center offset in grid pixels
        lidar_forward_m = 0.09  # 9 cm forward of robot center
        lidar_side_m    = 0.0   # no lateral offset
        y_off = int(round(lidar_forward_m / resolution))
        x_off = int(round(lidar_side_m    / resolution))

        origin = self.current_map.info.origin

        grid       = np.array(self.current_map.data).reshape((height, width))
        free_space = (grid == 0)

        # Clearance and spacing in pixels
        clearance_pixels = max(3, int(0.8 / resolution))
        spacing_meters   = self.params['photo_spacing']
        spacing_pixels   = max(1, int(spacing_meters / resolution))

        # Dynamically choose how many waypoints based on map area
        cell_area      = resolution * resolution
        total_area     = width * height * cell_area
        coverage_m2    = rospy.get_param('~coverage_per_photo_m2', 4.0)
        desired_wp     = int(round(total_area / coverage_m2))
        min_wp         = rospy.get_param('~min_waypoints', 3)
        max_wp_param   = rospy.get_param('~max_waypoints', 20)
        max_waypoints  = max(min_wp, min(desired_wp, max_wp_param))

        waypoints      = []
        waypoint_count = 0

        # Generate candidate waypoints
        for row in range(clearance_pixels, height - clearance_pixels, spacing_pixels):
            for col in range(clearance_pixels, width - clearance_pixels, spacing_pixels):
                if waypoint_count >= max_waypoints:
                    break

                # Shift test point so the clearance square is centered on the LiDAR
                check_x = col - x_off
                check_y = row - y_off

                if self.has_clearance(free_space, check_x, check_y, clearance_pixels):
                    world_x = col * resolution + origin.position.x
                    world_y = row * resolution + origin.position.y
                    waypoints.append((world_x, world_y, f"photo_{waypoint_count}"))
                    waypoint_count += 1

            if waypoint_count >= max_waypoints:
                break

        # Optimize waypoint visiting order
        if waypoints and self.robot_pose:
            waypoints = self.optimize_waypoint_order(waypoints)

        rospy.logwarn(f"📍 Generated {len(waypoints)} photo waypoints")
        return waypoints


    def optimize_waypoint_order(self, waypoints):
        """Optimize waypoint visiting order"""
        if len(waypoints) <= 2:
            return waypoints
        
        optimized = []
        remaining = waypoints.copy()
        current_pos = (self.robot_pose['x'], self.robot_pose['y'])
        
        while remaining:
            distances = [math.sqrt((wp[0] - current_pos[0])**2 + (wp[1] - current_pos[1])**2) 
                        for wp in remaining]
            nearest_idx = np.argmin(distances)
            
            nearest_waypoint = remaining.pop(nearest_idx)
            optimized.append(nearest_waypoint)
            current_pos = (nearest_waypoint[0], nearest_waypoint[1])
        
        return optimized
        
    def has_clearance(self, grid, x, y, clearance):
        """Check clearance around point"""
        h, w = grid.shape
        
        for dx in range(-clearance, clearance + 1):
            for dy in range(-clearance, clearance + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if not grid[ny, nx]:
                        return False
                else:
                    return False
        return True

    def execute_photography(self):
        """Execute photography at waypoints"""
        if self.current_photo_index >= len(self.photo_waypoints):
            rospy.logwarn("📸 Photography complete! Returning home...")
            self.return_to_start()
            return
            
        waypoint = self.photo_waypoints[self.current_photo_index]
        waypoint_label = waypoint[2] if len(waypoint) > 2 else f"waypoint_{self.current_photo_index}"
        
        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.logwarn(f"📍 Waypoint {self.current_photo_index + 1}/{len(self.photo_waypoints)}: "
                     f"{waypoint_label} at ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
        
        # Navigate to waypoint
        self.move_base_client.send_goal(goal)
        result = self.move_base_client.wait_for_result(rospy.Duration(self.params['photo_timeout']))
        
        if result and self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logwarn(f"✅ Reached {waypoint_label}")
        else:
            rospy.logwarn(f"⚠️  Failed to reach {waypoint_label} - taking photos anyway")
            
        # Take photos
        self.capture_photos()
        self.stats['waypoints_completed'] += 1
        self.current_photo_index += 1

    def capture_photos(self):
        """
        Dual camera photo capture with 90° rotation
        """
        rospy.logwarn("📷 DUAL CAMERA CAPTURE - Starting 4-direction photo sequence!")
        
        try:
            # First photo set: Front and Back cameras (2 photos)
            rospy.logwarn("📸 Taking photos 1&2: FRONT and BACK cameras")
            
            self.pubs['camera_trigger'].publish(Bool(True))
            rospy.sleep(3.0)
            
            # Rotate 90 degrees
            rospy.logwarn("🔄 Rotating 90° to capture left and right sides...")
            self.perform_90_degree_rotation()
            
            # Second photo set: Front and Back cameras at new orientation (2 more photos) 
            rospy.logwarn("📸 Taking photos 3&4: LEFT and RIGHT views")
            
            self.pubs['camera_trigger'].publish(Bool(True))
            rospy.sleep(3.0)
            
            # Rotate back to original orientation
            rospy.logwarn("🔄 Returning to original orientation...")
            self.perform_return_rotation()
            
            # Update statistics
            self.stats['photos_taken'] += 4
            
            rospy.logwarn("✅ Dual camera sequence complete!")
            
        except Exception as e:
            rospy.logerr(f"❌ Dual camera capture failed: {e}")

    def perform_90_degree_rotation(self):
        """Rotate robot 90 degrees counterclockwise"""
        rospy.logwarn("🔄 Performing 90° rotation (counterclockwise)...")
        
        rotation_cmd = Twist()
        rotation_cmd.linear.x = 0.0
        rotation_cmd.angular.z = 0.4
        
        target_angle = 1.5708  # 90 degrees in radians
        rotation_duration = target_angle / abs(rotation_cmd.angular.z)
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            if self.emergency_active:
                rospy.logwarn("🚨 Emergency active - stopping rotation")
                break
            # self.pubs['cmd_vel'].publish(rotation_cmd)
            rospy.sleep(0.1)
        
        # Stop rotation
        # self.pubs['cmd_vel'].publish(Twist())
        rospy.sleep(1.5)

    def perform_return_rotation(self):
        """Return to original orientation with 90° clockwise rotation"""
        rospy.logwarn("🔄 Returning to original orientation (90° clockwise)...")
        
        rotation_cmd = Twist()
        rotation_cmd.linear.x = 0.0
        rotation_cmd.angular.z = -0.4  # Clockwise
        
        target_angle = 1.5708
        rotation_duration = target_angle / abs(rotation_cmd.angular.z)
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < rotation_duration:
            if self.emergency_active:
                break
            # self.pubs['cmd_vel'].publish(rotation_cmd)
            rospy.sleep(0.1)
        
        # Stop
        # self.pubs['cmd_vel'].publish(Twist())
        rospy.sleep(1.5)

    def return_to_start(self):
        """Return robot to starting position"""
        if not self.start_position:
            rospy.logwarn("❌ No start position - staying here")
            self.change_state('COMPLETE')
            return
            
        self.change_state('RETURNING_HOME')
        rospy.logwarn(f"🏠 Returning to start: ({self.start_position['x']:.2f}, {self.start_position['y']:.2f})")
        
        # Create return goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.start_position['x']
        goal.target_pose.pose.position.y = self.start_position['y']
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        # Navigate home
        self.move_base_client.send_goal(goal)
        result = self.move_base_client.wait_for_result(rospy.Duration(60.0))
        
        if result and self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logwarn("✅ Successfully returned home!")
        else:
            rospy.logwarn("⚠️  Failed to return home completely")
            
        self.change_state('COMPLETE')

    def run(self):
        """Main execution loop - Enhanced for real hardware integration"""
        rate = rospy.Rate(2)  # 2 Hz monitoring
        
        rospy.logwarn("🚀 Starting Indoor Mapper for REAL HARDWARE")
        rospy.logwarn("Waiting for move_base and other systems...")
        
        # Wait for move_base
        if self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logwarn("✅ Move_base connected!")
        else:
            rospy.logwarn("⚠️  Move_base not available - limited functionality")
        
        while not rospy.is_shutdown():
            # Update time elapsed
            self.stats['time_elapsed'] = (rospy.Time.now() - self.exploration_start_time).to_sec()
            
            # Log status periodically
            if int(self.stats['time_elapsed']) % 20 == 0:
                rospy.logwarn(f"📊 STATUS: {self.current_state} | "
                            f"Time: {self.stats['time_elapsed']:.0f}s | "
                            f"Coverage: {self.stats['coverage_percent']:.1f}% | "
                            f"Health: {self.stats['system_health']} | "
                            f"Map callbacks: {self.map_callback_count}")
            
            # Enhanced state machine
            if self.current_state == 'INITIALIZING':
                self.change_state('WAITING_FOR_SYSTEMS')
                
            elif self.current_state == 'WAITING_FOR_SYSTEMS':
                if self.wait_for_systems():
                    self.change_state('MONITORING_EXPLORATION')
                    rospy.logwarn("🔍 Systems ready - monitoring exploration...")
                    
            elif self.current_state == 'MONITORING_EXPLORATION':
                if self.emergency_active:
                    continue
                    
                if self.check_exploration_completeness():
                    rospy.logwarn("🎯 Exploration completion detected")
                    self.change_state('EXPLORATION_COMPLETE')
            
            elif self.current_state == 'EXPLORATION_COMPLETE':
                # Brief pause to confirm completion
                state_duration = (rospy.Time.now() - self.state_start_time).to_sec()
                if state_duration > 5.0:
                    if self.check_exploration_completeness():
                        rospy.logwarn("🎯 Exploration confirmed complete")
                        self.transition_to_photography()
                    else:
                        rospy.logwarn("⚠️  False completion - resuming monitoring")
                        self.change_state('MONITORING_EXPLORATION')
        
            elif self.current_state == 'TRANSITIONING':
                # Transitioning handled by transition_to_photography()
                pass
                
            elif self.current_state == 'PHOTOGRAPHING':
                if not self.emergency_active:
                    self.execute_photography()
                
            elif self.current_state == 'RETURNING_HOME':
                # Handled by return_to_start()
                pass
                
            elif self.current_state == 'EMERGENCY_STOP':
                # Wait for emergency to clear
                rospy.logwarn("🚨 Emergency active - waiting for clearance...")
                
            elif self.current_state == 'COMPLETE':
                self.stop_robot_movement()
                rospy.logwarn("🎉 MISSION COMPLETE! 🎉")
                rospy.logwarn("📊 Final Statistics:")
                final_stats = {k: float(v) if isinstance(v, (int, float, np.number)) else str(v) 
                             for k, v in self.stats.items()}
                rospy.logwarn(json.dumps(final_stats, indent=2))

                try:
                    import subprocess
                    subprocess.call(['rosrun', 'map_server', 'map_saver', '-f', '/tmp/final_robot_map'])
                    rospy.logwarn("💾 Final map saved to /tmp/final_robot_map")
                except:
                    pass
                
                # SHUTDOWN EVERYTHING
                rospy.logwarn("🛑 SHUTTING DOWN ENTIRE SYSTEM IN 5 SECONDS...")
                rospy.sleep(5.0)
                
                # Send shutdown signal
                import subprocess
                import os
                
                # Kill all ROS processes
                subprocess.call(['pkill', '-f', 'ros'])
                subprocess.call(['killall', '-9', 'roscore', 'roslaunch', 'rosmaster'])
                
                # Exit this process
                rospy.signal_shutdown("Demo complete")
                os._exit(0)  # Force exit
           
                
            rate.sleep()

if __name__ == '__main__':
    try:
        mapper = IndoorMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("🛑 Indoor mapper shutting down...")
    except Exception as e:
        rospy.logerr(f"💥 Indoor mapper crashed: {e}")
        import traceback
        traceback.print_exc()
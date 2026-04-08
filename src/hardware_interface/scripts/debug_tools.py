#!/usr/bin/env python3
"""
Debug Tools Collection for Indoor Mapping Robot
Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/debug_tools.py

Comprehensive debugging utilities for monitoring robot behavior
"""

import rospy
import json
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading
from collections import deque, defaultdict

class RobotDebugMonitor:
    def __init__(self):
        rospy.init_node('robot_debug_monitor', anonymous=True)
        
        # Data storage
        self.topics_data = defaultdict(deque)
        self.last_messages = {}
        self.message_counts = defaultdict(int)
        self.error_counts = defaultdict(int)
        
        # Timing tracking
        self.start_time = time.time()
        self.last_update_time = defaultdict(float)
        
        # Status tracking
        self.robot_state = "UNKNOWN"
        self.safety_status = "UNKNOWN"
        self.emergency_active = False
        self.camera_status = "UNKNOWN"
        self.coverage_percent = 0.0
        
        # Subscribers for monitoring
        self.subscribers = {
            '/mapper_state': rospy.Subscriber('/mapper_state', String, self.mapper_state_callback),
            '/mapper_debug': rospy.Subscriber('/mapper_debug', String, self.mapper_debug_callback),
            '/safety_status': rospy.Subscriber('/safety_status', String, self.safety_status_callback),
            '/emergency_stop': rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback),
            '/camera/photo_status': rospy.Subscriber('/camera/photo_status', String, self.camera_status_callback),
            '/map_coverage': rospy.Subscriber('/map_coverage', Float32, self.coverage_callback),
            '/cmd_vel': rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback),
            '/cmd_vel_nav': rospy.Subscriber('/cmd_vel_nav', Twist, self.cmd_vel_nav_callback),
            '/scan': rospy.Subscriber('/scan', LaserScan, self.scan_callback),
            '/odom': rospy.Subscriber('/odom', Odometry, self.odom_callback),
        }
        
        # Publishers for debug info
        self.debug_summary_pub = rospy.Publisher('/debug_summary', String, queue_size=1)
        
        # Start monitoring
        self.monitor_thread = threading.Thread(target=self.continuous_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo("🔍 Robot Debug Monitor started")

    def mapper_state_callback(self, msg):
        self.robot_state = msg.data
        self.update_topic_data('/mapper_state', msg.data)

    def mapper_debug_callback(self, msg):
        try:
            debug_data = json.loads(msg.data)
            self.update_topic_data('/mapper_debug', debug_data)
        except json.JSONDecodeError:
            self.error_counts['/mapper_debug'] += 1

    def safety_status_callback(self, msg):
        self.safety_status = msg.data
        self.update_topic_data('/safety_status', msg.data)

    def emergency_stop_callback(self, msg):
        self.emergency_active = msg.data
        self.update_topic_data('/emergency_stop', msg.data)

    def camera_status_callback(self, msg):
        self.camera_status = msg.data
        self.update_topic_data('/camera/photo_status', msg.data)

    def coverage_callback(self, msg):
        self.coverage_percent = msg.data
        self.update_topic_data('/map_coverage', msg.data)

    def cmd_vel_callback(self, msg):
        vel_data = {
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
        }
        self.update_topic_data('/cmd_vel', vel_data)

    def cmd_vel_nav_callback(self, msg):
        vel_data = {
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
        }
        self.update_topic_data('/cmd_vel_nav', vel_data)

    def scan_callback(self, msg):
        scan_summary = {
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges_count': len(msg.ranges),
            'min_range_detected': min([r for r in msg.ranges if r > msg.range_min and r < msg.range_max], default=0),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max
        }
        self.update_topic_data('/scan', scan_summary)

    def odom_callback(self, msg):
        odom_data = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z,
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z,
            }
        }
        self.update_topic_data('/odom', odom_data)

    def update_topic_data(self, topic, data):
        """Update topic data with timestamp"""
        current_time = time.time()
        self.last_messages[topic] = data
        self.message_counts[topic] += 1
        self.last_update_time[topic] = current_time
        
        # Keep last 10 messages for each topic
        if len(self.topics_data[topic]) >= 10:
            self.topics_data[topic].popleft()
        
        self.topics_data[topic].append({
            'timestamp': current_time,
            'data': data
        })

    def get_topic_health(self, topic):
        """Check if topic is healthy (received messages recently)"""
        if topic not in self.last_update_time:
            return "NEVER_RECEIVED"
        
        time_since_last = time.time() - self.last_update_time[topic]
        
        if time_since_last < 2.0:
            return "HEALTHY"
        elif time_since_last < 10.0:
            return "STALE"
        else:
            return "DEAD"

    def get_system_summary(self):
        """Generate comprehensive system summary"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        summary = {
            'timestamp': current_time,
            'uptime_seconds': uptime,
            'robot_state': self.robot_state,
            'safety_status': self.safety_status,
            'emergency_active': self.emergency_active,
            'camera_status': self.camera_status,
            'coverage_percent': self.coverage_percent,
            'topic_health': {
                topic: self.get_topic_health(topic) 
                for topic in ['/scan', '/odom', '/safety_status', '/mapper_state', '/cmd_vel']
            },
            'message_counts': dict(self.message_counts),
            'error_counts': dict(self.error_counts),
            'recent_activity': {
                topic: list(data)[-3:] if data else []  # Last 3 messages
                for topic, data in self.topics_data.items()
            }
        }
        
        return summary

    def print_status_screen(self):
        """Print formatted status to terminal"""
        import os
        os.system('clear')  # Clear terminal
        
        print("🤖 INDOOR MAPPING ROBOT - DEBUG MONITOR")
        print("=" * 60)
        print(f"Time: {time.strftime('%H:%M:%S')}")
        print(f"Uptime: {time.time() - self.start_time:.1f}s")
        print()
        
        # Robot Status
        print("🤖 ROBOT STATUS")
        print("-" * 20)
        print(f"State: {self.robot_state}")
        print(f"Safety: {self.safety_status}")
        print(f"Emergency: {'🚨 ACTIVE' if self.emergency_active else '✅ Clear'}")
        print(f"Camera: {self.camera_status}")
        print(f"Coverage: {self.coverage_percent:.1f}%")
        print()
        
        # Topic Health
        print("📡 TOPIC HEALTH")
        print("-" * 20)
        critical_topics = ['/scan', '/odom', '/safety_status', '/mapper_state', '/cmd_vel']
        for topic in critical_topics:
            health = self.get_topic_health(topic)
            status_emoji = "✅" if health == "HEALTHY" else "⚠️ " if health == "STALE" else "❌"
            count = self.message_counts.get(topic, 0)
            print(f"{status_emoji} {topic}: {health} ({count} msgs)")
        print()
        
        # Recent Command Velocities
        if '/cmd_vel' in self.last_messages:
            cmd_vel = self.last_messages['/cmd_vel']
            cmd_vel_nav = self.last_messages.get('/cmd_vel_nav', {})
            print("🎮 MOVEMENT COMMANDS")
            print("-" * 20)
            print(f"Safety Output: linear={cmd_vel['linear']['x']:.2f}, angular={cmd_vel['angular']['z']:.2f}")
            if cmd_vel_nav:
                print(f"Nav Input:     linear={cmd_vel_nav['linear']['x']:.2f}, angular={cmd_vel_nav['angular']['z']:.2f}")
            print()
        
        # Recent Scan Data
        if '/scan' in self.last_messages:
            scan = self.last_messages['/scan']
            print("📡 LIDAR STATUS")
            print("-" * 20)
            print(f"Min range detected: {scan.get('min_range_detected', 0):.2f}m")
            print(f"Ranges count: {scan.get('ranges_count', 0)}")
            print()
        
        # Recent Position
        if '/odom' in self.last_messages:
            odom = self.last_messages['/odom']
            pos = odom['position']
            vel = odom['linear_velocity']
            print("📍 POSITION & VELOCITY")
            print("-" * 20)
            print(f"Position: x={pos['x']:.2f}, y={pos['y']:.2f}")
            print(f"Velocity: x={vel['x']:.2f}, z_angular={odom['angular_velocity']['z']:.2f}")
            print()
        
        # Error Summary
        if any(self.error_counts.values()):
            print("⚠️  ERRORS")
            print("-" * 20)
            for topic, count in self.error_counts.items():
                if count > 0:
                    print(f"{topic}: {count} errors")
            print()
        
        print("Press Ctrl+C to exit")

    def continuous_monitor(self):
        """Continuous monitoring loop"""
        while not rospy.is_shutdown():
            try:
                self.print_status_screen()
                
                # Publish summary for other tools
                summary = self.get_system_summary()
                self.debug_summary_pub.publish(String(json.dumps(summary, indent=2)))
                
                time.sleep(2)  # Update every 2 seconds
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Monitor error: {e}")

    def get_diagnostic_report(self):
        """Generate detailed diagnostic report"""
        summary = self.get_system_summary()
        
        report = f"""
INDOOR MAPPING ROBOT - DIAGNOSTIC REPORT
========================================
Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}
Uptime: {summary['uptime_seconds']:.1f} seconds

ROBOT STATUS:
- State: {summary['robot_state']}
- Safety: {summary['safety_status']}
- Emergency: {'ACTIVE' if summary['emergency_active'] else 'Clear'}
- Camera: {summary['camera_status']}
- Coverage: {summary['coverage_percent']:.1f}%

TOPIC HEALTH:
"""
        for topic, health in summary['topic_health'].items():
            report += f"- {topic}: {health}\n"
        
        report += "\nMESSAGE COUNTS:\n"
        for topic, count in summary['message_counts'].items():
            report += f"- {topic}: {count} messages\n"
        
        if any(summary['error_counts'].values()):
            report += "\nERRORS:\n"
            for topic, count in summary['error_counts'].items():
                if count > 0:
                    report += f"- {topic}: {count} errors\n"
        
        return report

class TopicLatencyMonitor:
    """Monitor topic update frequencies and latencies"""
    
    def __init__(self):
        self.topic_times = defaultdict(list)
        self.subscribers = {}
        
        # Monitor critical topics
        critical_topics = [
            ('/scan', LaserScan),
            ('/odom', Odometry),
            ('/safety_status', String),
            ('/mapper_state', String),
        ]
        
        for topic, msg_type in critical_topics:
            self.subscribers[topic] = rospy.Subscriber(
                topic, msg_type, 
                lambda msg, t=topic: self.topic_callback(t, msg)
            )
    
    def topic_callback(self, topic, msg):
        current_time = time.time()
        self.topic_times[topic].append(current_time)
        
        # Keep only last 10 timestamps
        if len(self.topic_times[topic]) > 10:
            self.topic_times[topic].pop(0)
    
    def get_frequencies(self):
        """Calculate topic frequencies"""
        frequencies = {}
        
        for topic, times in self.topic_times.items():
            if len(times) < 2:
                frequencies[topic] = 0.0
            else:
                time_diffs = [times[i] - times[i-1] for i in range(1, len(times))]
                avg_interval = sum(time_diffs) / len(time_diffs)
                frequencies[topic] = 1.0 / avg_interval if avg_interval > 0 else 0.0
        
        return frequencies
    
    def print_frequency_report(self):
        """Print frequency report"""
        frequencies = self.get_frequencies()
        
        print("\n📊 TOPIC FREQUENCIES")
        print("=" * 30)
        for topic, freq in frequencies.items():
            status = "✅" if freq > 0.5 else "⚠️ " if freq > 0.1 else "❌"
            print(f"{status} {topic}: {freq:.2f} Hz")

def main():
    """Main debug monitor function"""
    try:
        monitor = RobotDebugMonitor()
        latency_monitor = TopicLatencyMonitor()
        
        # Additional monitoring thread for frequencies
        def frequency_monitor():
            while not rospy.is_shutdown():
                time.sleep(10)  # Update frequencies every 10 seconds
                latency_monitor.print_frequency_report()
        
        freq_thread = threading.Thread(target=frequency_monitor)
        freq_thread.daemon = True
        freq_thread.start()
        
        rospy.loginfo("🔍 Starting comprehensive robot debugging...")
        rospy.loginfo("Monitor will display in terminal and publish to /debug_summary")
        
        rospy.spin()
        
    except KeyboardInterrupt:
        print("\n🛑 Debug monitor shutting down...")
    except Exception as e:
        rospy.logerr(f"Debug monitor error: {e}")

if __name__ == '__main__':
    main()
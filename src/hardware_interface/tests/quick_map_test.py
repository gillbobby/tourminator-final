#!/usr/bin/env python3
"""
Quick Map Test - Check if map is being generated
Run this while full_robot_debug.launch is running
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import time

class QuickMapTest:
    def __init__(self):
        rospy.init_node('quick_map_test')
        
        self.map_received = False
        self.map_count = 0
        self.mapper_state = "unknown"
        
        # Subscribe to topics
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.state_sub = rospy.Subscriber('/mapper_state', String, self.state_callback)
        
        rospy.logwarn("🔍 Quick Map Test Started")
        rospy.logwarn("Checking if map is being generated...")
        
    def map_callback(self, msg):
        """Called when map is received"""
        self.map_count += 1
        
        if not self.map_received:
            self.map_received = True
            rospy.logwarn("🗺️  FIRST MAP RECEIVED!")
            rospy.logwarn(f"   Size: {msg.info.width} x {msg.info.height}")
            rospy.logwarn(f"   Resolution: {msg.info.resolution} m/cell")
            rospy.logwarn(f"   Data length: {len(msg.data)}")
            
            # Check map content
            free_cells = sum(1 for cell in msg.data if cell == 0)
            occupied_cells = sum(1 for cell in msg.data if cell == 100)
            unknown_cells = sum(1 for cell in msg.data if cell == -1)
            
            rospy.logwarn(f"   Free cells: {free_cells}")
            rospy.logwarn(f"   Occupied cells: {occupied_cells}")
            rospy.logwarn(f"   Unknown cells: {unknown_cells}")
            
            if free_cells > 50 and occupied_cells > 5:
                rospy.logwarn("✅ Map looks good! SLAM is working!")
            else:
                rospy.logwarn("⚠️  Map data seems limited")
                
        elif self.map_count % 10 == 0:  # Every 10th map
            rospy.logwarn(f"📊 Map update #{self.map_count}")
    
    def state_callback(self, msg):
        """Track mapper state"""
        if msg.data != self.mapper_state:
            self.mapper_state = msg.data
            rospy.logwarn(f"🤖 Mapper state: {msg.data}")
    
    def run_test(self):
        """Run the test for 60 seconds"""
        rospy.logwarn("Testing for 60 seconds...")
        
        start_time = time.time()
        while (time.time() - start_time) < 60 and not rospy.is_shutdown():
            rospy.sleep(1)
            
            elapsed = int(time.time() - start_time)
            if elapsed % 10 == 0:  # Every 10 seconds
                if self.map_received:
                    rospy.logwarn(f"⏰ {elapsed}s: ✅ Map working, received {self.map_count} updates")
                else:
                    rospy.logwarn(f"⏰ {elapsed}s: ❌ No map received yet")
                    
        # Final report
        rospy.logwarn("=" * 50)
        rospy.logwarn("🎯 FINAL TEST RESULTS:")
        
        if self.map_received:
            rospy.logwarn(f"✅ SUCCESS: Map is being generated!")
            rospy.logwarn(f"   Total map updates: {self.map_count}")
            rospy.logwarn(f"   Mapper state: {self.mapper_state}")
            rospy.logwarn("")
            rospy.logwarn("🎉 GMAPPING IS WORKING!")
            rospy.logwarn("The issue with indoor_mapper should be fixed now.")
        else:
            rospy.logwarn("❌ FAILURE: No map received")
            rospy.logwarn("GMAPPING IS NOT WORKING")
            rospy.logwarn("")
            rospy.logwarn("🔧 Troubleshooting needed:")
            rospy.logwarn("1. Check if slam_gmapping node is running: rosnode list | grep slam")
            rospy.logwarn("2. Check scan data: rostopic echo /scan -n 1")
            rospy.logwarn("3. Check odom data: rostopic echo /odom -n 1")
            rospy.logwarn("4. Check TF transforms: rosrun tf tf_echo odom base_link")

if __name__ == '__main__':
    try:
        tester = QuickMapTest()
        tester.run_test()
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
#!/bin/bash
# Comprehensive System Diagnostics for Indoor Mapping Robot
# Run this while full_robot_debug.launch is running

echo "🔍 INDOOR MAPPING ROBOT - SYSTEM DIAGNOSTICS"
echo "=============================================="
echo ""

# Check if roscore is running
echo "1. 🤖 ROS CORE STATUS"
echo "-------------------"
if pgrep -f roscore > /dev/null; then
    echo "✅ ROS Core is running"
else
    echo "❌ ROS Core is NOT running"
    exit 1
fi
echo ""

# Check active nodes
echo "2. 📡 ACTIVE ROS NODES"
echo "---------------------"
rosnode list | grep -E "(slam_gmapping|indoor_mapper|safety_supervisor|motor_bridge|camera_bridge|rplidar|explore)" | while read node; do
    if rosnode ping $node -c 1 > /dev/null 2>&1; then
        echo "✅ $node"
    else
        echo "❌ $node (not responding)"
    fi
done
echo ""

# Check critical topics
echo "3. 📊 TOPIC STATUS"
echo "------------------"
critical_topics=("/scan" "/odom" "/map" "/cmd_vel" "/cmd_vel_nav" "/safety_status")

for topic in "${critical_topics[@]}"; do
    if rostopic list | grep -q "^$topic$"; then
        # Check if topic has data
        if timeout 3 rostopic echo $topic -n 1 > /dev/null 2>&1; then
            hz_info=$(timeout 5 rostopic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}')
            if [ ! -z "$hz_info" ]; then
                echo "✅ $topic: ${hz_info} Hz"
            else
                echo "⚠️  $topic: Published but low/no rate"
            fi
        else
            echo "❌ $topic: No data"
        fi
    else
        echo "❌ $topic: Topic doesn't exist"
    fi
done
echo ""

# Check gmapping specifically
echo "4. 🗺️  GMAPPING STATUS"
echo "---------------------"
if rosnode list | grep -q slam_gmapping; then
    echo "✅ gmapping node is running"
    
    # Check if map is being published
    if timeout 5 rostopic echo /map -n 1 > /dev/null 2>&1; then
        echo "✅ /map topic has data"
        
        # Get map info
        map_info=$(rostopic echo /map/info -n 1 2>/dev/null)
        if [ ! -z "$map_info" ]; then
            width=$(echo "$map_info" | grep "width:" | awk '{print $2}')
            height=$(echo "$map_info" | grep "height:" | awk '{print $2}')
            resolution=$(echo "$map_info" | grep "resolution:" | awk '{print $2}')
            echo "   Map size: ${width}x${height}, resolution: ${resolution}m/cell"
        fi
    else
        echo "❌ /map topic has no data"
    fi
    
    # Check gmapping parameters
    echo "   Checking gmapping parameters:"
    rosparam get /slam_gmapping/scan_topic 2>/dev/null && echo "   ✅ scan_topic parameter set" || echo "   ❌ scan_topic parameter missing"
    rosparam get /slam_gmapping/odom_frame 2>/dev/null && echo "   ✅ odom_frame parameter set" || echo "   ❌ odom_frame parameter missing"
    rosparam get /slam_gmapping/base_frame 2>/dev/null && echo "   ✅ base_frame parameter set" || echo "   ❌ base_frame parameter missing"
else
    echo "❌ gmapping node is NOT running"
fi
echo ""

# Check TF tree
echo "5. 🔗 TF TRANSFORM STATUS"
echo "-------------------------"
if which rosrun >/dev/null && rospack find tf2_tools >/dev/null 2>&1; then
    echo "TF Tree (showing available frames):"
    timeout 5 rosrun tf tf_echo odom base_link 2>/dev/null && echo "✅ odom -> base_link transform OK" || echo "❌ odom -> base_link transform FAILED"
    timeout 5 rosrun tf tf_echo map base_link 2>/dev/null && echo "✅ map -> base_link transform OK" || echo "❌ map -> base_link transform FAILED"
    timeout 5 rosrun tf tf_echo base_link lidar_link 2>/dev/null && echo "✅ base_link -> lidar_link transform OK" || echo "❌ base_link -> lidar_link transform FAILED"
else
    echo "⚠️  TF tools not available for detailed checking"
fi
echo ""

# Check explore_lite
echo "6. 🎯 EXPLORATION STATUS"
echo "------------------------"
if rosnode list | grep -q explore; then
    echo "✅ explore_lite node is running"
    if timeout 3 rostopic echo /explore/status -n 1 > /dev/null 2>&1; then
        echo "✅ explore status available"
    else
        echo "⚠️  explore status not publishing"
    fi
else
    echo "❌ explore_lite node is NOT running"
fi
echo ""

# Check navigation stack
echo "7. 🧭 NAVIGATION STACK"
echo "----------------------"
if rosnode list | grep -q move_base; then
    echo "✅ move_base node is running"
    
    # Check if costmaps are working
    if timeout 3 rostopic echo /move_base/local_costmap/costmap -n 1 > /dev/null 2>&1; then
        echo "✅ local_costmap active"
    else
        echo "❌ local_costmap not publishing"
    fi
    
    if timeout 3 rostopic echo /move_base/global_costmap/costmap -n 1 > /dev/null 2>&1; then
        echo "✅ global_costmap active"
    else
        echo "❌ global_costmap not publishing"
    fi
else
    echo "❌ move_base node is NOT running"
fi
echo ""

# Hardware status
echo "8. 🔧 HARDWARE STATUS"
echo "---------------------"
# LiDAR
if timeout 3 rostopic echo /scan -n 1 > /dev/null 2>&1; then
    scan_data=$(timeout 3 rostopic echo /scan -n 1 2>/dev/null)
    range_count=$(echo "$scan_data" | grep -c "ranges:" 2>/dev/null || echo "0")
    if [ "$range_count" -gt 0 ]; then
        echo "✅ LiDAR publishing scan data"
    else
        echo "⚠️  LiDAR connected but scan data format issue"
    fi
else
    echo "❌ LiDAR not publishing data"
fi

# Motors/Odometry
if timeout 3 rostopic echo /odom -n 1 > /dev/null 2>&1; then
    echo "✅ Motor bridge publishing odometry"
else
    echo "❌ Motor bridge not publishing odometry"
fi

# Safety
if timeout 3 rostopic echo /safety_status -n 1 > /dev/null 2>&1; then
    safety_msg=$(timeout 3 rostopic echo /safety_status -n 1 2>/dev/null | grep "data:" | cut -d'"' -f2)
    echo "✅ Safety supervisor active: $safety_msg"
else
    echo "❌ Safety supervisor not active"
fi
echo ""

# Final recommendations
echo "9. 🎯 RECOMMENDATIONS"
echo "---------------------"
echo "If you see issues above:"
echo "1. ❌ /map topic has no data → Check gmapping configuration and input topics"
echo "2. ❌ gmapping node NOT running → Navigation launch file issue"
echo "3. ❌ Transform errors → Check static transform publishers and TF tree"
echo "4. ❌ Topics missing → Check launch file includes and node startup sequence"
echo ""
echo "Next steps:"
echo "• Run: rosnode info /slam_gmapping (if it exists)"
echo "• Run: rostopic info /scan"
echo "• Run: rostopic info /odom"
echo "• Check: rqt_graph to visualize node connections"
echo ""
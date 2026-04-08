#!/bin/bash
# Real-time Robot System Monitor
# Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/system_monitor.sh

echo "🤖 INDOOR MAPPING ROBOT - SYSTEM MONITOR"
echo "========================================"
echo "Monitor all key topics and system health in real-time"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to check if a topic is publishing
check_topic() {
    local topic=$1
    local timeout=${2:-2}
    
    if timeout $timeout rostopic echo $topic -n 1 >/dev/null 2>&1; then
        echo -e "${GREEN}✅ $topic${NC}"
        return 0
    else
        echo -e "${RED}❌ $topic${NC}"
        return 1
    fi
}

# Function to get topic frequency
get_frequency() {
    local topic=$1
    local freq=$(timeout 5 rostopic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$freq" ]; then
        echo -e "${BLUE}📊 $topic: ${freq} Hz${NC}"
    else
        echo -e "${YELLOW}⚠️  $topic: No data${NC}"
    fi
}

# Function to display current robot state
show_robot_state() {
    echo -e "\n${BLUE}🤖 ROBOT STATE${NC}"
    echo "=============="
    
    # Get mapper state
    local mapper_state=$(timeout 2 rostopic echo /mapper_state -n 1 2>/dev/null | grep "data:" | cut -d'"' -f2)
    if [ ! -z "$mapper_state" ]; then
        case "$mapper_state" in
            *"COMPLETE"*)
                echo -e "State: ${GREEN}$mapper_state${NC}"
                ;;
            *"EMERGENCY"*|*"ERROR"*)
                echo -e "State: ${RED}$mapper_state${NC}"
                ;;
            *"MONITORING"*|*"PHOTOGRAPHING"*)
                echo -e "State: ${CYAN}$mapper_state${NC}"
                ;;
            *)
                echo -e "State: ${YELLOW}$mapper_state${NC}"
                ;;
        esac
    else
        echo -e "State: ${RED}Unknown${NC}"
    fi
    
    # Get safety status
    local safety_status=$(timeout 2 rostopic echo /safety_status -n 1 2>/dev/null | grep "data:" | cut -d'"' -f2)
    if [ ! -z "$safety_status" ]; then
        case "$safety_status" in
            *"clear"*)
                echo -e "Safety: ${GREEN}$safety_status${NC}"
                ;;
            *"emergency"*)
                echo -e "Safety: ${RED}$safety_status${NC}"
                ;;
            *"critical"*)
                echo -e "Safety: ${YELLOW}$safety_status${NC}"
                ;;
            *)
                echo -e "Safety: ${BLUE}$safety_status${NC}"
                ;;
        esac
    else
        echo -e "Safety: ${RED}Unknown${NC}"
    fi
    
    # Get map coverage
    local coverage=$(timeout 2 rostopic echo /map_coverage -n 1 2>/dev/null | grep "data:" | awk '{print $2}')
    if [ ! -z "$coverage" ]; then
        local coverage_int=$(echo "$coverage" | cut -d'.' -f1)
        if [ "$coverage_int" -ge 60 ]; then
            echo -e "Coverage: ${GREEN}${coverage}%${NC}"
        elif [ "$coverage_int" -ge 30 ]; then
            echo -e "Coverage: ${YELLOW}${coverage}%${NC}"
        else
            echo -e "Coverage: ${CYAN}${coverage}%${NC}"
        fi
    else
        echo -e "Coverage: ${RED}Unknown${NC}"
    fi
    
    # Get camera status
    local camera_status=$(timeout 2 rostopic echo /camera/photo_status -n 1 2>/dev/null | grep "data:" | cut -d'"' -f2)
    if [ ! -z "$camera_status" ]; then
        if [[ "$camera_status" == *"active"* ]] || [[ "$camera_status" == *"ready"* ]]; then
            echo -e "Camera: ${GREEN}$camera_status${NC}"
        elif [[ "$camera_status" == *"error"* ]] || [[ "$camera_status" == *"failed"* ]]; then
            echo -e "Camera: ${RED}$camera_status${NC}"
        else
            echo -e "Camera: ${BLUE}$camera_status${NC}"
        fi
    else
        echo -e "Camera: ${RED}No status${NC}"
    fi
}

# Function to show movement status
show_movement_status() {
    echo -e "\n${PURPLE}🎮 MOVEMENT STATUS${NC}"
    echo "=================="
    
    # Get current velocities
    local cmd_vel=$(timeout 2 rostopic echo /cmd_vel -n 1 2>/dev/null)
    local cmd_vel_nav=$(timeout 2 rostopic echo /cmd_vel_nav -n 1 2>/dev/null)
    
    if [ ! -z "$cmd_vel" ]; then
        local linear=$(echo "$cmd_vel" | grep -A3 "linear:" | grep "x:" | awk '{print $2}')
        local angular=$(echo "$cmd_vel" | grep -A3 "angular:" | grep "z:" | awk '{print $2}')
        
        if [ ! -z "$linear" ] && [ ! -z "$angular" ]; then
            # Color code based on movement
            if [ "$(echo "$linear" | awk '{if($1>0.05 || $1<-0.05) print "moving"; else print "stationary"}')" = "moving" ] || [ "$(echo "$angular" | awk '{if($1>0.1 || $1<-0.1) print "moving"; else print "stationary"}')" = "moving" ]; then
                echo -e "Motor Output: ${GREEN}linear=${linear}, angular=${angular}${NC}"
            else
                echo -e "Motor Output: ${BLUE}linear=${linear}, angular=${angular}${NC} (stationary)"
            fi
        fi
    else
        echo -e "Motor Output: ${RED}No data${NC}"
    fi
    
    if [ ! -z "$cmd_vel_nav" ]; then
        local nav_linear=$(echo "$cmd_vel_nav" | grep -A3 "linear:" | grep "x:" | awk '{print $2}')
        local nav_angular=$(echo "$cmd_vel_nav" | grep -A3 "angular:" | grep "z:" | awk '{print $2}')
        
        if [ ! -z "$nav_linear" ] && [ ! -z "$nav_angular" ]; then
            echo -e "Nav Command:  ${CYAN}linear=${nav_linear}, angular=${nav_angular}${NC}"
        fi
    else
        echo -e "Nav Command:  ${RED}No data${NC}"
    fi
    
    # Get position
    local odom=$(timeout 2 rostopic echo /odom -n 1 2>/dev/null)
    if [ ! -z "$odom" ]; then
        local pos_x=$(echo "$odom" | grep -A6 "position:" | grep "x:" | awk '{print $2}')
        local pos_y=$(echo "$odom" | grep -A6 "position:" | grep "y:" | awk '{print $2}')
        
        if [ ! -z "$pos_x" ] && [ ! -z "$pos_y" ]; then
            echo -e "Position:     ${BLUE}x=${pos_x}, y=${pos_y}${NC}"
        fi
    fi
}

# Function to show sensor status
show_sensor_status() {
    echo -e "\n${CYAN}📡 SENSOR STATUS${NC}"
    echo "================"
    
    # LiDAR status
    local scan=$(timeout 2 rostopic echo /scan -n 1 2>/dev/null)
    if [ ! -z "$scan" ]; then
        local range_count=$(echo "$scan" | grep -A1000 "ranges:" | grep -c "- ")
        local angle_min=$(echo "$scan" | grep "angle_min:" | awk '{print $2}')
        local angle_max=$(echo "$scan" | grep "angle_max:" | awk '{print $2}')
        
        if [ ! -z "$range_count" ] && [ "$range_count" -gt 100 ]; then
            echo -e "LiDAR:    ${GREEN}Active (${range_count} points)${NC}"
        else
            echo -e "LiDAR:    ${YELLOW}Limited data (${range_count} points)${NC}"
        fi
    else
        echo -e "LiDAR:    ${RED}No data${NC}"
    fi
    
    # Check if emergency stop is active
    local emergency=$(timeout 2 rostopic echo /emergency_stop -n 1 2>/dev/null | grep "data:" | awk '{print $2}')
    if [ ! -z "$emergency" ]; then
        if [ "$emergency" = "True" ] || [ "$emergency" = "true" ]; then
            echo -e "Emergency: ${RED}🚨 ACTIVE${NC}"
        else
            echo -e "Emergency: ${GREEN}✅ Clear${NC}"
        fi
    else
        echo -e "Emergency: ${YELLOW}Unknown${NC}"
    fi
}

# Function to show detailed debug info
show_debug_info() {
    echo -e "\n${YELLOW}🔍 DEBUG INFO${NC}"
    echo "=============="
    
    # Show latest debug message (first 15 lines)
    local debug_msg=$(timeout 5 rostopic echo /mapper_debug -n 1 2>/dev/null)
    if [ ! -z "$debug_msg" ]; then
        echo "$debug_msg" | head -15
        echo ""
        echo -e "${BLUE}(Showing first 15 lines of debug output)${NC}"
    else
        echo "No debug info available"
    fi
}

# Function to show node status
show_node_status() {
    echo -e "\n${PURPLE}🔧 NODE STATUS${NC}"
    echo "=============="
    
    # Check critical nodes
    local nodes=$(rosnode list 2>/dev/null)
    
    if [ ! -z "$nodes" ]; then
        # Check for key nodes
        if echo "$nodes" | grep -q "safety_supervisor"; then
            echo -e "Safety Supervisor: ${GREEN}✅ Running${NC}"
        else
            echo -e "Safety Supervisor: ${RED}❌ Missing${NC}"
        fi
        
        if echo "$nodes" | grep -q "motor_bridge"; then
            echo -e "Motor Bridge:      ${GREEN}✅ Running${NC}"
        else
            echo -e "Motor Bridge:      ${RED}❌ Missing${NC}"
        fi
        
        if echo "$nodes" | grep -q "indoor_mapper"; then
            echo -e "Indoor Mapper:     ${GREEN}✅ Running${NC}"
        else
            echo -e "Indoor Mapper:     ${RED}❌ Missing${NC}"
        fi
        
        if echo "$nodes" | grep -q "camera_bridge"; then
            echo -e "Camera Bridge:     ${GREEN}✅ Running${NC}"
        else
            echo -e "Camera Bridge:     ${RED}❌ Missing${NC}"
        fi
        
        if echo "$nodes" | grep -q "rplidar"; then
            echo -e "RPLidar Node:      ${GREEN}✅ Running${NC}"
        else
            echo -e "RPLidar Node:      ${RED}❌ Missing${NC}"
        fi
    else
        echo -e "${RED}Cannot get node list - is roscore running?${NC}"
    fi
}

# Function to monitor system continuously
continuous_monitor() {
    while true; do
        clear
        echo "🤖 INDOOR MAPPING ROBOT - LIVE MONITOR"
        echo "======================================"
        date
        echo ""
        
        # Check critical topics
        echo -e "${BLUE}📡 TOPIC STATUS${NC}"
        echo "==============="
        check_topic "/scan" 3
        check_topic "/odom" 3
        check_topic "/map" 3
        check_topic "/safety_status" 3
        check_topic "/mapper_state" 3
        check_topic "/cmd_vel" 3
        check_topic "/cmd_vel_nav" 3
        check_topic "/camera/photo_status" 3
        
        echo ""
        echo -e "${BLUE}📊 FREQUENCIES${NC}"
        echo "=============="
        get_frequency "/scan"
        get_frequency "/odom"
        get_frequency "/cmd_vel"
        get_frequency "/safety_status"
        
        # Show robot state
        show_robot_state
        
        # Show movement status
        show_movement_status
        
        # Show sensor status
        show_sensor_status
        
        echo ""
        echo -e "${YELLOW}Commands: 'd'=debug info, 'n'=nodes, 'q'=quit, Enter=refresh${NC}"
        
        # Wait for input or timeout
        read -t 3 -n 1 key
        case $key in
            'd'|'D')
                clear
                show_debug_info
                echo ""
                read -p "Press Enter to continue..."
                ;;
            'n'|'N')
                clear
                show_node_status
                echo ""
                read -p "Press Enter to continue..."
                ;;
            'q'|'Q')
                echo ""
                echo "Exiting monitor..."
                exit 0
                ;;
        esac
    done
}

# Function to run comprehensive system check
comprehensive_check() {
    echo -e "${BLUE}🔍 COMPREHENSIVE SYSTEM CHECK${NC}"
    echo "============================="
    echo ""
    
    # Check ROS core
    if pgrep -f roscore > /dev/null; then
        echo -e "${GREEN}✅ ROS Core running${NC}"
    else
        echo -e "${RED}❌ ROS Core not running${NC}"
        echo "Start with: roscore"
        return 1
    fi
    
    echo ""
    echo -e "${BLUE}📡 CHECKING ALL TOPICS${NC}"
    echo "====================="
    check_topic "/scan" 5
    check_topic "/odom" 5
    check_topic "/map" 5
    check_topic "/safety_status" 5
    check_topic "/mapper_state" 5
    check_topic "/camera/photo_status" 5
    check_topic "/cmd_vel" 5
    check_topic "/cmd_vel_nav" 5
    check_topic "/emergency_stop" 5
    check_topic "/map_coverage" 5
    
    show_robot_state
    show_movement_status
    show_sensor_status
    show_node_status
    
    echo ""
    echo -e "${GREEN}System check complete!${NC}"
}

# Main menu
case "${1:-menu}" in
    "continuous"|"c")
        continuous_monitor
        ;;
    "check"|"status")
        comprehensive_check
        ;;
    "debug"|"d")
        show_debug_info
        ;;
    "freq"|"f")
        echo -e "${BLUE}📊 TOPIC FREQUENCIES${NC}"
        echo "==================="
        get_frequency "/scan"
        get_frequency "/odom"
        get_frequency "/cmd_vel"
        get_frequency "/safety_status"
        get_frequency "/mapper_debug"
        get_frequency "/camera/photo_status"
        ;;
    "nodes"|"n")
        show_node_status
        ;;
    "state"|"s")
        show_robot_state
        show_movement_status
        ;;
    "help"|"h"|"--help")
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  continuous, c  - Live monitoring (default)"
        echo "  check, status  - Comprehensive system check"
        echo "  debug, d       - Show debug information"
        echo "  freq, f        - Show topic frequencies"
        echo "  nodes, n       - Show node status"
        echo "  state, s       - Show robot state only"
        echo "  help, h        - Show this help"
        echo ""
        echo "Examples:"
        echo "  $0 continuous  # Live monitoring"
        echo "  $0 check       # Quick status check"
        echo "  $0 debug       # Debug information"
        echo ""
        ;;
    *)
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  continuous, c  - Live monitoring (default)"
        echo "  check, status  - Check all topics once"
        echo "  debug, d       - Show debug information"
        echo "  freq, f        - Show topic frequencies"
        echo "  nodes, n       - Show node status"
        echo "  state, s       - Show robot state"
        echo "  help, h        - Show help"
        echo ""
        echo "Starting continuous monitoring in 3 seconds..."
        sleep 3
        continuous_monitor
        ;;
esac
#!/bin/bash
echo "🤖 COMPLETE ROBOT HARDWARE CONNECTION CHECK"
echo "==========================================="
echo "Run this after: clone → chmod → catkin_make"
echo ""

# 1. POWER & SYSTEM HEALTH (CRITICAL)
echo "=== 🔋 POWER & SYSTEM HEALTH ==="
vcgencmd measure_volts 2>/dev/null && vcgencmd measure_temp 2>/dev/null || echo "❌ vcgencmd not available"
throttled=$(vcgencmd get_throttled 2>/dev/null)
if [ "$throttled" = "throttled=0x0" ]; then
    echo "✅ No power throttling detected"
else
    echo "⚠️  Power throttling detected: $throttled"
fi
echo ""

# 2. GPIO ACCESS & TESTING
echo "=== 🔌 GPIO ACCESS & TESTING ==="
if command -v gpio &> /dev/null; then
    echo "✅ wiringPi available"
    gpio readall
else
    echo "❌ wiringPi not installed, using sysfs method"
    sudo cat /sys/kernel/debug/gpio 2>/dev/null || echo "Debug GPIO not accessible"
fi

echo "Testing motor GPIO pins..."
for pin in 17 18 22 23; do
    echo "Testing GPIO $pin (Motor pin)"
    echo "$pin" | sudo tee /sys/class/gpio/export 2>/dev/null
    echo "out" | sudo tee /sys/class/gpio/gpio$pin/direction 2>/dev/null
    echo "1" | sudo tee /sys/class/gpio/gpio$pin/value 2>/dev/null
    sleep 0.1
    echo "0" | sudo tee /sys/class/gpio/gpio$pin/value 2>/dev/null
    echo "$pin" | sudo tee /sys/class/gpio/unexport 2>/dev/null
done

echo "Testing GPIO 21 (Emergency button pin)"
echo "21" | sudo tee /sys/class/gpio/export 2>/dev/null
echo "in" | sudo tee /sys/class/gpio/gpio21/direction 2>/dev/null
button_value=$(cat /sys/class/gpio/gpio21/value 2>/dev/null)
echo "Emergency button pin 21 value: $button_value (1=not pressed, 0=pressed)"
echo "21" | sudo tee /sys/class/gpio/unexport 2>/dev/null
echo ""

# 3. USB DEVICES & CAMERAS (ENHANCED)
echo "=== 📷 USB DEVICES & CAMERAS (ENHANCED) ==="
echo "All USB devices:"
lsusb
echo ""

# Check specifically for Microsoft LifeCam Studio cameras
echo "Microsoft LifeCam Studio Detection:"
microsoft_cameras=$(lsusb | grep -i microsoft | wc -l)
if [ "$microsoft_cameras" -gt 0 ]; then
    echo "✅ $microsoft_cameras Microsoft device(s) detected:"
    lsusb | grep -i microsoft
    
    # Check for LifeCam specifically
    if lsusb | grep -i lifecam >/dev/null; then
        echo "✅ Microsoft LifeCam Studio confirmed"
    else
        echo "⚠️  Microsoft device found but may not be LifeCam Studio"
    fi
else
    echo "❌ No Microsoft cameras detected"
fi
echo ""

echo "Video device enumeration:"
if ls /dev/video* &>/dev/null; then
    echo "✅ Video devices found:"
    ls -la /dev/video*
    echo ""
    
    # Enhanced camera testing with v4l2-ctl
    if command -v v4l2-ctl &> /dev/null; then
        echo "📹 Detailed camera information:"
        v4l2-ctl --list-devices 2>/dev/null
        echo ""
        
        # Test each camera with detailed info
        for video in /dev/video*; do
            if [ -c "$video" ]; then
                echo "Testing $video:"
                
                # Get camera capabilities
                echo "  Capabilities:"
                v4l2-ctl --device="$video" --list-formats-ext 2>/dev/null | head -10 || echo "  ❌ No format info"
                
                # Test camera access and capture
                echo "  Capture test:"
                if timeout 10s ffmpeg -f v4l2 -i "$video" -frames 1 -y test_$(basename $video).jpg >/dev/null 2>&1; then
                    file_size=$(stat -c%s "test_$(basename $video).jpg" 2>/dev/null || echo "0")
                    if [ "$file_size" -gt 10000 ]; then  # At least 10KB for a valid image
                        echo "  ✅ $video works (${file_size} bytes captured)"
                    else
                        echo "  ⚠️  $video captured but small file ($file_size bytes)"
                    fi
                else
                    echo "  ❌ $video failed to capture"
                fi
                
                # Check device permissions
                if [ -r "$video" ] && [ -w "$video" ]; then
                    echo "  ✅ Permissions OK"
                else
                    echo "  ❌ Permission issue - run: sudo chmod 666 $video"
                fi
                echo ""
            fi
        done
    else
        echo "❌ v4l2-ctl not available (install: sudo apt install v4l-utils)"
        echo "Basic camera testing:"
        for video in /dev/video*; do
            if [ -c "$video" ]; then
                if timeout 5s ffmpeg -f v4l2 -i "$video" -frames 1 -y test_$(basename $video).jpg >/dev/null 2>&1; then
                    echo "✅ $video works"
                else
                    echo "❌ $video failed"
                fi
            fi
        done
    fi
    
    # Camera dependency check
    echo "📦 Camera software dependencies:"
    
    # Check OpenCV
    if python3 -c "import cv2; print(f'OpenCV {cv2.__version__}')" 2>/dev/null; then
        echo "✅ OpenCV available"
    else
        echo "❌ OpenCV not available (install: pip3 install opencv-python)"
    fi
    
    # Check cv_bridge
    if python3 -c "from cv_bridge import CvBridge" 2>/dev/null; then
        echo "✅ cv_bridge available"
    else
        echo "❌ cv_bridge not available (install: sudo apt install ros-noetic-cv-bridge)"
    fi
    
    # Check ROS sensor messages
    if python3 -c "from sensor_msgs.msg import Image" 2>/dev/null; then
        echo "✅ ROS sensor_msgs available"
    else
        echo "❌ ROS sensor_msgs not available"
    fi
    
else
    echo "❌ No camera devices found (/dev/video*)"
    echo "Troubleshooting:"
    echo "1. Check USB connections"
    echo "2. Check camera power"
    echo "3. Try different USB ports"
    echo "4. Check dmesg for USB errors: dmesg | grep -i usb | tail -10"
fi
echo ""

# 4. SERIAL DEVICES (LIDAR)
echo "=== 📡 SERIAL DEVICES (LIDAR) ==="
# Load and bind CP210x driver for RPLIDAR
echo "Loading CP210x USB-serial driver for RPLIDAR..."
sudo modprobe cp210x
echo 10c4 ea60 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id 2>/dev/null
sleep 1
echo "Detecting serial devices (/dev/ttyUSB* or /dev/ttyACM*)"
if ls /dev/ttyUSB* /dev/ttyACM* &>/dev/null; then
    echo "Serial devices found:"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
    echo ""
    # LiDAR specific checks
    dmesg | grep -i lidar | tail -5
    lsusb | grep -i rplidar && echo "✅ RPLiDAR detected" || echo "❌ No RPLiDAR in USB"
else
    echo "❌ No serial devices found (/dev/ttyUSB* or /dev/ttyACM*)"
fi
echo ""

# 5. I2C DEVICES
echo "=== 🔌 I2C DEVICES ==="
echo "I2C bus scan:"
sudo i2cdetect -y 1 2>/dev/null || echo "❌ I2C tools not available (install: sudo apt install i2c-tools)"
echo ""
echo "I2C modules loaded:"
lsmod | grep i2c
echo ""
echo "I2C device permissions:"
ls -la /dev/i2c* 2>/dev/null || echo "❌ No I2C devices found"
echo ""

# 6. MOTOR CONTROLLER
echo "=== ⚙️  MOTOR CONTROLLER ==="
echo "Checking for motor controller..."
dmesg | grep -i motor | tail -5
lsusb | grep -i motor && echo "✅ Motor controller found in USB" || echo "❌ No motor controller in USB"
# Check for common motor controller I2C addresses
sudo i2cdetect -y 1 2>/dev/null | grep -E "(48|60|70)" && echo "✅ Possible motor controller on I2C" || echo "❌ No common motor controller I2C addresses"
echo ""

# 7. ROS ENVIRONMENT
echo "=== 🤖 ROS ENVIRONMENT ==="
echo "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH:-Not set}"
echo "ROS_MASTER_URI: ${ROS_MASTER_URI:-Not set}"

if [ -f ~/robot_project/indoor_robot_ws/devel/setup.bash ]; then
    source ~/robot_project/indoor_robot_ws/devel/setup.bash 2>/dev/null && \
    echo "✅ Workspace sourced successfully" || echo "❌ Workspace source failed"
    echo "Workspace ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
else
    echo "❌ Workspace setup.bash not found - did catkin_make succeed?"
fi
echo ""

# 8. ENHANCED CAMERA INTEGRATION TEST
echo "=== 📸 CAMERA INTEGRATION TEST ==="
if ls /dev/video* &>/dev/null && python3 -c "import cv2" 2>/dev/null; then
    echo "Running integrated camera test..."
    
    # Create a simple Python test inline
    python3 << 'EOF'
import cv2
import sys

print("Testing OpenCV camera integration...")
working_cameras = 0

for i in range(4):  # Test indices 0-3
    try:
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # Set resolution matching camera_bridge.py
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print(f"✅ Camera {i}: {w}x{h} resolution")
                working_cameras += 1
            else:
                print(f"❌ Camera {i}: Failed to capture")
            cap.release()
        else:
            pass  # Camera not available, skip silently
    except Exception as e:
        print(f"❌ Camera {i}: Error - {e}")

cv2.destroyAllWindows()
print(f"Result: {working_cameras} working cameras for robot")

if working_cameras >= 2:
    print("✅ Sufficient cameras for dual-camera setup")
elif working_cameras >= 1:
    print("⚠️  Only 1 camera - can work but limited coverage")
else:
    print("❌ No working cameras - check connections")
EOF

else
    echo "❌ Cannot test camera integration (missing OpenCV or no video devices)"
fi
echo ""

# 9. SUMMARY (ENHANCED)
echo "=== 📋 HARDWARE CHECK SUMMARY ==="
echo "Power: $(vcgencmd measure_volts 2>/dev/null | cut -d= -f2 || echo 'Unknown')"
echo "Temperature: $(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo 'Unknown')"

# Enhanced camera summary
camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
microsoft_count=$(lsusb | grep -i microsoft | wc -l)
echo "Cameras: $camera_count video devices detected, $microsoft_count Microsoft devices"

echo "Serial devices: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l) detected"
echo "I2C devices: $(sudo i2cdetect -y 1 2>/dev/null | grep -c '[0-9a-f][0-9a-f]' || echo 0) detected"

# Overall readiness assessment
echo ""
echo "🎯 SYSTEM READINESS ASSESSMENT:"

# Check critical components
opencv_ok=false
python3 -c "import cv2" 2>/dev/null && opencv_ok=true

camera_ok=false
[ "$camera_count" -ge 1 ] && camera_ok=true

ros_ok=false
[ -n "$ROS_PACKAGE_PATH" ] && ros_ok=true

if $opencv_ok && $camera_ok && $ros_ok; then
    echo "✅ SYSTEM READY for camera testing!"
    echo ""
    echo "🚀 Recommended next steps:"
    echo "1. python3 test_camera.py (comprehensive camera test)"
    echo "2. python3 test_emergency_stop.py (safety systems)"
    echo "3. python3 test_motors.py (ONLY if motors connected!)"
    echo "4. roslaunch hardware_interface full_robot.launch (full demo)"
else
    echo "⚠️  SYSTEM NOT READY - fix issues before proceeding:"
    $opencv_ok || echo "❌ Install OpenCV: pip3 install opencv-python"
    $camera_ok || echo "❌ Fix camera connections and permissions"
    $ros_ok || echo "❌ Source ROS workspace: source ~/robot_project/indoor_robot_ws/devel/setup.bash"
fi

echo ""
echo "💡 Quick camera test: cheese (if GUI available)"
echo "💡 Manual camera test: ffmpeg -f v4l2 -i /dev/video0 -frames 1 manual_test.jpg"

# Clean up test images
rm -f test_video*.jpg 2>/dev/null
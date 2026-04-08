
#!/usr/bin/env python3
"""
Phase 1 Test: Dependencies and Environment Check
Tests all required dependencies before hardware testing

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_dependencies.py
Usage: python3 test_dependencies.py
"""

import sys
import subprocess
import importlib
import os
import glob

class DependencyTester:
    def __init__(self):
        self.passed_tests = 0
        self.failed_tests = 0
        self.test_results = []

    def test_result(self, test_name, passed, message=""):
        """Record test result"""
        if passed:
            self.passed_tests += 1
            status = "✅ PASS"
            print(f"{status}: {test_name}")
        else:
            self.failed_tests += 1
            status = "❌ FAIL"
            print(f"{status}: {test_name}")
            if message:
                print(f"    Error: {message}")
        
        self.test_results.append({
            'test': test_name,
            'passed': passed,
            'message': message
        })

    def test_python_packages(self):
        """Test required Python packages"""
        print("\n=== Testing Python Dependencies ===")
        
        required_packages = {
            'rospy': 'ROS Python library',
            'gpiozero': 'GPIO control library',
            'rplidar': 'RPLidar Python library',
            'numpy': 'Numerical computation library',
            'tf2_ros': 'ROS TF2 library',
            'sensor_msgs.msg': 'ROS sensor messages',
            'geometry_msgs.msg': 'ROS geometry messages',
            'nav_msgs.msg': 'ROS navigation messages',
            'std_msgs.msg': 'ROS standard messages'
        }
        
        for package, description in required_packages.items():
            try:
                # Import module or submodule directly
                importlib.import_module(package)
                self.test_result(f"Python package: {package}", True)
            except ImportError as e:
                self.test_result(f"Python package: {package}", False,
                                 f"{description} - {str(e)}")

    def test_ros_environment(self):
        """Test ROS environment setup"""
        print("\n=== Testing ROS Environment ===")
        
        # Check ROS_MASTER_URI
        ros_master = os.environ.get('ROS_MASTER_URI')
        if ros_master:
            self.test_result("ROS_MASTER_URI set", True, f"Value: {ros_master}")
        else:
            self.test_result("ROS_MASTER_URI set", False, "Environment variable not set")
        
        # Check ROS_PACKAGE_PATH
        ros_path = os.environ.get('ROS_PACKAGE_PATH')
        if ros_path and 'indoor_robot_ws' in ros_path:
            self.test_result("ROS workspace sourced", True)
        else:
            self.test_result("ROS workspace sourced", False, "Workspace not in ROS_PACKAGE_PATH")
        
        # Check if roscore is available
        try:
            result = subprocess.run(['which', 'roscore'], capture_output=True, text=True)
            if result.returncode == 0:
                self.test_result("roscore available", True)
            else:
                self.test_result("roscore available", False, "roscore command not found")
        except Exception as e:
            self.test_result("roscore available", False, str(e))

    def test_ros_packages(self):
        """Test required ROS packages"""
        print("\n=== Testing ROS Packages ===")
        
        required_ros_packages = [
            'gmapping',
            'move_base',
            'explore_lite',
            'robot_localization',
            'tf2_ros',
            'navigation'
        ]
        
        for package in required_ros_packages:
            try:
                result = subprocess.run(['rospack', 'find', package], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    self.test_result(f"ROS package: {package}", True)
                else:
                    self.test_result(f"ROS package: {package}", False, 
                                   f"Package not found. Install with: sudo apt install ros-noetic-{package.replace('_', '-')}")
            except Exception as e:
                self.test_result(f"ROS package: {package}", False, str(e))

    def test_hardware_interfaces(self):
        """Test hardware interface availability"""
        print("\n=== Testing Hardware Interfaces ===")
        
        # Check GPIO access
        try:
            import gpiozero
            # Try to create a simple device to test permissions
            test_led = gpiozero.LED(2)  # Use a safe pin
            test_led.close()
            self.test_result("GPIO access", True)
        except Exception as e:
            self.test_result("GPIO access", False, 
                           f"GPIO permission issue: {str(e)}. Run: sudo usermod -a -G gpio $USER")
        
        # Check I2C interface
        i2c_devices = glob.glob('/dev/i2c-*')
        if i2c_devices:
            self.test_result("I2C interface available", True, f"Found: {i2c_devices}")
        else:
            self.test_result("I2C interface available", False, 
                           "No I2C devices found. Enable I2C with raspi-config")
        
        # Check USB serial devices (for LiDAR)
        usb_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if usb_devices:
            self.test_result("USB serial devices", True, f"Found: {usb_devices}")
        else:
            self.test_result("USB serial devices", False, 
                           "No USB serial devices found. Check LiDAR connection")

    def test_workspace_build(self):
        """Test if workspace is properly built"""
        print("\n=== Testing Workspace Build ===")
        
        # Check if devel/setup.bash exists
        setup_file = os.path.expanduser("~/robot_project/indoor_robot_ws/devel/setup.bash")
        if os.path.exists(setup_file):
            self.test_result("Workspace built", True)
        else:
            self.test_result("Workspace built", False, 
                           "Run 'catkin_make' in ~/robot_project/indoor_robot_ws")
        
        # Check if our packages are available
        try:
            result = subprocess.run(['rospack', 'find', 'hardware_interface'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.test_result("hardware_interface package found", True)
            else:
                self.test_result("hardware_interface package found", False, 
                               "Package not found in ROS path")
        except Exception as e:
            self.test_result("hardware_interface package found", False, str(e))

    def test_script_permissions(self):
        """Test if scripts are executable"""
        print("\n=== Testing Script Permissions ===")
        
        script_paths = [
            "~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/motor_bridge.py",
            "~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/lidar_bridge.py",
            "~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/imu_bridge.py",
            "~/robot_project/indoor_robot_ws/src/hardware_interface/scripts/emergency_stop.py"
        ]
        
        for script_path in script_paths:
            full_path = os.path.expanduser(script_path)
            if os.path.exists(full_path):
                if os.access(full_path, os.X_OK):
                    self.test_result(f"Script executable: {os.path.basename(full_path)}", True)
                else:
                    self.test_result(f"Script executable: {os.path.basename(full_path)}", False, 
                                   f"Run: chmod +x {full_path}")
            else:
                self.test_result(f"Script exists: {os.path.basename(full_path)}", False, 
                               "Script file not found")

    def print_summary(self):
        """Print test summary"""
        print("\n" + "="*50)
        print("DEPENDENCY TEST SUMMARY")
        print("="*50)
        print(f"Passed: {self.passed_tests}")
        print(f"Failed: {self.failed_tests}")
        print(f"Total:  {self.passed_tests + self.failed_tests}")
        
        if self.failed_tests == 0:
            print("\n🎉 ALL DEPENDENCIES READY! You can proceed to hardware testing.")
        else:
            print(f"\n⚠️  {self.failed_tests} dependencies need attention before hardware testing.")
            print("\nFailed tests:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  - {result['test']}: {result['message']}")
        
        return self.failed_tests == 0


def main():
    print("Phase 1 Test: Dependencies and Environment Check")
    print("="*50)
    
    tester = DependencyTester()
    
    # Run all dependency tests
    tester.test_python_packages()
    tester.test_ros_environment()
    tester.test_ros_packages()
    tester.test_hardware_interfaces()
    tester.test_workspace_build()
    tester.test_script_permissions()
    
    # Print summary and exit with appropriate code
    all_passed = tester.print_summary()
    sys.exit(0 if all_passed else 1)

if __name__ == "__main__":
    main()


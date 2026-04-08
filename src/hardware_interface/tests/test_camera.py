#!/usr/bin/env python3
"""
Phase 1 Test: Camera Hardware Test
Tests USB camera connection, image capture, and ROS integration before full system testing

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_camera.py
Usage: python3 test_camera.py

This test validates:
1. USB camera detection and access
2. OpenCV camera initialization
3. Image capture quality and timing
4. ROS camera_bridge integration
5. Photo trigger functionality
"""

import sys
import time
import os
import cv2
import glob
import subprocess
import json
from datetime import datetime

class CameraTester:
    def __init__(self):
        self.test_results = []
        self.cameras = []
        self.test_directory = "/tmp/camera_test_images"
        
        # Ensure test directory exists
        os.makedirs(self.test_directory, exist_ok=True)
        
    def log_result(self, test_name, passed, message=""):
        """Log test result"""
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

    def test_opencv_import(self):
        """Test if OpenCV can be imported"""
        print("\n=== Testing OpenCV Library ===")
        try:
            import cv2
            version = cv2.__version__
            self.log_result("OpenCV import", True, f"Version: {version}")
            return True
        except ImportError as e:
            self.log_result("OpenCV import", False, 
                           "Install with: pip3 install opencv-python")
            return False

    def test_usb_camera_detection(self):
        """Test for USB camera devices"""
        print("\n=== Testing USB Camera Detection ===")
        
        # Check for video devices
        video_devices = glob.glob('/dev/video*')
        
        if video_devices:
            self.log_result("USB camera devices found", True, f"Devices: {video_devices}")
            
            # Test device permissions
            for device in video_devices:
                try:
                    with open(device, 'rb') as f:
                        pass
                    self.log_result(f"Device access: {device}", True, "Read permission OK")
                except PermissionError:
                    self.log_result(f"Device access: {device}", False, 
                                   f"Permission denied. Run: sudo chmod 666 {device}")
                except Exception as e:
                    self.log_result(f"Device access: {device}", False, str(e))
            
            return True
        else:
            self.log_result("USB camera devices found", False, 
                           "No camera devices found. Check camera connections")
            return False

    def test_camera_initialization(self):
        """Test camera initialization with OpenCV"""
        print("\n=== Testing Camera Initialization ===")
        
        if not cv2:
            return False
        
        # Test multiple camera indices (matching your hardware setup)
        camera_indices = [0, 2]  # Test up to 4 cameras
        working_cameras = []
        
        for i in camera_indices:
            try:
                print(f"Testing camera index {i}...")
                
                cap = cv2.VideoCapture(i)
                
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FOURCC,
+                            cv2.VideoWriter_fourcc(*'MJPG'))
                    # Set resolution to match your camera_bridge.py settings
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 85)
                    
                    # Test capture
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        working_cameras.append({
                            'index': i,
                            'cap': cap,
                            'resolution': frame.shape,
                            'device': f"/dev/video{i}"
                        })
                        self.log_result(f"Camera {i} initialization", True, 
                                      f"Resolution: {frame.shape}")
                    else:
                        cap.release()
                        self.log_result(f"Camera {i} capture test", False, 
                                      "Failed to capture test frame")
                else:
                    self.log_result(f"Camera {i} initialization", False, 
                                  "Failed to open camera")
                    
            except Exception as e:
                self.log_result(f"Camera {i} initialization", False, f"Error: {str(e)}")
                continue
        
        self.cameras = working_cameras
        
        if working_cameras:
            self.log_result("Camera system initialization", True, 
                          f"{len(working_cameras)} cameras available")
            return True
        else:
            self.log_result("Camera system initialization", False, 
                          "No working cameras found")
            return False

    def test_image_capture_quality(self):
        """Test image capture quality and parameters"""
        if not self.cameras:
            return False
            
        print("\n=== Testing Image Capture Quality ===")
        
        try:
            for camera_info in self.cameras:
                camera_index = camera_info['index']
                cap = camera_info['cap']
                
                print(f"Testing image quality for camera {camera_index}...")
                
                # Capture multiple frames to ensure stable image
                for warm_up in range(5):  # Warm-up frames
                    ret, frame = cap.read()
                
                # Capture test image
                ret, frame = cap.read()
                
                if ret and frame is not None:
                    # Check image properties
                    height, width, channels = frame.shape
                    
                    # Check for reasonable image properties
                    if width >= 640 and height >= 480:
                        self.log_result(f"Camera {camera_index} resolution", True, 
                                      f"{width}x{height}")
                    else:
                        self.log_result(f"Camera {camera_index} resolution", False, 
                                      f"Low resolution: {width}x{height}")
                    
                    # Check for color channels
                    if channels == 3:
                        self.log_result(f"Camera {camera_index} color", True, "RGB color")
                    else:
                        self.log_result(f"Camera {camera_index} color", False, 
                                      f"Unexpected channels: {channels}")
                    
                    # Check image brightness (simple test)
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    mean_brightness = cv2.mean(gray)[0]
                    
                    if 20 <= mean_brightness <= 235:  # Reasonable brightness range
                        self.log_result(f"Camera {camera_index} exposure", True, 
                                      f"Good brightness: {mean_brightness:.1f}")
                    else:
                        self.log_result(f"Camera {camera_index} exposure", False, 
                                      f"Poor exposure: {mean_brightness:.1f}")
                    
                    # Save test image
                    test_filename = f"camera_{camera_index}_test.jpg"
                    test_filepath = os.path.join(self.test_directory, test_filename)
                    success = cv2.imwrite(test_filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    
                    if success:
                        self.log_result(f"Camera {camera_index} image save", True, 
                                      f"Saved: {test_filepath}")
                    else:
                        self.log_result(f"Camera {camera_index} image save", False, 
                                      "Failed to save image")
                else:
                    self.log_result(f"Camera {camera_index} capture", False, 
                                  "Failed to capture image")
            
            return True
            
        except Exception as e:
            self.log_result("Image capture quality test", False, f"Error: {str(e)}")
            return False

    def test_capture_timing(self):
        """Test image capture timing and consistency"""
        if not self.cameras:
            return False
            
        print("\n=== Testing Capture Timing ===")
        
        try:
            # Test with first camera
            camera_info = self.cameras[0]
            cap = camera_info['cap']
            
            capture_times = []
            num_captures = 5
            
            print(f"Capturing {num_captures} timed images...")
            
            for i in range(num_captures):
                start_time = time.time()
                ret, frame = cap.read()
                end_time = time.time()
                
                if ret and frame is not None:
                    capture_duration = end_time - start_time
                    capture_times.append(capture_duration)
                    print(f"  Capture {i+1}: {capture_duration:.3f}s")
                else:
                    self.log_result("Capture timing consistency", False, 
                                  f"Failed to capture image {i+1}")
                    return False
                
                time.sleep(0.5)  # Wait between captures
            
            # Analyze timing
            avg_time = sum(capture_times) / len(capture_times)
            max_time = max(capture_times)
            min_time = min(capture_times)
            
            if avg_time < 0.5:  # Should capture reasonably quickly
                self.log_result("Capture speed", True, 
                              f"Average: {avg_time:.3f}s (range: {min_time:.3f}-{max_time:.3f}s)")
            else:
                self.log_result("Capture speed", False, 
                              f"Slow capture: {avg_time:.3f}s average")
            
            # Check timing consistency
            time_variance = max_time - min_time
            if time_variance < 0.2:  # Should be fairly consistent
                self.log_result("Capture timing consistency", True, 
                              f"Consistent timing (variance: {time_variance:.3f}s)")
            else:
                self.log_result("Capture timing consistency", False, 
                              f"Inconsistent timing (variance: {time_variance:.3f}s)")
            
            return True
            
        except Exception as e:
            self.log_result("Capture timing test", False, f"Error: {str(e)}")
            return False

    def test_microsoft_lifecam_detection(self):
        """Test specifically for Microsoft LifeCam Studio cameras (from your specs)"""
        print("\n=== Testing Microsoft LifeCam Detection ===")
        
        try:
            # Check USB devices for Microsoft cameras
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            if result.returncode == 0:
                usb_output = result.stdout.lower()
                
                if 'microsoft' in usb_output:
                    self.log_result("Microsoft cameras detected", True, 
                                  "Found Microsoft devices in USB")
                    
                    # Look for LifeCam specifically
                    if 'lifecam' in usb_output:
                        self.log_result("Microsoft LifeCam detected", True, 
                                      "LifeCam Studio found")
                    else:
                        self.log_result("Microsoft LifeCam detected", False, 
                                      "Microsoft device found but not LifeCam")
                else:
                    self.log_result("Microsoft cameras detected", False, 
                                  "No Microsoft devices found in USB")
            else:
                self.log_result("USB device detection", False, "lsusb command failed")
            
            return True
            
        except Exception as e:
            self.log_result("Microsoft LifeCam detection", False, f"Error: {str(e)}")
            return False

    def test_dual_camera_capture(self):
        """Test simultaneous capture from multiple cameras (matching your setup)"""
        if len(self.cameras) < 2:
            self.log_result("Dual camera test", False, "Need at least 2 cameras for this test")
            return False
            
        print("\n=== Testing Dual Camera Capture ===")
        
        try:
            print("Testing simultaneous capture from 2 cameras...")
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            captured_files = []
            
            # Capture from first two cameras simultaneously
            for i, camera_info in enumerate(self.cameras[:2]):
                cap = camera_info['cap']
                
                ret, frame = cap.read()
                if ret and frame is not None:
                    filename = f"dual_test_{timestamp}_camera_{i}.jpg"
                    filepath = os.path.join(self.test_directory, filename)
                    
                    success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if success:
                        captured_files.append(filepath)
                        print(f"  Captured: {filename}")
                    else:
                        self.log_result("Dual camera capture", False, 
                                      f"Failed to save image from camera {i}")
                        return False
                else:
                    self.log_result("Dual camera capture", False, 
                                  f"Failed to capture from camera {i}")
                    return False
            
            if len(captured_files) == 2:
                self.log_result("Dual camera capture", True, 
                              f"Successfully captured from 2 cameras")
                
                # Create metadata file (matching camera_bridge.py format)
                metadata = {
                    'timestamp': timestamp,
                    'test_type': 'dual_camera_test',
                    'files': captured_files,
                    'camera_count': len(captured_files)
                }
                
                metadata_file = os.path.join(self.test_directory, f"dual_test_{timestamp}_metadata.json")
                with open(metadata_file, 'w') as f:
                    json.dump(metadata, f, indent=2)
                
                self.log_result("Metadata generation", True, f"Saved: {metadata_file}")
                return True
            else:
                self.log_result("Dual camera capture", False, 
                              f"Only captured {len(captured_files)} images")
                return False
            
        except Exception as e:
            self.log_result("Dual camera capture test", False, f"Error: {str(e)}")
            return False

    def test_camera_bridge_compatibility(self):
        """Test compatibility with camera_bridge.py settings"""
        print("\n=== Testing Camera Bridge Compatibility ===")
        
        try:
            # Test the resolution settings from camera_bridge.py
            target_width = 1280
            target_height = 720
            
            if self.cameras:
                camera_info = self.cameras[0]
                cap = camera_info['cap']
                
                # Set target resolution
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
                
                # Capture with target settings
                ret, frame = cap.read()
                
                if ret and frame is not None:
                    actual_height, actual_width = frame.shape[:2]
                    
                    if actual_width >= target_width * 0.8 and actual_height >= target_height * 0.8:
                        self.log_result("Camera bridge resolution", True, 
                                      f"Resolution: {actual_width}x{actual_height}")
                    else:
                        self.log_result("Camera bridge resolution", False, 
                                      f"Low resolution: {actual_width}x{actual_height}")
                    
                    # Test JPEG quality settings
                    test_filename = "bridge_compat_test.jpg"
                    test_filepath = os.path.join(self.test_directory, test_filename)
                    
                    # Use same JPEG quality as camera_bridge.py
                    success = cv2.imwrite(test_filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    
                    if success:
                        # Check file size (should be reasonable for 1280x720 at 85% quality)
                        file_size = os.path.getsize(test_filepath)
                        file_size_kb = file_size / 1024
                        
                        if 50 <= file_size_kb <= 500:  # Reasonable range
                            self.log_result("JPEG quality test", True, 
                                          f"File size: {file_size_kb:.1f}KB")
                        else:
                            self.log_result("JPEG quality test", False, 
                                          f"Unusual file size: {file_size_kb:.1f}KB")
                    else:
                        self.log_result("JPEG quality test", False, "Failed to save JPEG")
                else:
                    self.log_result("Camera bridge compatibility", False, 
                                  "Failed to capture with bridge settings")
                    return False
            else:
                self.log_result("Camera bridge compatibility", False, "No cameras available")
                return False
            
            return True
            
        except Exception as e:
            self.log_result("Camera bridge compatibility", False, f"Error: {str(e)}")
            return False

    def test_ros_integration_readiness(self):
        """Test readiness for ROS camera_bridge integration"""
        print("\n=== Testing ROS Integration Readiness ===")
        
        try:
            # Check if we can import ROS modules
            try:
                import rospy
                from sensor_msgs.msg import Image, CompressedImage
                from std_msgs.msg import Bool
                self.log_result("ROS imports", True, "ROS modules available")
            except ImportError as e:
                self.log_result("ROS imports", False, f"ROS not available: {str(e)}")
                return False
            
            # Test cv_bridge compatibility
            try:
                from cv_bridge import CvBridge
                bridge = CvBridge()
                
                if self.cameras:
                    cap = self.cameras[0]['cap']
                    ret, frame = cap.read()
                    
                    if ret and frame is not None:
                        # Test conversion to ROS Image message
                        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header.stamp = rospy.Time.now() if rospy.Time else None
                        ros_image.header.frame_id = "camera_0"
                        
                        self.log_result("cv_bridge conversion", True, "OpenCV to ROS conversion works")
                    else:
                        self.log_result("cv_bridge conversion", False, "No frame to convert")
                else:
                    self.log_result("cv_bridge conversion", False, "No cameras for testing")
                
            except ImportError:
                self.log_result("cv_bridge import", False, "Install: sudo apt install ros-noetic-cv-bridge")
            except Exception as e:
                self.log_result("cv_bridge conversion", False, f"Conversion error: {str(e)}")
            
            return True
            
        except Exception as e:
            self.log_result("ROS integration test", False, f"Error: {str(e)}")
            return False

    def test_trigger_simulation(self):
        """Simulate camera trigger functionality (matching camera_bridge.py)"""
        print("\n=== Testing Trigger Simulation ===")
        
        if not self.cameras:
            return False
        
        try:
            print("Simulating camera trigger events...")
            
            # Simulate trigger-based capture (like camera_bridge.py)
            trigger_events = [
                {"name": "startup_trigger", "delay": 0.0},
                {"name": "navigation_trigger", "delay": 1.0},
                {"name": "waypoint_trigger", "delay": 2.0}
            ]
            
            captured_images = []
            
            for event in trigger_events:
                print(f"Simulating {event['name']}...")
                time.sleep(event['delay'])
                
                # Capture from all available cameras
                event_images = []
                for i, camera_info in enumerate(self.cameras):
                    cap = camera_info['cap']
                    ret, frame = cap.read()
                    
                    if ret and frame is not None:
                        filename = f"trigger_{event['name']}_camera_{i}.jpg"
                        filepath = os.path.join(self.test_directory, filename)
                        
                        success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if success:
                            event_images.append(filepath)
                        
                captured_images.extend(event_images)
                print(f"  Captured {len(event_images)} images for {event['name']}")
            
            if len(captured_images) >= len(trigger_events):
                self.log_result("Trigger simulation", True, 
                              f"Captured {len(captured_images)} triggered images")
            else:
                self.log_result("Trigger simulation", False, 
                              f"Only captured {len(captured_images)} images")
            
            return True
            
        except Exception as e:
            self.log_result("Trigger simulation", False, f"Error: {str(e)}")
            return False

    def cleanup(self):
        """Clean up camera resources"""
        try:
            for camera_info in self.cameras:
                camera_info['cap'].release()
            cv2.destroyAllWindows()
            print("\nCamera resources cleaned up")
        except Exception as e:
            print(f"Cleanup warning: {e}")

    def print_summary(self):
        """Print comprehensive test summary"""
        passed = sum(1 for result in self.test_results if result['passed'])
        failed = len(self.test_results) - passed
        
        print("\n" + "="*60)
        print("CAMERA HARDWARE TEST SUMMARY")
        print("="*60)
        print(f"Tests Run:  {len(self.test_results)}")
        print(f"Passed:     {passed}")
        print(f"Failed:     {failed}")
        print(f"Success:    {(passed/len(self.test_results)*100):.1f}%" if self.test_results else "0%")
        
        if failed == 0:
            print("\n🎉 ALL CAMERA TESTS PASSED!")
            print("\n✅ Camera System Ready:")
            print(f"  • {len(self.cameras)} working cameras detected")
            print(f"  • Images saved to: {self.test_directory}")
            print(f"  • Compatible with camera_bridge.py settings")
            print(f"  • ROS integration ready")
            print(f"  • Trigger simulation successful")
            
            print("\n📸 Camera Details:")
            for camera_info in self.cameras:
                print(f"  • Camera {camera_info['index']}: {camera_info['resolution']}")
            
            print("\n🔄 Next Steps:")
            print("1. Test camera_bridge.py ROS node")
            print("2. Integrate with indoor_mapper.py")
            print("3. Test photo capture during navigation")
            print("4. Verify image upload to server")
            
        else:
            print(f"\n⚠️  {failed} camera tests failed.")
            print(f"\n❌ Failed Tests:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  • {result['test']}: {result['message']}")
            
            print(f"\n🔧 Troubleshooting Steps:")
            print("1. Check camera connections and power")
            print("2. Install missing dependencies (OpenCV, ROS)")
            print("3. Check device permissions: sudo chmod 666 /dev/video*")
            print("4. Test cameras individually with: cheese or guvcview")
            print("5. Verify Microsoft LifeCam Studio drivers")
        
        print(f"\n📊 Test completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"📁 Test images saved to: {self.test_directory}")
        
        return failed == 0

def main():
    print("Phase 1 Test: Camera Hardware Verification")
    print("="*60)
    print("Testing USB camera system for indoor mapping robot")
    print("Based on Microsoft LifeCam Studio specifications")
    print("MAKE SURE CAMERAS ARE CONNECTED!")
    print("="*60)
    
    # Safety and preparation prompt
    print("\n📋 PRE-TEST CHECKLIST:")
    print("1. USB cameras connected and powered?")
    print("2. Camera permissions set (if needed)?")
    print("3. Good lighting for image quality test?")
    print("4. Enough disk space for test images?")
    
    response = input("\nAll checks complete - proceed with camera testing? (y/N): ")
    if response.lower() != 'y':
        print("Test cancelled. Complete setup and run again when ready.")
        sys.exit(1)
    
    tester = CameraTester()
    
    try:
        print(f"\nStarting comprehensive camera tests...")
        print(f"Test images will be saved to: {tester.test_directory}")
        
        # Run all tests in sequence
        if not tester.test_opencv_import():
            tester.print_summary()
            sys.exit(1)
        
        if not tester.test_usb_camera_detection():
            tester.print_summary()
            sys.exit(1)
        
        # Hardware-specific tests
        tester.test_microsoft_lifecam_detection()
        
        if not tester.test_camera_initialization():
            tester.print_summary()
            sys.exit(1)
        
        # Quality and functionality tests
        tester.test_image_capture_quality()
        tester.test_capture_timing()
        tester.test_dual_camera_capture()
        
        # Integration readiness
        tester.test_camera_bridge_compatibility()
        tester.test_ros_integration_readiness()
        tester.test_trigger_simulation()
        
        # Print results
        all_passed = tester.print_summary()
        sys.exit(0 if all_passed else 1)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        tester.cleanup()
        sys.exit(1)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        tester.cleanup()
        sys.exit(1)
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()

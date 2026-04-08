#!/usr/bin/env python3
"""
Camera Bridge Node - Working Implementation for Demo

Based on your hardware check showing:
- Multiple camera devices detected (/dev/video0, /dev/video1, etc.)
- Microsoft LifeCam Studio webcams (from design docs)
- USB camera integration needed for indoor_mapper

This node:
1. Interfaces with USB cameras (Microsoft LifeCam Studio)
2. Captures images when triggered by indoor_mapper
3. Saves images with timestamp and pose metadata
4. Provides status feedback for the mapping system

DEMO MODE: Focused on reliable triggering and status feedback
"""

import rospy
import cv2
import os
import json
import time
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class CameraBridge:
    def __init__(self):
        rospy.init_node('camera_bridge', anonymous=True)
        
        # Camera configuration (from your hardware check and design docs)
        self.camera_devices = rospy.get_param('~camera_devices', ['/dev/video0', '/dev/video2'])
        self.image_width = rospy.get_param('~image_width', 1280)    # Reduced for stability
        self.image_height = rospy.get_param('~image_height', 720)   # Reduced for stability  
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 85)
        
        # Photo capture parameters
        self.photo_directory = rospy.get_param('~photo_directory', '/tmp/robot_photos')
        self.auto_capture_enabled = rospy.get_param('~auto_capture_enabled', False)
        
        # Ensure photo directory exists
        os.makedirs(self.photo_directory, exist_ok=True)
        
        # Camera state tracking
        self.cameras = []
        self.cameras_available = False
        self.last_pose = None
        self.capture_count = 0
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # ROS publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.compressed_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=1)
        self.photo_status_pub = rospy.Publisher('/camera/photo_status', String, queue_size=1, latch=True)
        
        # ROS subscribers
        self.capture_trigger_sub = rospy.Subscriber('/camera/capture_trigger', Bool, self.capture_trigger_callback)
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        
        # Initialize camera hardware
        self.setup_cameras()
        
        # Status timer
        rospy.Timer(rospy.Duration(10.0), self.publish_status)
        
        rospy.loginfo("Camera Bridge initialized for DEMO")
        rospy.loginfo(f"Photo directory: {self.photo_directory}")
        rospy.loginfo(f"Cameras available: {self.cameras_available}")

    def setup_cameras(self):
        """
        Initialize camera hardware using OpenCV
        
        Based on your hardware check showing camera devices detected
        """
        
        try:
            import cv2
            rospy.loginfo("Setting up cameras...")
            
            # Test each camera device from your hardware check
            working_cameras = []
            
            for i, device in enumerate(self.camera_devices):
                try:
                    rospy.loginfo(f"Testing camera device: {device}")
                    
                    # Try to open camera
                    if device.startswith('/dev/video'):
                        # Extract device number from /dev/videoX
                        device_num = int(device.split('video')[1])
                    else:
                        device_num = i
                    
                    cap = cv2.VideoCapture(device_num)
                    
                    if cap.isOpened():
                        # Set resolution (conservative for stability)
                        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                        # cap.set(cv2.CAP_PROP_FPS, 15)  # Conservative frame rate
                        # import time
                        # time.sleep(4)
                        # # Test capture
                        # ret, frame = cap.read()
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Lower resolution first
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 10)            # Lower FPS
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Reduce buffer
                        
                        # Give camera time to initialize
                        import time
                        time.sleep(3)  # Extra time for video2
                        
                        # Try multiple capture attempts
                        ret, frame = None, None
                        for attempt in range(5):
                            rospy.loginfo(f"Camera {device} capture attempt {attempt+1}...")
                            ret, frame = cap.read()
                            if ret and frame is not None:
                                rospy.loginfo(f"✅ Camera {device} success on attempt {attempt+1}")
                                break
                            time.sleep(1)
                        if ret and frame is not None:
                            working_cameras.append({
                                'device': device,
                                'device_num': device_num,
                                'cap': cap,
                                'name': f"camera_{i}"
                            })
                            rospy.loginfo(f"✅ Camera {device} working - Resolution: {frame.shape}")
                        else:
                            cap.release()
                            rospy.logwarn(f"❌ Camera {device} - failed to capture test frame")
                    else:
                        rospy.logwarn(f"❌ Camera {device} - failed to open")
                        
                except Exception as e:
                    rospy.logwarn(f"❌ Camera {device} - error: {e}")
                    continue
            
            self.cameras = working_cameras
            self.cameras_available = len(working_cameras) > 0
            
            if self.cameras_available:
                rospy.loginfo(f"✅ {len(working_cameras)} cameras initialized successfully")
                for cam in working_cameras:
                    rospy.loginfo(f"  - {cam['name']}: {cam['device']}")
            else:
                rospy.logwarn("❌ No working cameras found - will run in simulation mode")
                
        except ImportError:
            rospy.logerr("OpenCV not available - install with: pip3 install opencv-python")
            self.cameras_available = False
        except Exception as e:
            rospy.logerr(f"Failed to initialize cameras: {e}")
            self.cameras_available = False

    def capture_trigger_callback(self, msg):
        """Handle photo capture trigger from indoor_mapper"""
        if msg.data:
            rospy.logwarn("📷 Camera capture triggered by indoor_mapper!")
            self.capture_photos()

    def pose_callback(self, msg):
        """Store robot pose for photo metadata"""
        self.last_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }

    def capture_photos(self):
        """
        Capture photos from all available cameras
        
        For demo: Captures from working cameras and saves with metadata
        """
        
        if not self.cameras_available:
            rospy.logwarn("📸 DEMO MODE: Simulating photo capture (no cameras available)")
            self.simulate_photo_capture()
            return
        
        try:
            timestamp = datetime.now()
            session_id = f"demo_{int(timestamp.timestamp())}"
            
            rospy.logwarn(f"📸 Capturing photos from {len(self.cameras)} cameras...")
            
            captured_files = []
            
            for i, camera in enumerate(self.cameras):
                try:
                    # Capture frame
                    ret, frame = camera['cap'].read()
                    
                    if ret and frame is not None:
                        # Generate filename
                        filename = f"{session_id}_{camera['name']}_{self.capture_count:03d}.jpg"
                        filepath = os.path.join(self.photo_directory, filename)
                        
                        # Save image
                        success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                        
                        if success:
                            captured_files.append(filepath)
                            rospy.loginfo(f"✅ Saved: {filename}")
                            
                            # Publish ROS image message for monitoring
                            try:
                                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                                ros_image.header.stamp = rospy.Time.now()
                                ros_image.header.frame_id = f"camera_{i}"
                                self.image_pub.publish(ros_image)
                            except Exception as e:
                                rospy.logwarn(f"Failed to publish ROS image: {e}")
                        else:
                            rospy.logwarn(f"❌ Failed to save image from {camera['name']}")
                    else:
                        rospy.logwarn(f"❌ Failed to capture from {camera['name']}")
                        
                except Exception as e:
                    rospy.logwarn(f"❌ Error capturing from {camera['name']}: {e}")
                    continue
            
            # Save metadata
            if captured_files:
                self.save_photo_metadata(session_id, captured_files, timestamp)
                
            self.capture_count += 1
            
            # Publish status
            status_msg = f"Captured {len(captured_files)} photos at session {session_id}"
            self.photo_status_pub.publish(String(status_msg))
            rospy.logwarn(f"📸 Photo capture complete: {len(captured_files)} images saved")
            
        except Exception as e:
            error_msg = f"Photo capture failed: {e}"
            rospy.logerr(error_msg)
            self.photo_status_pub.publish(String(error_msg))

    def simulate_photo_capture(self):
        """
        Simulate photo capture when no cameras available
        For testing and demo purposes
        """
        timestamp = datetime.now()
        session_id = f"demo_sim_{int(timestamp.timestamp())}"
        
        rospy.logwarn("📸 SIMULATION: Creating placeholder photo files...")
        
        # Create placeholder files for demo
        placeholder_files = []
        for i in range(2):  # Simulate 2 cameras
            filename = f"{session_id}_camera_{i}_{self.capture_count:03d}.txt"
            filepath = os.path.join(self.photo_directory, filename)
            
            with open(filepath, 'w') as f:
                f.write(f"DEMO PLACEHOLDER IMAGE\n")
                f.write(f"Camera: {i}\n")
                f.write(f"Timestamp: {timestamp}\n")
                f.write(f"Capture count: {self.capture_count}\n")
                if self.last_pose:
                    f.write(f"Robot pose: {self.last_pose}\n")
            
            placeholder_files.append(filepath)
            rospy.loginfo(f"📝 Created placeholder: {filename}")
        
        # Save metadata
        self.save_photo_metadata(session_id, placeholder_files, timestamp)
        
        self.capture_count += 1
        
        # Publish status
        status_msg = f"SIMULATION: Created {len(placeholder_files)} placeholder photos"
        self.photo_status_pub.publish(String(status_msg))

    def save_photo_metadata(self, session_id, file_paths, timestamp):
        """Save photo metadata as JSON for virtual tour generation"""
        try:
            metadata = {
                'session_id': session_id,
                'timestamp': timestamp.isoformat(),
                'capture_count': self.capture_count,
                'files': file_paths,
                'robot_pose': self.last_pose,
                'camera_config': {
                    'width': self.image_width,
                    'height': self.image_height,
                    'quality': self.jpeg_quality,
                    'num_cameras': len(self.cameras) if self.cameras_available else 2
                }
            }
            
            metadata_file = os.path.join(self.photo_directory, f"{session_id}_metadata.json")
            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            rospy.loginfo(f"📋 Metadata saved: {metadata_file}")
            
        except Exception as e:
            rospy.logwarn(f"Failed to save metadata: {e}")

    def publish_status(self, event):
        """Periodic status publishing"""
        if self.cameras_available:
            status = f"Camera bridge active - {len(self.cameras)} cameras ready, {self.capture_count} captures completed"
        else:
            status = f"Camera bridge active - SIMULATION MODE, {self.capture_count} captures completed"
        
        self.photo_status_pub.publish(String(status))

    def cleanup(self):
        """Cleanup camera resources on shutdown"""
        try:
            rospy.loginfo("Cleaning up camera resources...")
            
            for camera in self.cameras:
                try:
                    camera['cap'].release()
                    rospy.loginfo(f"Released {camera['name']}")
                except Exception as e:
                    rospy.logwarn(f"Error releasing {camera['name']}: {e}")
            
            # OpenCV cleanup
            cv2.destroyAllWindows()
            rospy.loginfo("Camera bridge cleanup complete")
            
        except Exception as e:
            rospy.logwarn(f"Error during camera cleanup: {e}")

if __name__ == '__main__':
    try:
        camera_bridge = CameraBridge()
        
        # Register cleanup function
        rospy.on_shutdown(camera_bridge.cleanup)
        
        rospy.loginfo("Camera bridge ready for DEMO")
        rospy.loginfo("🎬 Waiting for capture triggers from indoor_mapper...")
        rospy.loginfo("📁 Photos will be saved to: /tmp/robot_photos")
        
        # Test trigger info
        rospy.loginfo("Manual test: rostopic pub /camera/capture_trigger std_msgs/Bool 'data: true'")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera bridge shutting down")
    except Exception as e:
        rospy.logerr(f"Camera bridge error: {e}")
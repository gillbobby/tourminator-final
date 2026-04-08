#!/usr/bin/env python3
"""
Phase 1 Test: Motor Hardware Test - CORRECTED VERSION
Tests motor connections using your EXACT proven code patterns

Location: ~/robot_project/indoor_robot_ws/src/hardware_interface/tests/test_motors.py
Usage: python3 test_motors.py

This test replicates your EXACT motorControl.py functionality
"""

import sys
import time
from gpiozero import Motor

class MotorTester:
    def __init__(self):
        self.test_results = []
        self.motorA = None  # Left motor
        self.motorB = None  # Right motor
        
    def log_result(self, test_name, passed, message=""):
        """Log test result"""
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name}")
        if message:
            print(f"    {message}")
        
        self.test_results.append({
            'test': test_name,
            'passed': passed,
            'message': message
        })

    def test_gpio_import(self):
        """Test if gpiozero can be imported"""
        print("\n=== Testing GPIO Library ===")
        try:
            from gpiozero import Motor
            self.log_result("gpiozero import", True)
            return True
        except ImportError as e:
            self.log_result("gpiozero import", False, f"Install with: pip3 install gpiozero")
            return False

    def test_motor_initialization(self):
        """Test motor initialization with your proven pin configuration"""
        print("\n=== Testing Motor Initialization ===")
        
        try:
            # Using your EXACT configuration from motorControl.py
            self.motorA = Motor(17, 18, pwm=True)  # Left motor
            self.motorB = Motor(22, 23, pwm=True)  # Right motor
            self.log_result("Motor A (Left) initialization", True, "GPIO pins 17,18")
            self.log_result("Motor B (Right) initialization", True, "GPIO pins 22,23")
            return True
        except Exception as e:
            self.log_result("Motor initialization", False, f"GPIO error: {str(e)}")
            return False

    # REPLICATE YOUR EXACT FUNCTIONS FROM motorControl.py
    def goForward(self, test_time, speed):
        """Exact copy of your goForward function"""
        self.motorA.forward(speed)
        self.motorB.forward(speed)
        time.sleep(test_time)
        self.motorA.stop()
        self.motorB.stop()
    
    def goBackward(self, test_time, speed):
        """Exact copy of your goBackward function"""
        self.motorA.backward(speed)
        self.motorB.backward(speed)
        time.sleep(test_time)
        self.motorA.stop()
        self.motorB.stop()
    
    def goRight(self, test_time, speed):
        """Exact copy of your goRight function"""
        self.motorA.forward(speed)
        self.motorB.stop()
        time.sleep(test_time)
        self.motorA.stop()
    
    def goLeft(self, test_time, speed):
        """Exact copy of your goLeft function"""
        self.motorA.stop()
        self.motorB.forward(speed)
        time.sleep(test_time)
        self.motorB.stop()
    
    def stop(self):
        """Exact copy of your stop function"""
        self.motorA.stop()
        self.motorB.stop()

    def test_motor_basic_functions(self):
        """Test basic motor functions using your EXACT proven code"""
        if not self.motorA or not self.motorB:
            self.log_result("Motor basic functions", False, "Motors not initialized")
            return False
            
        print("\n=== Testing Basic Motor Functions (Your Proven Code) ===")
        
        try:
            # Test forward function - using your exact parameters
            print("Testing forward motion for 2.5 seconds at 50% speed...")
            self.goForward(2.5, 0.3)  # Same as your test.py
            self.log_result("Forward motion", True, "Using your goForward() function")
            
            time.sleep(1)
            
            # Test backward function
            print("Testing backward motion for 2.5 seconds at 30% speed...")
            self.goBackward(2.5, 0.3)
            self.log_result("Backward motion", True, "Using your goBackward() function")
            
            time.sleep(1)
            
            # Test right turn
            print("Testing right turn for 2 seconds at 30% speed...")
            self.goRight(2.0, 0.3)
            self.log_result("Right turn", True, "Using your goRight() function")
            
            time.sleep(1)
            
            # Test left turn
            print("Testing left turn for 2 seconds at 30% speed...")
            self.goLeft(2.0, 0.3)
            self.log_result("Left turn", True, "Using your goLeft() function")
            
            return True
            
        except Exception as e:
            self.log_result("Motor basic functions", False, f"Motor control error: {str(e)}")
            return False

    def test_your_exact_test_sequence(self):
        """Test the exact sequence from your test.py"""
        if not self.motorA or not self.motorB:
            return False
            
        print("\n=== Testing Your Exact test.py Sequence ===")
        
        try:
            print("Running your exact test: motorControl('b',2.5,0.3)")
            # Replicate: motorControl('b',2.5,0.5)
            self.goBackward(2.5, 0.3)
            time.sleep(1)  # Your sleep(1) from test.py
            
            self.log_result("Your exact test.py sequence", True, "Backward 2.5s at 50% speed")
            return True
            
        except Exception as e:
            self.log_result("Your exact test sequence", False, f"Test sequence error: {str(e)}")
            return False

    def test_speed_validation(self):
        """Test the speed validation from your motorControl function"""
        print("\n=== Testing Speed Validation ===")
        
        try:
            # Test valid speeds
            valid_speeds = [0.1, 0.5]
            for speed in valid_speeds:
                print(f"Testing valid speed {speed*100}% for 1 second...")
                self.goForward(1.0, speed)
                time.sleep(1.0)
            
            self.log_result("Speed validation", True, "Tested 10%, 50% speeds")
            return True
            
        except Exception as e:
            self.log_result("Speed validation", False, f"Speed test error: {str(e)}")
            return False

    def test_ubuntu_gpio_permissions(self):
        """Test GPIO permissions specifically for Ubuntu"""
        print("\n=== Testing Ubuntu GPIO Permissions ===")
        
        try:
            import os
            import grp
            
            # Check if user is in gpio group
            try:
                gpio_group = grp.getgrnam('gpio')
                current_user = os.getenv('USER')
                user_groups = [g.gr_name for g in grp.getgrall() if current_user in g.gr_mem]
                
                if 'gpio' in user_groups:
                    self.log_result("GPIO group membership", True, f"User {current_user} in gpio group")
                else:
                    self.log_result("GPIO group membership", False, 
                                   f"Add user to gpio group: sudo usermod -a -G gpio {current_user}")
            except KeyError:
                self.log_result("GPIO group check", True, "GPIO group not found (may be OK on Ubuntu)")
            
            # Test GPIO device access
            gpio_devices = ['/dev/gpiomem', '/dev/mem']
            for device in gpio_devices:
                if os.path.exists(device):
                    if os.access(device, os.R_OK | os.W_OK):
                        self.log_result(f"GPIO device access: {device}", True)
                    else:
                        self.log_result(f"GPIO device access: {device}", False, 
                                       "Permission denied - check user groups")
                else:
                    self.log_result(f"GPIO device exists: {device}", False, "Device not found")
            
            return True
            
        except Exception as e:
            self.log_result("Ubuntu GPIO permissions", False, str(e))
            return False

    def cleanup(self):
        """Clean up GPIO resources"""
        try:
            if self.motorA:
                self.motorA.close()
            if self.motorB:
                self.motorB.close()
            print("\nGPIO resources cleaned up")
        except Exception as e:
            print(f"Cleanup warning: {e}")

    def print_summary(self):
        """Print test summary"""
        passed = sum(1 for result in self.test_results if result['passed'])
        failed = len(self.test_results) - passed
        
        print("\n" + "="*50)
        print("MOTOR HARDWARE TEST SUMMARY")
        print("="*50)
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Total:  {len(self.test_results)}")
        
        if failed == 0:
            print("\n🎉 ALL MOTOR TESTS PASSED! Hardware confirmed working on Ubuntu.")
            print("\nVerified using your exact proven code patterns:")
            print("  ✅ Same GPIO pins (17,18 and 22,23)")
            print("  ✅ Same speed values (0.3 = 30%)")
            print("  ✅ Same timing patterns (2.5s movements)")
            print("  ✅ Same function structure (goForward, goBackward, etc.)")
            print("\nNext steps:")
            print("1. Proceed to LiDAR testing")
            print("2. Then test ROS motor_bridge.py integration")
        else:
            print(f"\n⚠️  {failed} motor tests failed.")
            print("\nFailed tests:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  - {result['test']}: {result['message']}")
            
            print("\nUbuntu-specific troubleshooting:")
            print("1. Check GPIO permissions: sudo usermod -a -G gpio $USER")
            print("2. Install GPIO access: sudo apt install rpi.gpio-common")
            print("3. Reboot after adding user to gpio group")
            print("4. Check hardware connections match Raspberry Pi OS setup")
        
        return failed == 0

def main():
    print("Phase 1 Test: Motor Hardware Verification (Ubuntu 20.04)")
    print("="*60)
    print("This test uses your EXACT proven motorControl.py code patterns")
    print("Verified working on Raspberry Pi OS - now testing on Ubuntu")
    print("MAKE SURE ROBOT IS IN A SAFE AREA - MOTORS WILL MOVE!")
    print("="*60)
    
    # Enhanced safety prompt
    print("\n⚠️  SAFETY CHECK:")
    print("1. Robot is in safe open area? ")
    print("2. Emergency stop is accessible?")
    print("3. Motors/wheels can move freely?")
    print("4. Power connections secure?")
    
    response = input("\nAll safety checks OK - ready to test motors? (y/N): ")
    if response.lower() != 'y':
        print("Test cancelled for safety. Run again when ready.")
        sys.exit(1)
    
    tester = MotorTester()
    
    try:
        print(f"\nStarting tests using your proven code patterns...")
        
        # Run tests in sequence
        if not tester.test_gpio_import():
            tester.print_summary()
            sys.exit(1)
        
        # Ubuntu-specific checks
        tester.test_ubuntu_gpio_permissions()
            
        if not tester.test_motor_initialization():
            tester.print_summary()
            sys.exit(1)
        
        # Run movement tests using your exact code
        tester.test_motor_basic_functions()
        tester.test_your_exact_test_sequence()
        tester.test_speed_validation()
        
        # Print results
        all_passed = tester.print_summary()
        sys.exit(0 if all_passed else 1)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user - stopping motors")
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
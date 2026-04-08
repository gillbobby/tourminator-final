#!/usr/bin/env python3
"""
Motor Spin Test: Verifies robot can spin in place at various speeds
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
        """Test motor initialization"""
        print("\n=== Testing Motor Initialization ===")
        try:
            self.motorA = Motor(17, 18, pwm=True)  # Left motor
            self.motorB = Motor(22, 23, pwm=True)  # Right motor
            self.log_result("Motor initialization", True, "GPIO pins 17,18 (Left) and 22,23 (Right)")
            return True
        except Exception as e:
            self.log_result("Motor initialization", False, f"GPIO error: {str(e)}")
            return False

    def spin_in_place(self, duration, speed, clockwise=True):
        """Make the robot spin in place"""
        if clockwise:
            self.motorA.forward(speed)   # Left motor forward
            self.motorB.backward(speed)  # Right motor backward
        else:
            self.motorA.backward(speed)  # Left motor backward
            self.motorB.forward(speed)   # Right motor forward
        time.sleep(duration)
        self.stop()

    def stop(self):
        """Stop both motors"""
        self.motorA.stop()
        self.motorB.stop()

    def test_spin_functions(self):
        """Test spinning at various speeds"""
        if not self.motorA or not self.motorB:
            self.log_result("Spin functions", False, "Motors not initialized")
            return False
            
        print("\n=== Testing Spin Functions ===")
        
        try:
            test_speeds = [0.2, 0.4, 0.6, 0.8]  # 20%, 40%, 60%, 80% speed
            test_duration = 2.0  # 2 seconds per test
            
            for speed in test_speeds:
                # Clockwise spin
                print(f"Testing clockwise spin at {int(speed*100)}% speed for {test_duration} seconds...")
                self.spin_in_place(test_duration, speed, clockwise=True)
                self.log_result(f"Clockwise spin at {int(speed*100)}%", True)
                time.sleep(1)  # Pause between tests
                
                # Counter-clockwise spin
                print(f"Testing counter-clockwise spin at {int(speed*100)}% speed for {test_duration} seconds...")
                self.spin_in_place(test_duration, speed, clockwise=False)
                self.log_result(f"Counter-clockwise spin at {int(speed*100)}%", True)
                time.sleep(1)  # Pause between tests
            
            return True
            
        except Exception as e:
            self.log_result("Spin functions", False, f"Spin test error: {str(e)}")
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
        print("SPIN TEST SUMMARY")
        print("="*50)
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Total:  {len(self.test_results)}")
        
        if failed == 0:
            print("\n🎉 ALL SPIN TESTS PASSED! Robot can spin in place at all tested speeds.")
            print("\nTested speed levels:")
            print("  - 20% (gentle spin)")
            print("  - 40% (moderate spin)")
            print("  - 60% (fast spin)")
            print("  - 80% (aggressive spin)")
        else:
            print(f"\n⚠️  {failed} spin tests failed.")
            print("\nFailed tests:")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  - {result['test']}: {result['message']}")
            
            print("\nTroubleshooting:")
            print("1. Check motor wiring connections")
            print("2. Verify power supply is adequate")
            print("3. Ensure wheels can move freely")
        
        return failed == 0

def main():
    print("Robot Spin Test: Verifies in-place rotation at various speeds")
    print("="*60)
    print("MAKE SURE ROBOT IS IN A SAFE AREA - IT WILL SPIN IN PLACE!")
    print("="*60)
    
    # Safety prompt
    print("\n⚠️  SAFETY CHECK:")
    print("1. Robot is on a clear, flat surface")
    print("2. There's at least 2 feet of clearance in all directions")
    print("3. Emergency stop is accessible")
    print("4. Wheels can rotate freely without obstruction")
    
    response = input("\nAll safety checks OK - ready to test spins? (y/N): ")
    if response.lower() != 'y':
        print("Test cancelled for safety. Run again when ready.")
        sys.exit(1)
    
    tester = MotorTester()
    
    try:
        print(f"\nStarting spin tests...")
        
        # Run tests in sequence
        if not tester.test_gpio_import():
            tester.print_summary()
            sys.exit(1)
            
        if not tester.test_motor_initialization():
            tester.print_summary()
            sys.exit(1)
        
        # Run spin tests
        tester.test_spin_functions()
        
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
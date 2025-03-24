#!/usr/bin/env python3
import sys
import os
import time
from SCServo_Linux.SCServo import SCServo

# Add the SCServo_Linux directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

class ServoController:
    def __init__(self, port):
        self.servo = SCServo(port)
        self.servo.open()
        time.sleep(0.1)  # Wait for servo to initialize
        
        # Initialize servos to their current positions
        self.servo.read_pos(1)  # Joint 3
        self.servo.read_pos(2)  # Joint 4
        self.servo.read_pos(3)  # Joint 5
        
    def move_servos(self, joint3_angle, joint4_angle, joint5_angle):
        """
        Move the servos to the specified angles (in degrees)
        """
        try:
            # Convert angles to servo positions (assuming 0.24 degrees per step)
            joint3_pos = int(joint3_angle / 0.24)
            joint4_pos = int(joint4_angle / 0.24)
            joint5_pos = int(joint5_angle / 0.24)
            
            # Move servos
            self.servo.write_pos(1, joint3_pos)
            self.servo.write_pos(2, joint4_pos)
            self.servo.write_pos(3, joint5_pos)
            
            # Wait for servos to reach position
            time.sleep(0.1)
            return True
            
        except Exception as e:
            print(f"Error moving servos: {e}")
            return False
            
    def close(self):
        """
        Close the servo connection
        """
        self.servo.close()

# Global servo controller instance
servo_controller = None

def init_servos(port):
    """
    Initialize the servo controller
    """
    global servo_controller
    try:
        servo_controller = ServoController(port)
        return True
    except Exception as e:
        print(f"Error initializing servos: {e}")
        return False

def move_servos(joint3_angle, joint4_angle, joint5_angle):
    """
    Move the servos to the specified angles
    """
    global servo_controller
    if servo_controller is None:
        print("Servo controller not initialized")
        return False
    return servo_controller.move_servos(joint3_angle, joint4_angle, joint5_angle) 
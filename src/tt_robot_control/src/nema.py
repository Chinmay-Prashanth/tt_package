#!/usr/bin/env python3
import sys
import os
import time
from servost import ServoST

# Add the servost directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class NemaController:
    def __init__(self, port):
        self.servo = ServoST(port)
        self.servo.open()
        time.sleep(0.1)  # Wait for servo to initialize
        
        # Initialize motors to their current positions
        self.servo.read_pos(1)  # Joint 1
        self.servo.read_pos(2)  # Joint 2
        
    def move_motors(self, joint1_steps, joint2_degrees):
        """
        Move the NEMA motors to the specified positions
        joint1_steps: Steps for the linear actuator (Joint 1)
        joint2_degrees: Degrees for the rotary motor (Joint 2)
        """
        try:
            # Convert joint2 degrees to steps (assuming 200 steps per revolution)
            joint2_steps = int(joint2_degrees * 200 / 360)
            
            # Move motors
            self.servo.write_pos(1, joint1_steps)
            self.servo.write_pos(2, joint2_steps)
            
            # Wait for motors to reach position
            time.sleep(0.1)
            return True
            
        except Exception as e:
            print(f"Error moving NEMA motors: {e}")
            return False
            
    def close(self):
        """
        Close the servo connection
        """
        self.servo.close()

# Global NEMA controller instance
nema_controller = None

def init_motors(port):
    """
    Initialize the NEMA motor controller
    """
    global nema_controller
    try:
        nema_controller = NemaController(port)
        return True
    except Exception as e:
        print(f"Error initializing NEMA motors: {e}")
        return False

def move_motors(joint1_steps, joint2_degrees):
    """
    Move the NEMA motors to the specified positions
    """
    global nema_controller
    if nema_controller is None:
        print("NEMA controller not initialized")
        return False
    return nema_controller.move_motors(joint1_steps, joint2_degrees) 
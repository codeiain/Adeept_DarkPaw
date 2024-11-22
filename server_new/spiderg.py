import socket
import time
import threading

try:
    import Adafruit_PCA9685
except ImportError:
    # Install the Adafruit_PCA9685 library if not already installed
    import os
    os.system("sudo pip3 install adafruit-pca9685")
    import Adafruit_PCA9685

# Initialize PCA9685 PWM controller
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)  # Set PWM frequency to 50 Hz


class MPU6050Manager:
    """
    Class to manage MPU6050 sensor and related functionalities such as PID and Kalman filters.
    """
    def __init__(self):
        try:
            from mpu6050 import mpu6050
            import PID
            import Kalman_filter
            
            self.sensor = mpu6050(0x68)
            print('mpu6050 connected\nmpu6050 is connected and related functions are available.')
            self.mpu_tor = 0
            self.X_steady = 0
            self.Y_steady = 0
            self.P = 0.3
            self.I = 0.1
            self.D = 0
            
            # Initialize PID controllers
            self.X_pid = PID.PID()
            self.X_pid.SetKp(self.P)
            self.X_pid.SetKd(self.I)
            self.X_pid.SetKi(self.D)
            
            self.Y_pid = PID.PID()
            self.Y_pid.SetKp(self.P)
            self.Y_pid.SetKd(self.I)
            self.Y_pid.SetKi(self.D)
            
            # Initialize Kalman filters
            self.kalman_filter_X = Kalman_filter.Kalman_filter(0.001, 0.1)
            self.kalman_filter_Y = Kalman_filter.Kalman_filter(0.001, 0.1)
            self.steadyMode = 0
            self.MPU_connection = 1
        except ImportError:
            # Handle cases where the MPU6050 library is unavailable
            print('mpu6050 disconnected\nmpu6050 is not connected and related functions are unavailable.')
            self.MPU_connection = 0
            self.steadyMode = 0


class ServoController:
    """
    Class to handle servo motor initialization, configuration, and movement.
    """
    def __init__(self):
        # Define servo ports
        self.servo_ports = {
            "FLB": 0, "FLM": 1, "FLE": 2,
            "FRB": 6, "FRM": 7, "FRE": 8,
            "HLB": 3, "HLM": 4, "HLE": 5,
            "HRB": 9, "HRM": 10, "HRE": 11,
            "P": 12, "T": 13
        }

        # Define initial PWM values for each servo
        self.init_pwm_values = {
            "FLB": 390, "FLM": 312, "FLE": 313,
            "FRB": 191, "FRM": 281, "FRE": 301,
            "HLB": 362, "HLM": 287, "HLE": 260,
            "HRB": 194, "HRM": 195, "HRE": 335,
            "P": 300, "T": 300
        }

        # Define servo movement directions
        self.directions = {
            "FLB": 1, "FLM": -1, "FLE": -1,
            "FRB": -1, "FRM": 1, "FRE": 1,
            "HLB": -1, "HLM": 1, "HLE": 1,
            "HRB": 1, "HRM": -1, "HRE": -1,
            "P": 1, "T": 1
        }

    def initialize_servos(self):
        """
        Set all servos to their initial positions.
        """
        for servo, pwm_value in self.init_pwm_values.items():
            port = self.servo_ports[servo]
            pwm.set_pwm(port, 0, pwm_value)
        print("All servos initialized to default positions.")

    def set_servo_pwm(self, servo_name, pwm_value):
        """
        Set PWM value for a specific servo.
        """
        port = self.servo_ports[servo_name]
        pwm.set_pwm(port, 0, pwm_value)
        
    def pwm_to_angle(pwm_value, freq=50, min_pulse=1, max_pulse=2, min_angle=0, max_angle=180):
        pulse_width = (pwm_value / 4096) * (1 / freq) * 1000  # Pulse width in ms
        angle = ((pulse_width - min_pulse) / (max_pulse - min_pulse)) * (max_angle - min_angle) + min_angle
        return angle


class RobotController:
    """
    Main class to manage the robot's movement and state transitions.
    """
    def __init__(self):
        self.servo_controller = ServoController()
        self.mpu_manager = MPU6050Manager()

        # State dictionaries to manage current and goal positions
        self.current_state = self.servo_controller.init_pwm_values.copy()
        self.goal_state = self.servo_controller.init_pwm_values.copy()

    def smooth_transition(self, servo_name, target_pwm, steps=20, delay=0.02):
        """
        Smoothly transition a servo from its current position to a target position.
        """
        current_pwm = self.current_state[servo_name]
        step_increment = (target_pwm - current_pwm) / steps

        for step in range(steps):
            new_pwm = int(current_pwm + step_increment * step)
            self.servo_controller.set_servo_pwm(servo_name, new_pwm)
            time.sleep(delay)

        self.current_state[servo_name] = target_pwm

    def move_to_goal(self):
        """
        Transition all servos to their goal positions smoothly.
        """
        for servo_name, target_pwm in self.goal_state.items():
            self.smooth_transition(servo_name, target_pwm)
        print("Movement to goal positions complete.")

    def update_goal_position(self, servo_name, pwm_value):
        """
        Update the goal PWM value for a specific servo.
        """
        self.goal_state[servo_name] = pwm_value


# Main Execution
if __name__ == "__main__":
    robot = RobotController()

    # Initialize servos
    robot.servo_controller.initialize_servos()

    # Example: Move a specific servo smoothly to a new position
    robot.update_goal_position("FLB", 400)
    robot.move_to_goal()

rewrite file3 to be oo using file1 and 2


file1

#!/usr/bin/env python3
# File name   : servo.py
# Description : Control lights for a robot, including LED strips and GPIO-controlled lights.
# Date        : 2019/02/23

import time
import threading

try:
    # Attempt to use RPi.GPIO for Raspberry Pi
    import RPi.GPIO as GPIO
except ImportError:
    # Fallback to Mock.GPIO for testing on non-Raspberry Pi systems
    import Mock.GPIO as GPIO

from rpi_ws281x import *

class RobotLight(threading.Thread):
    """
    A class for managing the robot's lighting system, including LED strips and GPIO-controlled lights.
    """

    def __init__(self, *args, **kwargs):
        super(RobotLight, self).__init__(*args, **kwargs)

        # LED strip configuration
        self.LED_COUNT = 16           # Number of LEDs in the strip
        self.LED_PIN = 12             # GPIO pin for LED strip data
        self.LED_FREQ_HZ = 800000     # LED signal frequency (standard is 800kHz)
        self.LED_DMA = 10             # DMA channel for generating PWM signal
        self.LED_BRIGHTNESS = 255     # Brightness of LEDs (0 to 255)
        self.LED_INVERT = False       # Invert signal for specific hardware setups
        self.LED_CHANNEL = 0          # PWM channel (0 or 1 based on GPIO pin)

        # Color configuration for breathing mode
        self.colorBreathR = 0         # Red component (0-255)
        self.colorBreathG = 0         # Green component (0-255)
        self.colorBreathB = 0         # Blue component (0-255)
        self.breathSteps = 10         # Steps for smooth breathing transitions

        # Current light mode ('none', 'police', or 'breath')
        self.lightMode = 'none'

        # GPIO setup for controlling other lights
        GPIO.setwarnings(False)       # Suppress GPIO warnings
        GPIO.setmode(GPIO.BCM)        # Use Broadcom pin numbering
        GPIO.setup(5, GPIO.OUT)       # GPIO pin 5 for headlight
        GPIO.setup(6, GPIO.OUT)       # GPIO pin 6 for auxiliary light 1
        GPIO.setup(13, GPIO.OUT)      # GPIO pin 13 for auxiliary light 2

        # Initialize the LED strip using the Adafruit NeoPixel library
        self.strip = Adafruit_NeoPixel(
            self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ,
            self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL
        )
        self.strip.begin()  # Begin communication with the LED strip

        # Thread event management
        self.__flag = threading.Event()  # Event to control thread execution
        self.__flag.clear()              # Start with thread paused

    def setColor(self, R, G, B):
        """
        Set all LEDs in the strip to a specified color.
        Parameters:
        - R: Red intensity (0-255)
        - G: Green intensity (0-255)
        - B: Blue intensity (0-255)
        """
        color = Color(int(R), int(G), int(B))
        for i in range(self.strip.numPixels()):  # Iterate over all LEDs
            self.strip.setPixelColor(i, color)  # Apply color to each LED
        self.strip.show()  # Update LED strip with new colors

    def setSomeColor(self, R, G, B, IDs):
        """
        Set specific LEDs in the strip to a specified color.
        Parameters:
        - R: Red intensity (0-255)
        - G: Green intensity (0-255)
        - B: Blue intensity (0-255)
        - IDs: List of LED indices to update
        """
        color = Color(int(R), int(G), int(B))
        for i in IDs:  # Iterate over specified LEDs
            self.strip.setPixelColor(i, color)
        self.strip.show()  # Update LED strip

    def pause(self):
        """
        Pause the current light mode and turn off LEDs.
        """
        self.lightMode = 'none'  # Reset light mode
        self.setColor(0, 0, 0)   # Turn off all LEDs
        self.__flag.clear()      # Pause thread execution

    def resume(self):
        """
        Resume light mode processing.
        """
        self.__flag.set()  # Resume thread execution

    def police(self):
        """
        Activate the police light mode.
        Alternates flashing between blue and red.
        """
        self.lightMode = 'police'
        self.resume()

    def policeProcessing(self):
        """
        Execute the police light flashing pattern.
        """
        while self.lightMode == 'police':
            # Flash blue lights
            for _ in range(3):
                self.setSomeColor(0, 0, 255, range(13))  # All LEDs blue
                time.sleep(0.05)
                self.setSomeColor(0, 0, 0, range(13))    # Turn off
                time.sleep(0.05)
            if self.lightMode != 'police':
                break
            time.sleep(0.1)

            # Flash red lights
            for _ in range(3):
                self.setSomeColor(255, 0, 0, range(13))  # All LEDs red
                time.sleep(0.05)
                self.setSomeColor(0, 0, 0, range(13))    # Turn off
                time.sleep(0.05)
            time.sleep(0.1)

    def breath(self, R, G, B):
        """
        Activate the breathing light mode with the specified color.
        Parameters:
        - R: Red intensity (0-255)
        - G: Green intensity (0-255)
        - B: Blue intensity (0-255)
        """
        self.lightMode = 'breath'
        self.colorBreathR = R
        self.colorBreathG = G
        self.colorBreathB = B
        self.resume()

    def breathProcessing(self):
        """
        Execute the breathing light effect.
        LEDs fade in and out smoothly.
        """
        while self.lightMode == 'breath':
            # Fade in
            for i in range(self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(
                    self.colorBreathR * i / self.breathSteps,
                    self.colorBreathG * i / self.breathSteps,
                    self.colorBreathB * i / self.breathSteps
                )
                time.sleep(0.03)

            # Fade out
            for i in range(self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(
                    self.colorBreathR - (self.colorBreathR * i / self.breathSteps),
                    self.colorBreathG - (self.colorBreathG * i / self.breathSteps),
                    self.colorBreathB - (self.colorBreathB * i / self.breathSteps)
                )
                time.sleep(0.03)

    def frontLight(self, switch):
        """
        Control the front auxiliary lights.
        Parameters:
        - switch: 'on' to enable lights, 'off' to disable lights
        """
        GPIO.output(6, GPIO.HIGH if switch == 'on' else GPIO.LOW)
        GPIO.output(13, GPIO.HIGH if switch == 'on' else GPIO.LOW)

    def switch(self, port, status):
        """
        Control individual GPIO-controlled lights.
        Parameters:
        - port: Light port (1, 2, or 3)
        - status: 1 to turn on, 0 to turn off
        """
        pins = {1: 5, 2: 6, 3: 13}
        if port in pins:
            GPIO.output(pins[port], GPIO.HIGH if status else GPIO.LOW)
        else:
            print("Invalid port! Use 1, 2, or 3.")

    def set_all_switch_off(self):
        """
        Turn off all GPIO-controlled lights.
        """
        for port in range(1, 4):
            self.switch(port, 0)

    def headLight(self, switch):
        """
        Control the headlight.
        Parameters:
        - switch: 'on' to enable, 'off' to disable
        """
        GPIO.output(5, GPIO.HIGH if switch == 'on' else GPIO.LOW)

    def lightChange(self):
        """
        Execute the appropriate light pattern based on the current mode.
        """
        if self.lightMode == 'police':
            self.policeProcessing()
        elif self.lightMode == 'breath':
            self.breathProcessing()

    def run(self):
        """
        Main thread loop for handling light modes.
        Waits for a signal to execute the current mode's logic.
        """
        while True:
            self.__flag.wait()  # Wait for resume signal
            self.lightChange()

# Main execution block
if __name__ == '__main__':
    RL = RobotLight()  # Create an instance of the RobotLight class
    RL.start()         # Start the thread
    RL.breath(70, 70, 255)  # Activate breathing mode with blue color
    time.sleep(10)    
    
    
file 2

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
    
file 3

#!/usr/bin/env/python3
# File name   : server.py
# Production  : RaspTankPro
# Website     : www.adeept.com
# Author      : William
# Date        : 2020/03/17

import time
import threading
import move
import Adafruit_PCA9685
import os
import info

import robotLight
import switch
import socket

import SpiderG
SpiderG.move_init()

#websocket
import asyncio
import websockets

import json
import app

OLED_connection = 0
curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

direction_command = 'no'
turn_command = 'no'

FLB_init_pwm = SpiderG.FLB_init_pwm
FLM_init_pwm = SpiderG.FLM_init_pwm
FLE_init_pwm = SpiderG.FLE_init_pwm

FRB_init_pwm = SpiderG.FRB_init_pwm
FRM_init_pwm = SpiderG.FRM_init_pwm
FRE_init_pwm = SpiderG.FRE_init_pwm

HLB_init_pwm = SpiderG.HLB_init_pwm
HLM_init_pwm = SpiderG.HLM_init_pwm
HLE_init_pwm = SpiderG.HLE_init_pwm

HRB_init_pwm = SpiderG.HRB_init_pwm
HRM_init_pwm = SpiderG.HRM_init_pwm
HRE_init_pwm = SpiderG.HRE_init_pwm


def servoPosInit():
    SpiderG.move_init()


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    global r
    newline=""
    str_num=str(new_num)
    with open(thisPath+"/SpiderG.py","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num+"\n")
            newline += line
    with open(thisPath+"/SpiderG.py","w") as f:
        f.writelines(newline)


def ap_thread():
    os.system("sudo create_ap wlan0 eth0 AdeeptRobot 12345678")


def functionSelect(command_input, response):
    global functionMode
    
    if 'findColor' == command_input:
        flask_app.modeselect('findColor')

    elif 'motionGet' == command_input:
        flask_app.modeselect('watchDog')

    elif 'stopCV' == command_input:
        flask_app.modeselect('none')
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)
        time.sleep(0.1)
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)

    elif 'police' == command_input:
        RL.police()

    elif 'policeOff' == command_input:
        RL.setColor(0,255,64)
    elif "redAlert" == command_input:
        RL.setColor(255,0,0)
        SpiderG.status_GenOut(-200, 0, 0)
        SpiderG.direct_M_move()
    elif "redAlertOff" == command_input:
        RL.setColor(0,255,64)
        SpiderG.move_init()
    elif "breath" == command_input:
        RL.breath(70,70,255)
    elif "breathOff" == command_input:
        RL.pause()
        RL.setColor(0,255,64)
    elif "sleep" == command_input:
        flask_app.modeselect('none')
        switch.switch(1,0)
        switch.switch(2,0)
        switch.switch(3,0)
        RL.pause()
        SpiderG.status_GenOut(200, 0, 0)
        SpiderG.direct_M_move()
    elif "wake" == command_input:
        RL.setColor(0,255,64)
        SpiderG.move_init()

def switchCtrl(command_input, response):
    if 'Switch_1_on' in command_input:
        switch.switch(1,1)

    elif 'Switch_1_off' in command_input:
        switch.switch(1,0)

    elif 'Switch_2_on' in command_input:
        switch.switch(2,1)

    elif 'Switch_2_off' in command_input:
        switch.switch(2,0)

    elif 'Switch_3_on' in command_input:
        switch.switch(3,1)

    elif 'Switch_3_off' in command_input:
        switch.switch(3,0) 


def robotCtrl(command_input, response):
    global direction_command, turn_command
    if 'forward' == command_input:
        command_input = 'forward'
        SpiderG.walk('forward')
    
    elif 'backward' == command_input:
        command_input = 'backward'
        SpiderG.walk('backward')

    elif 'DS' in command_input:
        command_input = 'no'
        if turn_command == 'no':
            SpiderG.move_init()
            SpiderG.servoStop()
        elif turn_command == 'left':
            SpiderG.walk('turnleft')
        elif turn_command == 'right':
            SpiderG.walk('turnright')


    elif 'left' == command_input:
        turn_command = 'left'
        SpiderG.walk('turnleft')

    elif 'right' == command_input:
        turn_command = 'right'
        SpiderG.walk('turnright')

    elif 'TS' in command_input:
        turn_command = 'no'
        if direction_command == 'no':
            SpiderG.move_init()
            SpiderG.servoStop()
        else:
            SpiderG.walk(direction_command)


    elif 'steadyCamera' == command_input:     
        SpiderG.move_init()               #Steady
        SpiderG.steadyModeOn()

    elif 'steadyCameraOff' == command_input:                    #Steady
        SpiderG.steadyModeOff()


    elif 'lookleft' == command_input:
        SpiderG.walk('Lean-L')

    elif 'lookright' == command_input:
        SpiderG.walk('Lean-R')

    elif 'up' == command_input:
        SpiderG.status_GenOut(0, -150, 0)
        SpiderG.direct_M_move()

    elif 'down' == command_input:
        SpiderG.status_GenOut(0, 150, 0)
        SpiderG.direct_M_move()

    elif 'stop' == command_input:
        SpiderG.servoStop()

    elif 'home' == command_input:
        SpiderG.move_init()

    elif 'wsB' in command_input:
        try:
            set_B=command_input.split()
            speed_set = int(set_B[1])
        except:
            pass

    elif 'grab' == command_input:
        SpiderG.status_GenOut(-200, 0, 0)
        SpiderG.direct_M_move()

    elif 'loose' == command_input:
        SpiderG.status_GenOut(200, 0, 0)
        SpiderG.direct_M_move()


    elif 'home' in command_input:#3
        SpiderG.status_GenOut(0, 0, 0)
        SpiderG.direct_M_move()

    else:
        pass

    print(command_input)



def configPWM(command_input, response):
    global  FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm

    if 'SiLeft' in command_input:
        numServo = int(command_input[7:])
        if numServo == 0:
            FLB_init_pwm -= 1
            SpiderG.FLB_init_pwm = FLB_init_pwm
        elif numServo == 1:
            FLM_init_pwm -= 1
            SpiderG.FLM_init_pwm = FLM_init_pwm
        elif numServo == 2:
            FLE_init_pwm -= 1
            SpiderG.FLE_init_pwm = FLE_init_pwm

        elif numServo == 3:
            HLB_init_pwm -= 1
            SpiderG.HLB_init_pwm = HLB_init_pwm
        elif numServo == 4:
            HLM_init_pwm -= 1
            SpiderG.HLM_init_pwm = HLM_init_pwm
        elif numServo == 5:
            HLE_init_pwm -= 1
            SpiderG.HLE_init_pwm = HLE_init_pwm

        elif numServo == 6:
            FRB_init_pwm -= 1
            SpiderG.FRB_init_pwm = FRB_init_pwm
        elif numServo == 7:
            FRM_init_pwm -= 1
            SpiderG.FRM_init_pwm = FRM_init_pwm
        elif numServo == 8:
            FRE_init_pwm -= 1
            SpiderG.FRE_init_pwm = FRE_init_pwm

        elif numServo == 9:
            HRB_init_pwm -= 1
            SpiderG.HRB_init_pwm = HRB_init_pwm
        elif numServo == 10:
            HRM_init_pwm -= 1
            SpiderG.HRM_init_pwm = HRM_init_pwm
        elif numServo == 11:
            HRE_init_pwm -= 1
            SpiderG.HRE_init_pwm = HRE_init_pwm

        SpiderG.move_init()


    if 'SiRight' in command_input:
        numServo = int(command_input[8:])
        if numServo == 0:
            FLB_init_pwm += 1
            SpiderG.FLB_init_pwm = FLB_init_pwm
        elif numServo == 1:
            FLM_init_pwm += 1
            SpiderG.FLM_init_pwm = FLM_init_pwm
        elif numServo == 2:
            FLE_init_pwm += 1
            SpiderG.FLE_init_pwm = FLE_init_pwm

        elif numServo == 3:
            HLB_init_pwm += 1
            SpiderG.HLB_init_pwm = HLB_init_pwm
        elif numServo == 4:
            HLM_init_pwm += 1
            SpiderG.HLM_init_pwm = HLM_init_pwm
        elif numServo == 5:
            HLE_init_pwm += 1
            SpiderG.HLE_init_pwm = HLE_init_pwm

        elif numServo == 6:
            FRB_init_pwm += 1
            SpiderG.FRB_init_pwm = FRB_init_pwm
        elif numServo == 7:
            FRM_init_pwm += 1
            SpiderG.FRM_init_pwm = FRM_init_pwm
        elif numServo == 8:
            FRE_init_pwm += 1
            SpiderG.FRE_init_pwm = FRE_init_pwm

        elif numServo == 9:
            HRB_init_pwm += 1
            SpiderG.HRB_init_pwm = HRB_init_pwm
        elif numServo == 10:
            HRM_init_pwm += 1
            SpiderG.HRM_init_pwm = HRM_init_pwm
        elif numServo == 11:
            HRE_init_pwm += 1
            SpiderG.HRE_init_pwm = HRE_init_pwm

        SpiderG.move_init()


    if 'PWMMS' in command_input:
        numServo = int(command_input[6:])
        if numServo == 0:
            replace_num('FLB_init_pwm = ', FLB_init_pwm)
        elif numServo == 1:
            replace_num('FLM_init_pwm = ', FLM_init_pwm)
        elif numServo == 2:
            replace_num('FLE_init_pwm = ', FLE_init_pwm)

        elif numServo == 3:
            replace_num('HLB_init_pwm = ', HLB_init_pwm)
        elif numServo == 4:
            replace_num('HLM_init_pwm = ', HLM_init_pwm)
        elif numServo == 5:
            replace_num('HLE_init_pwm = ', HLE_init_pwm)

        elif numServo == 6:
            replace_num('FRB_init_pwm = ', FRB_init_pwm)
        elif numServo == 7:
            replace_num('FRM_init_pwm = ', FRM_init_pwm)
        elif numServo == 8:
            replace_num('FRE_init_pwm = ', FRE_init_pwm)

        elif numServo == 9:
            replace_num('HRB_init_pwm = ', HRB_init_pwm)
        elif numServo == 10:
            replace_num('HRM_init_pwm = ', HRM_init_pwm)
        elif numServo == 11:
            replace_num('HRE_init_pwm = ', HRE_init_pwm)

        SpiderG.move_init()

    if 'PWMINIT' == command_input:
        SpiderG.move_init()

    elif 'PWMD' == command_input:
        FLB_init_pwm = 300
        FLM_init_pwm = 300
        FLE_init_pwm = 300

        HLB_init_pwm = 300
        HLM_init_pwm = 300
        HLE_init_pwm = 300

        FRB_init_pwm = 300
        FRM_init_pwm = 300
        FRE_init_pwm = 300

        HRB_init_pwm = 300
        HRM_init_pwm = 300
        HRE_init_pwm = 300

        SpiderG.FLB_init_pwm = FLB_init_pwm
        SpiderG.FLM_init_pwm = FLM_init_pwm
        SpiderG.FLE_init_pwm = FLE_init_pwm

        SpiderG.HLB_init_pwm = HLB_init_pwm
        SpiderG.HLM_init_pwm = HLM_init_pwm
        SpiderG.HLE_init_pwm = HLE_init_pwm

        SpiderG.FRB_init_pwm = FRB_init_pwm
        SpiderG.FRM_init_pwm = FRM_init_pwm
        SpiderG.FRE_init_pwm = FRE_init_pwm

        SpiderG.HRB_init_pwm = HRB_init_pwm
        SpiderG.HRM_init_pwm = HRM_init_pwm
        SpiderG.HRE_init_pwm = HRE_init_pwm

        replace_num('FLB_init_pwm = ', FLB_init_pwm)
        replace_num('FLM_init_pwm = ', FLM_init_pwm)
        replace_num('FLE_init_pwm = ', FLE_init_pwm)

        replace_num('HLB_init_pwm = ', HLB_init_pwm)
        replace_num('HLM_init_pwm = ', HLM_init_pwm)
        replace_num('HLE_init_pwm = ', HLE_init_pwm)

        replace_num('FRB_init_pwm = ', FRB_init_pwm)
        replace_num('FRM_init_pwm = ', FRM_init_pwm)
        replace_num('FRE_init_pwm = ', FRE_init_pwm)

        replace_num('HRB_init_pwm = ', HRB_init_pwm)
        replace_num('HRM_init_pwm = ', HRM_init_pwm)
        replace_num('HRE_init_pwm = ', HRE_init_pwm)

# def update_code():
#     # Update local to be consistent with remote
#     projectPath = thisPath[:-7]
#     with open(f'{projectPath}/config.json', 'r') as f1:
#         config = json.load(f1)
#         if not config['production']:
#             print('Update code')
#             # Force overwriting local code
#             if os.system(f'cd {projectPath} && sudo git fetch --all && sudo git reset --hard origin/master && sudo git pull') == 0:
#                 print('Update successfully')
#                 print('Restarting...')
#                 os.system('sudo reboot')

def wifi_check():
    try:
        s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect(("1.1.1.1",80))
        ipaddr_check=s.getsockname()[0]
        s.close()
        print(ipaddr_check)
        # update_code()
        if OLED_connection:
            screen.screen_show(2, 'IP:'+ipaddr_check)
            screen.screen_show(3, 'AP MODE OFF')
    except:
        RL.pause()
        RL.setColor(0,255,64)
        ap_threading=threading.Thread(target=ap_thread)   #Define a thread for data receiving
        ap_threading.setDaemon(True)                          #'True' means it is a front thread,it would close when the mainloop() closes
        ap_threading.start()                                  #Thread starts
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 10%')
        RL.setColor(0,16,50)
        time.sleep(1)
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 30%')
        RL.setColor(0,16,100)
        time.sleep(1)
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 50%')
        RL.setColor(0,16,150)
        time.sleep(1)
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 70%')
        RL.setColor(0,16,200)
        time.sleep(1)
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 90%')
        RL.setColor(0,16,255)
        time.sleep(1)
        if OLED_connection:
            screen.screen_show(2, 'AP Starting 100%')
        RL.setColor(35,255,35)
        if OLED_connection:
            screen.screen_show(2, 'IP:192.168.12.1')
            screen.screen_show(3, 'AP MODE ON')

async def check_permit(websocket):
    while True:
        recv_str = await websocket.recv()
        cred_dict = recv_str.split(":")
        if cred_dict[0] == "admin" and cred_dict[1] == "123456":
            response_str = "congratulation, you have connect with server\r\nnow, you can do something else"
            await websocket.send(response_str)
            return True
        else:
            response_str = "sorry, the username or password is wrong, please submit again"
            await websocket.send(response_str)

async def recv_msg(websocket):
    global speed_set, modeSelect
    # move.setup()
    direction_command = 'no'
    turn_command = 'no'

    while True: 
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }

        data = ''
        data = await websocket.recv()
        try:
            data = json.loads(data)
        except Exception as e:
            print('not A JSON')

        if not data:
            continue

        if isinstance(data,str):
            robotCtrl(data, response)

            switchCtrl(data, response)

            functionSelect(data, response)

            configPWM(data, response)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info()]

            if 'wsB' in data:
                try:
                    set_B=data.split()
                    speed_set = int(set_B[1])
                except:
                    pass

            elif 'AR' == data:
                modeSelect = 'AR'
                screen.screen_show(4, 'ARM MODE ON')
                try:
                    fpv.changeMode('ARM MODE ON')
                except:
                    pass

            elif 'PT' == data:
                modeSelect = 'PT'
                screen.screen_show(4, 'PT MODE ON')
                try:
                    fpv.changeMode('PT MODE ON')
                except:
                    pass

            #CVFL
            elif 'CVFL' == data:
                flask_app.modeselect('findlineCV')

            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                flask_app.camera.colorSet(color)

            elif 'CVFLL1' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_1(pos)

            elif 'CVFLL2' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_2(pos)

            elif 'CVFLSP' in data:
                err = int(data.split()[1])
                flask_app.camera.errorSet(err)

            elif 'defEC' in data:#Z
                fpv.defaultExpCom()

        elif(isinstance(data,dict)):
            if data['title'] == "findColorSet":
                color = data['data']
                flask_app.colorFindSet(color[0],color[1],color[2])

        print(data)
        response = json.dumps(response)
        await websocket.send(response)

async def main_logic(websocket, path):
    await check_permit(websocket)
    await recv_msg(websocket)

if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()

    HOST = ''
    PORT = 10223                              #Define port serial 
    BUFSIZ = 1024                             #Define buffer size
    ADDR = (HOST, PORT)

    global flask_app
    flask_app = app.webapp()
    flask_app.startthread()

    try:
        RL=robotLight.RobotLight()
        RL.start()
        RL.breath(70,70,255)
    except:
        print('Use "sudo pip3 install rpi_ws281x" to install WS_281x package\n使用"sudo pip3 install rpi_ws281x"命令来安装rpi_ws281x')
        pass

    while  1:
        wifi_check()
        try:                  #Start server,waiting for client
            start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
            asyncio.get_event_loop().run_until_complete(start_server)
            print('waiting for connection...')
            # print('...connected from :', addr)
            break
        except Exception as e:
            print(e)
            RL.setColor(0,0,0)

        try:
            RL.breath(70,70,255)
        except:
            pass
    try:
        RL.pause()
        RL.setColor(0,255,64)
        asyncio.get_event_loop().run_forever()
    except Exception as e:
        print(e)
        RL.setColor(0,0,0)
        # move.destroy()

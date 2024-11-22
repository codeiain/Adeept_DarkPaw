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
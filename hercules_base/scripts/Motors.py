# Simple two DC motor robot class.  Exposes a simple LOGO turtle-like API for
# moving a robot forward, backward, and turning.  See RobotTest.py for an
# example of using this class.
# Author: Tony DiCola
# License: MIT License https://opensource.org/licenses/MIT
import time
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT

# Changed by Sean Scheideman Feb 11 2018
#  - uses 4 motors now 

class Motors(object):
    def __init__(self, addr=0x60, left_trim=0, right_trim=0,
                 stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameters:
         - addr: The I2C address of the motor HAT, default is 0x60.
         - left_id: The ID of the left motor, default is 1.
         - right_id: The ID of the right motor, default is 2.
         - left_trim: Amount to offset the speed of the left motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - right_trim: Amount to offset the speed of the right motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        # Initialize motor HAT and left, right motor.
        self._mh = Adafruit_MotorHAT(addr)
        
        self._front_left = self._mh.getMotor(4)
        self._front_right = self._mh.getMotor(1)
        
        self._back_left = self._mh.getMotor(3)
        self._back_right = self._mh.getMotor(2)
        
        
        self._left_trim = left_trim
        self._right_trim = right_trim
        # Start with motors turned off.
        self._front_left.run(Adafruit_MotorHAT.RELEASE)
        self._back_left.run(Adafruit_MotorHAT.RELEASE)
        self._front_right.run(Adafruit_MotorHAT.RELEASE)
        self._back_right.run(Adafruit_MotorHAT.RELEASE)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._left_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._front_left.setSpeed(speed)
        self._back_left.setSpeed(speed)

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._right_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._front_right.setSpeed(speed)
        self._back_right.setSpeed(speed)


    def stop(self):
        """Stop all movement."""
        self._front_left.run(Adafruit_MotorHAT.RELEASE)
        self._front_right.run(Adafruit_MotorHAT.RELEASE)
        
        self._back_left.run(Adafruit_MotorHAT.RELEASE)
        self._back_right.run(Adafruit_MotorHAT.RELEASE)

    def right(self, speed, seconds=None):
        """Spin to the right at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.        
        if(speed < 0):
            self._right_speed(-speed)
            self._front_right.run(Adafruit_MotorHAT.BACKWARD)
            self._back_right.run(Adafruit_MotorHAT.FORWARD)
        else:
            self._right_speed(speed)
            self._front_right.run(Adafruit_MotorHAT.FORWARD)
            self._back_right.run(Adafruit_MotorHAT.BACKWARD)
        
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def left(self, speed, seconds=None):
        """Spin to the left at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        if(speed < 0):
            self._left_speed(-speed)
            self._front_left.run(Adafruit_MotorHAT.BACKWARD)
            self._back_left.run(Adafruit_MotorHAT.FORWARD)
        else:
            self._left_speed(speed)
            self._front_left.run(Adafruit_MotorHAT.FORWARD)
            self._back_left.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

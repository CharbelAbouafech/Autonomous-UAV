#!/usr/bin/env python3
"""
Servo gripper (claw) controller.

Controls a servo on GPIO 21 via PWM for open/close operations.
Angles and pin match test_claw.py configuration.
"""

import logging
import time

import RPi.GPIO as GPIO

logger = logging.getLogger(__name__)

SERVO_PIN = 21
PWM_FREQ = 50  # 50 Hz standard servo frequency
OPEN_ANGLE = 19
CLOSE_ANGLE = 190.98


class ClawController:
    """Servo-driven gripper control."""

    def __init__(self, pin=SERVO_PIN, open_angle=OPEN_ANGLE, close_angle=CLOSE_ANGLE):
        self.pin = pin
        self.open_angle = open_angle
        self.close_angle = close_angle
        self._pwm = None
        self._is_open = False

    def start(self):
        """Initialize GPIO and PWM."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self.pin, PWM_FREQ)
        self._pwm.start(0)
        time.sleep(0.5)
        logger.info(f"Claw: initialized on GPIO {self.pin}")

    def stop(self):
        """Stop PWM and clean up GPIO."""
        if self._pwm:
            self._pwm.stop()
        GPIO.cleanup(self.pin)
        logger.info("Claw: stopped")

    def _set_angle(self, angle):
        """Move servo to angle (degrees)."""
        duty = 2.5 + (10 * angle / 180)
        self._pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self._pwm.ChangeDutyCycle(0)  # Stop signal to prevent jitter

    def open(self):
        """Open the gripper."""
        logger.info("Claw: opening")
        self._set_angle(self.open_angle)
        self._is_open = True

    def close(self):
        """Close the gripper (grab)."""
        logger.info("Claw: closing")
        self._set_angle(self.close_angle)
        self._is_open = False

    @property
    def is_open(self):
        return self._is_open

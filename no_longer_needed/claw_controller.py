#!/usr/bin/env python3
"""
Servo gripper (claw) controller.

Controls a servo on GPIO 21 via PWM for open/close operations.
Uses sweep motion for smooth movement and holds torque when closed.
"""

import logging
import time

import RPi.GPIO as GPIO

logger = logging.getLogger(__name__)

SERVO_PIN = 21
PWM_FREQ = 50  # 50 Hz standard servo frequency
OPEN_ANGLE = 0
CLOSE_ANGLE = 75
SWEEP_STEP = 75


class ClawController:
    """Servo-driven gripper control with sweep motion."""

    def __init__(self, pin=SERVO_PIN, open_angle=OPEN_ANGLE, close_angle=CLOSE_ANGLE):
        self.pin = pin
        self.open_angle = open_angle
        self.close_angle = close_angle
        self._pwm = None
        self._current_angle = open_angle
        self._is_open = True

    def start(self):
        """Initialize GPIO and PWM, move to open position."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=False)
        self._pwm = GPIO.PWM(self.pin, PWM_FREQ)
        self._pwm.start(0)
        time.sleep(1)
        self._set_angle(self.open_angle)
        time.sleep(1)
        self._current_angle = self.open_angle
        logger.info(f"Claw: initialized on GPIO {self.pin}")

    def stop(self):
        """Stop PWM and clean up GPIO."""
        if self._pwm:
            self._pwm.stop()
        GPIO.cleanup(self.pin)
        logger.info("Claw: stopped")

    def _set_angle(self, angle, hold=False):
        """Move servo to angle (degrees). If hold, keep signal active."""
        duty = 2.5 + (angle / 180) * 10
        self._pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        if not hold:
            self._pwm.ChangeDutyCycle(0)

    def _sweep_to(self, target_angle, hold=False):
        """Sweep smoothly from current angle to target angle."""
        step = SWEEP_STEP if target_angle > self._current_angle else -SWEEP_STEP
        for angle in range(self._current_angle, target_angle + step, step):
            angle = max(0, min(180, angle))
            is_last = (angle == target_angle)
            self._set_angle(angle, hold=(hold and is_last))
            time.sleep(0.02)
        self._current_angle = target_angle

    def open(self):
        """Open the gripper."""
        logger.info("Claw: opening")
        self._sweep_to(self.open_angle, hold=False)
        self._is_open = True

    def close(self):
        """Close the gripper (grab). Holds torque to maintain grip."""
        logger.info("Claw: closing")
        self._sweep_to(self.close_angle, hold=True)
        self._is_open = False

    @property
    def is_open(self):
        return self._is_open

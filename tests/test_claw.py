#!/usr/bin/env python3
"""Interactive test for the claw controller."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from controllers.claw_controller import ClawController

claw = ClawController()
claw.start()

print("Servo Control Ready: 'o' to Open, 'c' to Close, 'q' to Quit")

try:
    while True:
        cmd = input("Enter Command: ").strip().lower()

        if cmd == 'o':
            print("Opening...")
            claw.open()
        elif cmd == 'c':
            print("Closing...")
            claw.close()
        elif cmd == 'q':
            print("Quitting.")
            break
        else:
            print("Invalid Command!")
finally:
    claw.stop()

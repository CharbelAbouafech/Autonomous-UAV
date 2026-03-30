#!/usr/bin/env python3
"""Test the ClawController open/close."""

from controllers.claw_controller import ClawController

claw = ClawController()
claw.start()

try:
    while True:
        cmd = input("Enter command (o=open, c=close, e=exit): ").strip().lower()
        if cmd == "o":
            claw.open()
            print(f"Claw open: {claw.is_open}")
        elif cmd == "c":
            claw.close()
            print(f"Claw open: {claw.is_open}")
        elif cmd == "e":
            break
        else:
            print("Invalid command")
finally:
    claw.stop()

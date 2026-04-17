#!/bin/bash
# Launch PX4 SITL with Gazebo, home set to Mojave competition site
export PX4_HOME_LAT=35.0503
export PX4_HOME_LON=-118.1505
export PX4_HOME_ALT=830.0

cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500

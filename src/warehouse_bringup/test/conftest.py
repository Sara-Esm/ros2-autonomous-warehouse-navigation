"""Test configuration: isolate ROS traffic and make test helpers importable."""

import os
import random
import sys

# Isolate these tests from any running simulation: a random domain, localhost only.
# Must be set before rclpy initializes the middleware.
os.environ['ROS_DOMAIN_ID'] = str(random.randint(150, 220))
os.environ['ROS_LOCALHOST_ONLY'] = '1'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

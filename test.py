#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA waypoint follower assessment client script.

A controller assessment to follow a given trajectory, where the trajectory
can be defined using way-points.

STARTING in a moment...
"""
import numpy as np
x_p = [1, 2]
y_p = [1, 2]

line_coef = np.polyfit(x_p, y_p, 1)
a = -line_coef[0]
b = 1
c = -line_coef[1]
print(a, b, c)
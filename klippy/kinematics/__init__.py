# Package definition for the kinematics directory
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from .five_bar_shoulder import load_kinematics as five_bar_shoulder
lookup = {
    "five_bar_shoulder": five_bar_shoulder,
}
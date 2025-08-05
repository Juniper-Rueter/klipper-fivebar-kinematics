# Code for handling the kinematics of 5 bar linkages
#
# Copyright (C) 2025 Juniper Rueter <brueter918@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging
import stepper
import mathutil
import chelper


class FiveBarShoulder:
    def __init__(self, toolhead, config):
        self.limits = [(1.0, -1.0)] * 3
        self.axes_min = toolhead.Coord(-100, -100, 0, e=0.)
        self.axes_max = toolhead.Coord(100, 100, 0, e=0.)

        stepper_configs = [config.getsection('stepper_arm_left'),
                           config.getsection('stepper_arm_right')]

        rail_arm_left = stepper.LookupRail(stepper_configs[0], need_position_minmax=False, units_in_radians=True)
        rail_arm_right = stepper.LookupRail(stepper_configs[1], need_position_minmax=False, units_in_radians=True)

        self.inner_distance = config.getfloat('inner_distance', above=0.)
        self.x_origin = config.getfloat('x_origin')
        self.y_origin = config.getfloat('y_origin')
        self.home_x = config.getfloat('home_x')
        self.home_y = config.getfloat('home_y')

        self.left_inner_arm = stepper_configs[0].getfloat(
            'inner_arm_length', above=0.)
        self.left_outer_arm = stepper_configs[0].getfloat(
            'outer_arm_length', above=0.)
        rail_arm_left.setup_itersolve('fivebarelbow_stepper_alloc', b'l',
                                       self.left_inner_arm, self.left_outer_arm,
                                       self.inner_distance, self.x_origin, self.y_origin)

        self.right_inner_arm = stepper_configs[1].getfloat(
            'inner_arm_length', above=0.)
        self.right_outer_arm = stepper_configs[1].getfloat(
            'outer_arm_length', above=0.)
        rail_arm_right.setup_itersolve('fivebarelbow_stepper_alloc', b'r',
                                        self.right_inner_arm, self.right_outer_arm,
                                        self.inner_distance, self.x_origin, self.y_origin)

        self.rails = [rail_arm_left, rail_arm_right]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        config.get_printer().register_event_handler(
            "stepper_enable:motor_off",  self._motor_off)

        max_velocity, max_accel = toolhead.get_max_velocity()

        self.homedXY = False

        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()

        self.cartesian_kinematics_L = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.cartesian_kinematics_R = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'y'), ffi_lib.free)

        logging.info("5-Bar elbow driven %.2f %.2f %.2f %.2f %.2f",
                     self.left_inner_arm, self.left_outer_arm,
                     self.right_inner_arm, self.right_outer_arm,
                     self.inner_distance)
        self.gcode = self.printer.lookup_object('gcode')
        
        self.gcode.respond_info("initialized")

    def _motor_off(self, print_time):
        self.homedXY = False

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        left_angle = pos[0]
        right_angle = pos[1]
        [x, y] = self._angles_to_position(left_angle, right_angle)
        return [x, y]

    def set_position(self, newpos, homing_axes):
        logging.info("5be set_position %s    %s", newpos, homing_axes)
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        if 'x' in homing_axes and 'y' in homing_axes:
            self.homedXY = True
            logging.info("Set xy %f %f", newpos[0], newpos[1])


    def clear_homing_state(self, clear_axes):
        pass

    def home(self, homing_state):
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([self.home_x, self.home_y, 0.])

    def check_move(self, move):
        pass

    def get_status(self, eventtime):
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return FiveBarShoulder(toolhead, config)

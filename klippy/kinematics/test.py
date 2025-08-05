# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil, chelper

class CartKinematics:
    def __init__(self, toolhead, config):
        #self.axes_min = toolhead.Coord([0, 0, 0], e=0.)
        #self.axes_max = toolhead.Coord([0, 0, 0], e=0.)

        #self.limits = [(1.0, -1.0)] * 2
        #self.set_position([0., 0., 0.], "")



        stepper_configs = [config.getsection('stepper_arm_left'),
                           config.getsection('stepper_arm_right')]

        rail_arm_left = stepper.PrinterStepper(config.getsection('stepper_arm_left'))
        rail_arm_right = stepper.PrinterStepper(config.getsection('stepper_arm_right'))

        self.inner_distance = config.getfloat('inner_distance', above=0.)

        self.left_inner_arm = stepper_configs[0].getfloat('inner_arm_length', above=0.)
        self.left_outer_arm = stepper_configs[0].getfloat('outer_arm_length', above=0.)
        rail_arm_left.setup_itersolve('fivebarelbow_stepper_alloc', 'l',
                                      self.left_inner_arm, self.left_outer_arm,
                                      self.inner_distance)
        
        self.right_inner_arm = stepper_configs[1].getfloat('inner_arm_length', above=0.)
        self.right_outer_arm = stepper_configs[1].getfloat('outer_arm_length', above=0.)
        rail_arm_right.setup_itersolve(
            'fivebarelbow_stepper_alloc', 'r',
             self.right_inner_arm, self.right_outer_arm,
             self.inner_distance)

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
            ffi_lib.cartesian_stepper_alloc('x'), ffi_lib.free)
        self.cartesian_kinematics_R = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc('y'), ffi_lib.free)

        logging.info("5-Bar elbow driven %.2f %.2f %.2f %.2f %.2f",
                     self.left_inner_arm, self.left_outer_arm,
                     self.right_inner_arm, self.right_outer_arm,
                     self.inner_distance)

    def get_steppers(self):
        list(self.steppers)
    def calc_position(self, stepper_positions):
        rails = self.rails
        if self.dc_module:
            primary_rail = self.dc_module.get_primary_rail(
                    self.dual_carriage_axis)
            rails = (rails[:self.dual_carriage_axis] +
                     [primary_rail] + rails[self.dual_carriage_axis+1:])
        return [stepper_positions[rail.get_name()] for rail in rails]
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = "xyz".index(axis_name)
            if self.dc_module and axis == self.dual_carriage_axis:
                rail = self.dc_module.get_primary_rail(self.dual_carriage_axis)
            else:
                rail = self.rails[axis]
            self.limits[axis] = rail.get_range()
    def clear_homing_state(self, clear_axes):
        #for axis, axis_name in enumerate("xy"):
        #    if axis_name in clear_axes:
        #        self.limits[axis] = (1.0, -1.0)
        pass
    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
    def check_move(self, move):
        pass
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xy", self.limits) if l <= h]
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)

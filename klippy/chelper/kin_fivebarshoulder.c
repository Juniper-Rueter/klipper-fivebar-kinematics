// Five Bar SCARA driven by the shoulders
//
// Copyright (C) 2025 Juniper Rueter <brueter918@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct fivebarshoulder_stepper {
    struct stepper_kinematics sk;
    double inner_arm_length;
    double outer_arm_length;
    double inner_arm_offset;
    char arm;
    double x_origin;
    double y_origin;
};

static inline double sqr(double a) { return a*a; };

static double
fivebarshoulder_stepper_calc_position(
    struct stepper_kinematics *sk,
    struct move *m,
    double move_time)
{
    struct fivebarshoulder_stepper *fs =
        container_of(sk, struct fivebarshoulder_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    double x = -(c.x+fs->x_origin); //no clue why I have to invert X but here we are
    double y = c.y+fs->y_origin;
    double l0 = fs->inner_arm_offset;
    double l1 = fs->inner_arm_length;
    double l2 = fs->outer_arm_length;

    if (fs->arm == 'l') {
        double alpha = acos((sqr(l1)+(sqr(l0+x)+sqr(y))-sqr(l2)) / (2*l1*sqrt(sqr(l0+x)+sqr(y))));
        double beta = atan2(y, l0 + x);
        return M_PI-beta-alpha;
    } else {
        double alpha = acos((sqr(l1)+(sqr(l0-x)+sqr(y))-sqr(l2)) / (2*l1*sqrt(sqr(l0-x)+sqr(y))));
        double beta = atan2(y, l0 - x);
        return beta+alpha;
    }
}

struct stepper_kinematics * __visible
fivebarshoulder_stepper_alloc(char arm,
                           double inner_arm_length,
                           double outer_arm_length,
                           double inner_arms_distance,
                           double x_origin,
                           double y_origin)
{

    struct fivebarshoulder_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));

    fs->x_origin = x_origin;
    fs->y_origin = y_origin;
    fs->arm = arm;
    fs->sk.calc_position_cb = fivebarshoulder_stepper_calc_position;
    fs->inner_arm_length = inner_arm_length;
    fs->outer_arm_length = outer_arm_length;
    fs->inner_arm_offset = inner_arms_distance / 2.0; //set origin point to between motors
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
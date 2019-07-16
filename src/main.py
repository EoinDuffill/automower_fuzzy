#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 27 15:28:46 2019
@author: direnc
"""

import rospy
from am_driver.msg import Loop
from geometry_msgs.msg import Twist

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy

from T1_set import T1_Gaussian, T1_Triangular, T1_RightShoulder, T1_LeftShoulder
from inter_union import inter_union
from T1_output import T1_Triangular_output, T1_RightShoulder_output, T1_LeftShoulder_output
from draw_fis import draw_fis


def aggregate(rules,technique):
    #defining the domain
    left = rules[0][1].interval[0]
    right = rules[0][1].interval[1]

    for rule in rules:
        if left > rule[1].interval[0]:
            left = rule[1].interval[0]
        if right < rule[1].interval[1]:
            right = rule[1].interval[1]
    disc_of_all = np.linspace(left, right, len(rules)*100)

    if technique == "max":
        degree = []

        for x in disc_of_all:
            max_degree = 0.0
            for rule in rules:
                if max_degree < rule[1].get_degree(x):
                    max_degree = rule[1].get_degree(x)
            degree.append(max_degree)

        return degree, disc_of_all


def centroid((memberships, domain)):
    numerator_sum = 0
    denominator_sum = 0
    for i in range(len(domain)):
        numerator_sum += memberships[i] * domain[i]
        denominator_sum += memberships[i]

    return numerator_sum/denominator_sum


def rule_output(my_input, rule, operator):
    # TO DO multi input is not compatible
    # TO DO add a singleton option

    # check the consistency of the antecedents and input
    if (len(rule) != len(my_input)):  # !!
        raise Exception("The antecedents and inputs numbers are not equal")

    fs = []
    for index in range(len(rule)):  # !!
        if my_input[index].step == 0:
            fs.append(rule[index].get_degree(my_input[index].mean))
        else:
            FSs_interation = inter_union(rule[index], my_input[index], 100)
            fs.append(FSs_interation.return_firing_stregth("standard"))

    if operator == "min":
        return min(fs)

def noise_estimation(collected_sensor_values):

    if len(collected_sensor_values) > 1:
        diff_list = []
        for index, i in enumerate(collected_sensor_values[:-1]):
            try:
                diff_list.append((collected_sensor_values[index + 1] - i) / float(np.sqrt(2)))
            except:
                print("An exception occurred in the noise estimation")

        return (np.std(diff_list))
    else:
        return 0


class BoundarySensor(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.readings = []
        self.prev_value = None
        self.value = 0
        self.sd = 0
        self.noise_estimate = 0

    def calc_distri(self):
        if len(self.readings) > 0:
            # Mean
            sum = 0
            for reading in self.readings:
                sum += reading
            self.prev_value = self.value
            self.value = (sum * 1.0)/len(self.readings)

            # s.d.
            sum = 0
            for reading in self.readings:
                sum += pow(reading - self.value, 2)
            self.sd = np.sqrt((sum * 1.0)/len(self.readings))

            # noise
            self.noise_estimate = noise_estimation(self.readings)

            self.readings = []



class FIS(object):

    def __init__(self, observe):
        self.update_rate = 5

        # Publish output here
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # FC, FR, RL, RR, init w/ x,y co-ords on the automower
        self.front_center = BoundarySensor(0.3, 0)
        self.front_right = BoundarySensor(0.3, -0.07)
        self.rear_left = BoundarySensor(-0.15, 0.12)
        self.rear_right = BoundarySensor(-0.15, -0.12)
        self.observation_count = 0
        self.interval_time = 0

        self.x = []
        self.counter = 0
        self.mean_list = []
        self.file = open("mean_error_results.txt", "w")
        if not observe:
            self.file_inputs = open("fuzzy_inputs.txt", "w")

        # Control parameters
        self.cp_target = 7500

        self.dist_multiplier = (15000 - self.cp_target)/15000

        self.name = "Steering"

        # antecent 1 MF's
        very_close = T1_RightShoulder(self.cp_target + 1000, self.cp_target + 2000, self.cp_target + 3000)
        # med_close = T1_Triangular(self.cp_close, self.cp_med_close, self.cp_very_close)
        close = T1_Triangular(self.cp_target, self.cp_target + 1000, self.cp_target + 2000)
        medium = T1_Triangular(self.cp_target - 1000, self.cp_target, self.cp_target + 1000)
        far = T1_Triangular(self.cp_target - 2000, self.cp_target - 1000, self.cp_target)
        very_far = T1_LeftShoulder(self.cp_target - 3000, self.cp_target - 2000, self.cp_target - 1000)

        # input 2 MF's
        negative = T1_LeftShoulder(-500, -250, 0)
        none = T1_Triangular(-250, 0, 250)
        positive = T1_RightShoulder(0, 250, 500)

        # Output turn direction, right = -ve, left = +ve
        # Output MF's

        right_pair = self.create_output_pair(T1_Triangular_output, T1_Triangular_output, [-1, -0.25, 0])
        right_sharp_pair = self.create_output_pair(T1_LeftShoulder_output, T1_RightShoulder_output, [-1, -0.75, -0.5])

        left_pair = self.create_output_pair(T1_Triangular_output, T1_Triangular_output, [0, 0.25, 1])
        left_shallow_pair = self.create_output_pair(T1_Triangular_output, T1_Triangular_output, [0, 0.1, 0.2])

        straight_pair = self.create_output_pair(T1_Triangular_output, T1_Triangular_output, [-0.25, 0, 0.25])

        # DEBUG TODO 

        # plot(close.get_mf_degrees())

        rule_1 = [[close, positive], copy.deepcopy(right_pair), "If Close and +ve delta then Right"]
        rule_2 = [[close, none], copy.deepcopy(right_pair), "If Close and no delta then Straight"]
        rule_3 = [[close, negative], copy.deepcopy(straight_pair), "If Close and -ve delta then Left"]
        rule_4 = [[medium, positive], copy.deepcopy(right_pair), "If Medium and +ve delta then Right"]
        rule_5 = [[medium, none], copy.deepcopy(straight_pair), "If Medium and no delta then Straight"]
        rule_6 = [[medium, negative], copy.deepcopy(left_pair), "If Medium and -ve delta then Left"]
        rule_7 = [[far, positive], copy.deepcopy(straight_pair), "If Far and +ve delta then Straight"]
        rule_8 = [[far, none], copy.deepcopy(left_pair), "If Far and no delta then Left"]
        rule_9 = [[far, negative], copy.deepcopy(left_pair), "If Far and -ve delta then Left"]
        rule_10= [[very_close, positive], copy.deepcopy(right_sharp_pair), "If Very Close and +ve delta then Sharp Right"]
        rule_11= [[very_close, none], copy.deepcopy(right_pair), "If Very Close and no delta then Right"]
        rule_12= [[very_close, negative], copy.deepcopy(straight_pair), "If Very Close and -ve delta then Straight"]
        rule_13= [[very_far, positive], copy.deepcopy(left_shallow_pair), "If Very Far and +ve delta then Shallow Left"]
        rule_14= [[very_far, none], copy.deepcopy(left_shallow_pair), "If Very Far and no delta then Shallow Left"]
        rule_15= [[very_far, negative], copy.deepcopy(left_pair), "If Very Far and -ve delta then Left"]

        rule_set_base = [rule_1,
                         rule_2,
                         rule_3,
                         rule_4,
                         rule_5,
                         rule_6,
                         rule_7,
                         rule_8,
                         rule_9,
                         rule_10,
                         rule_11,
                         rule_12,
                         rule_13,
                         rule_14,
                         rule_15]

        self.Rule_set = self.split_ruleset(rule_set_base)
        self.inverse_rules = False

        # input with mean and sigma
        self.input_obj1 = T1_Gaussian(0, 1)
        self.input_obj2 = T1_Gaussian(0, 1)

        self.plots = draw_fis([
            [very_close, close, medium, far, very_far],
            [negative, none, positive]],
            [right_sharp_pair, right_pair, straight_pair, left_pair, left_shallow_pair])
        #self.plots.plot()



    def create_output_pair(self, f1, f2, params):
        return f1(params[0], params[1], params[2], 1), f2(-params[2], -params[1], -params[0], 1)

    def split_ruleset(self, ruleset):

        rules_new = []
        rules_new_inv = []

        for rule in ruleset:
            rules_new.append([rule[0], rule[1][0], rule[2]])
            rules_new_inv.append([rule[0], rule[1][1], rule[2]])

        return (rules_new, rules_new_inv)

    def update(self):
        # Get sensor value avg's since last update
        #


        if self.avg_loop() != -1:
            self.input_obj1 = T1_Gaussian(self.front_center.value, self.front_center.noise_estimate)
            if self.front_center.prev_value is not None:
                self.input_obj2 = T1_Gaussian(self.front_center.value - self.front_center.prev_value, 0)
            else:
                self.input_obj2 = T1_Gaussian(0,0)

            # create alist of inputs
            inputs = [self.input_obj1, self.input_obj2]  # !!

            # self.evalFIS(self.input_obj1)
            # send all the inputs as params#!!
            self.evalFIS(inputs)  # !!

            if self.inverse_rules:
                rules = self.Rule_set[1]
            else:
                rules = self.Rule_set[0]

            output = centroid(aggregate(rules, "max"))

            print output

            # Apply Output
            # min speed
            min = 0.25
            max = 0.75
            diff = max - min

            twist = Twist()
            twist.linear.x = ((1-abs(output))*diff) + min
            twist.angular.z = output

            # Publish output
            self.pub_vel.publish(twist)

        return

    def evalFIS(self, inputs):
        print "--- Multi Inputs ----"  # !!
        for input in inputs:  # !!
            print "Mean = " + str(input.mean) + ", S.D. = " + str(input.step)  # !!
        print "---------------"  # !!


        self.counter += 1

        self.file_inputs.write( str(self.counter) + ", " + str(inputs[0].mean) + "\t"+ str(inputs[0].step) + "\t" + str(inputs[1].mean) + "\t" + str(inputs[1].step) + "\n")

        self.file.write(str(self.counter) + ", " + str(abs(inputs[0].mean - self.cp_target))+"\n")

        if self.inverse_rules:
            rules = self.Rule_set[1]
        else:
            rules = self.Rule_set[0]

        for index, rule in enumerate(rules):

            rule[1].set_max_fs(float(rule_output(inputs, rule[0], "min")))
            print rule[2]+"\t" + str(rule[1].max_fs)

    # Accumulate loop sensor observations
    def parse_loop(self, data):

        self.observation_count += 1
        self.front_center.readings.append(data.frontCenter)
        self.front_right.readings.append(data.frontRight)
        self.rear_left.readings.append(data.rearLeft)
        self.rear_right.readings.append(data.rearRight)

    # Calculate avg since last avg calc
    def avg_loop(self):
        # AVG sensor values since last update
        # return -1 is no observations have been made (no sensor update)
        # return +ve if observations > 0

        if self.observation_count == 0:
            return -1
        else:
            self.front_center.calc_distri()
            self.front_right.calc_distri()
            self.rear_left.calc_distri()
            self.rear_right.calc_distri()

            self.observation_count = 0
            return 1



    def fini(self):
        print('Finishing...')


    def run(self):
        try:
            # Setup subscribers
            rospy.Subscriber("loop", Loop, self.parse_loop, queue_size=None)
            r = rospy.Rate(self.update_rate)
            while not rospy.is_shutdown():
                self.update()
                r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
        finally:
            self.fini()


if __name__ == '__main__':
    rospy.init_node('automower_fuzzy')
    automower_fuzzy = FIS(False)
    automower_fuzzy.run()

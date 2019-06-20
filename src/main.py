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

from T1_set import T1_Gaussian, T1_Triangular, T1_RightShoulder, T1_LeftShoulder
from inter_union import inter_union
from T1_output import T1_Triangular_output, T1_RightShoulder_output, T1_LeftShoulder_output


def aggregate(rules,technique):
    #defining the domain
    left=rules[0][1].interval[0]
    right=rules[0][1].interval[1]

    for rule in rules:
        if left> rule[1].interval[0]:
            left=rule[1].interval[0] 
        if right< rule[1].interval[1]:
            right=rule[1].interval[1]
    disc_of_all = np.linspace(left, right, len(rules)*100)

    if technique=="max":
        degree=[]
        
        for x in disc_of_all:
            max_degree = 0.0
            for rule in rules:
                if max_degree < rule[1].get_degree(x):
                    max_degree = rule[1].get_degree(x)
            degree.append(max_degree) 
            
        return degree , disc_of_all


def centroid((memberships, domain)):
    numerator_sum = 0
    denominator_sum = 0
    for i in range(len(domain)):
        numerator_sum += memberships[i] * domain[i]
        denominator_sum += memberships[i]

    return numerator_sum/denominator_sum


def rule_output(my_input,rule,operator):
    # TO DO multi input is not compatible
    # TO DO add a singleton option

    fs=[]
    for antecedent in rule:
        if my_input.step==0:
            fs.append(antecedent.get_degree(my_input.mean))
        else:
            FSs_interation=inter_union(antecedent, my_input, 100)
            fs.append(FSs_interation.return_firing_stregth("standard"))

    if (operator=="min"):
        return min(fs)


class BoundarySensor(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.readings = []
        self.prev_value = 0
        self.value = 0
        self.sd = 0

    def calc_distri(self):
        if len(self.readings) > 0:
            #Mean
            sum = 0
            for reading in self.readings:
                sum += reading
            self.prev_value = self.value
            self.value = (sum * 1.0)/len(self.readings)

            #S.d.
            sum = 0
            for reading in self.readings:
                sum += pow(reading - self.value,2)
            self.sd = np.sqrt((sum * 1.0)/len(self.readings))

            self.readings = []


class FIS(object):

    def __init__(self):
        self.update_rate = 5

        #Publish output here
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # FC, FR, RL, RR, init w/ x,y co-ords on the automower
        self.front_center = BoundarySensor(0.3, 0)
        self.front_right = BoundarySensor(0.3, -0.07)
        self.rear_left = BoundarySensor(-0.15, 0.12)
        self.rear_right = BoundarySensor(-0.15, -0.12)
        self.observation_count = 0
        self.interval_time = 0

        # Control parameters
        self.cp_very_close = 12000
        self.cp_med_close = 10000
        self.cp_close = 7000
        self.cp_target = 7500
        self.cp_far = 5500
        self.cp_very_far = 3000

        self.dist_multiplier = (15000 - self.cp_target)/15000

        self.name = "Steering"

        very_close = T1_RightShoulder(self.cp_target + 1000, self.cp_target + 2000, self.cp_target + 3000)
        # med_close = T1_Triangular(self.cp_close, self.cp_med_close, self.cp_very_close)
        close = T1_Triangular(self.cp_target, self.cp_target + 1000, self.cp_target + 2000)
        medium = T1_Triangular(self.cp_target - 1000, self.cp_target, self.cp_target + 1000)
        far = T1_Triangular(self.cp_target - 2000, self.cp_target - 1000, self.cp_target)
        very_far = T1_LeftShoulder(self.cp_target - 3000, self.cp_target - 2000, self.cp_target - 1000)

        # Right is -ve
        # Left is +ve
        right = T1_Triangular_output(-1, -0.5, 0.25, 1)
        # right_med = T1_Triangular_output(-0.1, 0.125, 1)
        right_sharp = T1_LeftShoulder_output(-1*self.dist_multiplier, -0.5, 0, 1)
        left = T1_RightShoulder_output(-0.25, 0.5, 1, 1)
        left_shallow = T1_Triangular_output(0, 0.1, 0.2, 1)
        straight = T1_Triangular_output(-0.25, 0, 0.25, 1)
        Output_set = [left, straight, right, right_sharp, left_shallow]

        Rule_1 = [[close], right, "If Close then Right"]
        Rule_2 = [[medium], straight, "If Medium then Straight"]
        Rule_3 = [[far], left, "If Far then Left"]
        Rule_4 = [[very_close], right_sharp, "If Very Close then Sharp Right"]
        Rule_5 = [[very_far], left_shallow, "If Very Far then Shallow Left"]
        # Rule_6 = [[med_close], right_med, "If Medium Close then Med Right"]
        self.Rule_set = [Rule_1,
                         Rule_2,
                         Rule_3,
                         Rule_4,
                         Rule_5]

        # input with mean and sigma
        self.input_obj1 = T1_Gaussian(3, 1)
        input_set = [self.input_obj1]

    def update(self):
        # Get sensor value avg's since last update
        #
        if self.avg_loop() != -1:
            self.input_obj1 = T1_Gaussian(self.front_center.value, self.front_center.sd)
            self.evalFIS(self.input_obj1)

            output = centroid(aggregate(self.Rule_set, "max"))
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

    def evalFIS(self, input):
        print "Mean = " + str(input.mean) + ", S.D. = " + str(input.step)

        for rule in self.Rule_set:
            rule[1].set_max_fs(rule_output(input, rule[0], "min"))
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
    automower_fuzzy = FIS()
    automower_fuzzy.run()

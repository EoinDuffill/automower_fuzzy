#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 27 15:28:46 2019
@author: direnc
"""


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


    # check the consistency of the antecedents and input
    if (len(rule) != len(my_input)):  # !!
        raise Exception("The antecedents and inputs numbers are not equal")

    fs = []
    for index in range(len(rule)):  # !!
        if my_input[index].step == 0:
            fs.append(rule[index].get_degree(my_input[index].mean))
        else:
            FSs_interation = inter_union(rule[index], my_input[index], 100)
            #TO DO tr different parameters (i.e cen_NS, sim_NS and sub_NS) 
            fs.append(FSs_interation.return_firing_stregth("sim_NS"))

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


class FIS(object):

    def __init__(self, observe):
       
        self.x = []
        self.counter = 0
        self.mean_list = []

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

        rule_1_s = [[very_close],right_sharp_pair,"If Very Close then Right Sharp"]
        rule_2_s = [[close], right_pair,"If Close then Right"]
        rule_3_s = [[medium], straight_pair, "If Medium then Straight"]
        rule_4_s = [[far], left_pair, "If Far then Left"]
        rule_5_s = [[very_far], left_shallow_pair, "If Very Far then Left Shallow"]

        rule_set_single = [rule_1_s,
                           rule_2_s,
                           rule_3_s,
                           rule_4_s,
                           rule_5_s]

        self.Rule_set = self.split_ruleset(rule_set_base)
        # self.Rule_set = self.split_ruleset(rule_set_single)
        self.inverse_rules = False

        # input with mean and sigma
        self.input_obj1 = T1_Gaussian(0, 1)
        self.input_obj2 = T1_Gaussian(0, 1)

        self.plots = draw_fis([
            [very_close, close, medium, far, very_far],
            [negative, none, positive]],
            [right_sharp_pair, right_pair, straight_pair, left_pair, left_shallow_pair])
        self.plots.plot()



    def create_output_pair(self, f1, f2, params):
        return f1(params[0], params[1], params[2], 1), f2(-params[2], -params[1], -params[0], 1)

    def split_ruleset(self, ruleset):

        rules_new = []
        rules_new_inv = []

        for rule in ruleset:
            rules_new.append([rule[0], rule[1][0], rule[2]])
            rules_new_inv.append([rule[0], rule[1][1], rule[2]])

        return (rules_new, rules_new_inv)

    def update(self,inpt1,estimated_noise1,inpt2,estimated_noise2):
        # Get sensor value avg's since last update
        #
        self.input_obj1 = T1_Gaussian(inpt1, estimated_noise1)
        if(inpt2):
            self.input_obj2 = T1_Gaussian(inpt2, estimated_noise2)


            # multi input
            inputs = [self.input_obj1, self.input_obj2]  # !!
            # single input
            # inputs = [self.input_obj1]

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



        return

    def evalFIS(self, inputs):
        # print "--- Multi Inputs ----"  # !!
        # for input in inputs:  # !!
        #     print "Mean = " + str(input.mean) + ", S.D. = " + str(input.step)  # !!
        # print "---------------"  # !!


        self.counter += 1


        file_str = str(self.counter) + ", "
        for input in inputs:
            file_str += str(input.mean) + "\t"+ str(input.step) + "\t"
        file_str = file_str[:-1]
        file_str += "\n"


        if self.inverse_rules:
            rules = self.Rule_set[1]
        else:
            rules = self.Rule_set[0]

        for index, rule in enumerate(rules):
            rule[1].set_max_fs(float(rule_output(inputs, rule[0], "min")))
            #print rule[2]+"\t" + str(rule[1].max_fs)



if __name__ == '__main__':
    automower_fuzzy = FIS(False)
    model_inputs=pd.read_csv("/home/psxdp/Desktop/Husqvarna/automower_fuzzy/src/experiments/completed_experiments/sim_noise_free_comparison/fuzzy_inputs.csv", header=None)
    
    for index,row in model_inputs.iterrows():
        automower_fuzzy.update(row[1],0,row[3],0)

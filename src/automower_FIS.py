#!/usr/bin/env python
import rospy
import numpy as np
from am_driver.msg import Loop


class BoundarySensor(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.accum = 0
        self.value = 0


class Input(object):
    def __init__(self, name, MFs, domain):
        self.name = name
        self.MFs = MFs
        self.domain = domain


class Output(object):
    def __init__(self, name, MFs, domain):
        self.name = name
        self.MFs = MFs
        self.domain = domain


class Rule(object):
    def __init__(self, ante1, ante2, consequent, impM):
        self.ante1 = ante1
        self.ante2 = ante2
        self.impM = impM
        self.consequent = consequent
        self.firing_strength = 1

    def rule_fire(self, x):

        ante1_result = None
        ante2_result = None

        if self.ante1 is not None:
            ante1_result = self.ante1.execMF(x)
        if self.ante2 is not None:
            ante2_result = self.ante2.execMF(x)

        self.firing_strength = self.impM(ante1_result, ante2_result)


class MembershipFunction(object):

    def __init__(self, name, function, params):
        self.name = name
        self.func = function
        self.params = params

    def execMF(self, x):
        #debug
        return self.func(self.params, x)


def trimf(params, x):
    # Outside the support of the func
    if x < params[0] or x > params[2]:
        return 0
    # At the rise/peak of tri
    elif x <= params[1]:
        return line_up(params[0], params[1], x)
    # At the fall of tri
    elif x < params[2]:
        return line_down(params[1], params[2], x)
    else:
        return -1


def trapmf(params, x):
    # Outside the support of the func
    if x < params[0] or x > params[3]:
        return 0
    # At the peak/shoulder of func
    elif params[1] <= x <= params[2]:
        return 1
    # At the rise of trap
    elif x < params[1]:
        return line_up(params[0], params[1], x)
    # At the fall of trap
    elif x < params[3]:
        return line_down(params[2], params[3], x)
    else:
        return -1


def line_up(a, b, x):
    return ((x*1.0)/(b - a)) - ((a*1.0)/(b - a))


def line_down(a, b, x):
    return (((-x) * 1.0)/(b - a)) + 1 + ((a * 1.0)/(b - a))


def min(x,y):
    if x is None and y is None:
        return 0
    elif x is None:
        return y
    elif y is None:
        return x
    else:
        if y < x:
            return y
        else:
            return x


def centroid():
    return -1


class FIS(object):

    def __init__(self):
        self.update_rate = 1

        # FC, FR, RL, RR, init w/ x,y co-ords on the automower
        self.front_center = BoundarySensor(0.3, 0)
        self.front_right = BoundarySensor(0.3, -0.07)
        self.rear_left = BoundarySensor(-0.15, 0.12)
        self.rear_right = BoundarySensor(-0.15, -0.12)
        self.observation_count = 0
        self.interval_time = 0

        # Control parameters
        self.close_sensor_value = 10000
        self.target_sensor_value = 7500
        self.far_sensor_value = 5000

        self.name = "Steering"
        # self.implication_method = min
        self.defuzz_method = centroid

        # Input 1
        self.distanceMFs = np.array([
            MembershipFunction("Close", trapmf, [self.target_sensor_value, self.close_sensor_value, 15000, 15000]),
            MembershipFunction("Medium", trimf, [self.far_sensor_value, self.target_sensor_value, self.close_sensor_value]),
            MembershipFunction("Far", trapmf, [0, 0, self.far_sensor_value, self.target_sensor_value])
        ])

        # Output 1
        self.steeringMFs = np.array([
            MembershipFunction("Left", trimf, [-1, -0.5, 0]),
            MembershipFunction("Straight", trimf, [-0.5, 1, 0.5]),
            MembershipFunction("Right", trimf, [0, 0.5, 1])
        ])

        # Array of Inputs
        self.inputs = np.array([
            Input("Distance", self.distanceMFs, (0, 15000))
        ])

        # Array of Outputs
        self.outputs = np.array([
            Output("Steering", self.steeringMFs, (-1, 1))
        ])

        # Array of Rules
        self.rules = np.array([
            Rule(self.distanceMFs[0], None, self.steeringMFs[2], min),
            Rule(self.distanceMFs[1], None, self.steeringMFs[1], min),
            Rule(self.distanceMFs[2], None, self.steeringMFs[0], min)
        ])

        # print init vars
        print "------------"
        print "FIS: " + str(self.name) + ", Type 1 Mamdani" + ", Defuzz: " + str(self.defuzz_method.__name__)
        print "------------"
        # Inputs
        self.print_var("Inputs" ,self.inputs)
        print
        # Outputs
        self.print_var("Outputs", self.outputs)
        print "------------"

    def print_var(self, title, var):
        for i in range(len(var)):
            print title+" "+str(i+1)+": Name = "+str(var[i].name)+", Domain = "+str(var[i].domain)
            for j in range(len(var[i].MFs)):
                print "MF "+str(j+1)+ ": "+str(var[i].MFs[j].name) + \
                      "\t Func: "+str(var[i].MFs[j].func.__name__) + \
                      "\t Params: " + str(var[i].MFs[j].params)

    def evalFIS(self, x):
        print "X = " + str(x)
        # Fire each rule
        for i in range(len(self.rules)):
            self.rules[i].rule_fire(x)
            print "Rule "+ str(i+1) + ": Ante 1: " +str(self.rules[i].ante1.name) + \
                  "\t Ante 2: " + str(self.rules[i].ante2) +\
                  "\t Firing Str: " + str(self.rules[i].firing_strength)
        print

    # Accumulate loop sensor observations
    def parse_loop(self, data):

        self.observation_count += 1
        self.front_center.accum += data.frontCenter
        self.front_right.accum += data.frontRight
        self.rear_left.accum += data.rearLeft
        self.rear_right.accum += data.rearRight

    # Calculate avg since last avg calc
    def avg_loop(self):
        # AVG sensor values since last update
        # return -1 is no observations have been made (no sensor update)
        # return +ve if observations > 0

        if self.observation_count == 0:
            return -1
        else:
            self.front_center.value = self.front_center.accum / self.observation_count
            self.front_right.value = self.front_right.accum / self.observation_count
            self.rear_left.value = self.rear_left.accum / self.observation_count
            self.rear_right.value = self.rear_right.accum / self.observation_count

            self.front_center.accum = 0
            self.front_right.accum = 0
            self.rear_left.accum = 0
            self.rear_right.accum = 0

            self.observation_count = 0
            return 1

    def update(self):
        # Get sensor value avg's since last update
        if self.avg_loop() >= 0:
            self.evalFIS(self.front_center.value)
        return

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

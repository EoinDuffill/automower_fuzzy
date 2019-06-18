#!/usr/bin/env python
import rospy
import numpy as np


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
        print(self.func(self.params, x))
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
    return (x/(b - a)) - (a/(b - a))


def line_down(a, b, x):
    return (-x/(b - a)) + 1 + (a/(b - a))


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

        self.name = "Steering"
        # self.implication_method = min
        self.defuzz_method = centroid

        # Input 1
        self.distanceMFs = np.array([
            MembershipFunction("Close", trimf, [2, 3, 4]),
            MembershipFunction("Medium", trimf, [3, 4, 5]),
            MembershipFunction("Far", trimf, [4, 5, 6])
        ])

        # Output 1
        self.steeringMFs = np.array([
            MembershipFunction("Left", trimf, [-1, -0.5, 0]),
            MembershipFunction("Straight", trimf, [-0.5, 1, 0.5]),
            MembershipFunction("Right", trimf, [0, 0.5, 1])
        ])

        # Array of Inputs
        self.inputs = np.array([
            Input("Distance", self.distanceMFs, (0, 5))
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

    def evalFIS(self):
        # Fire each rule
        for i in range(len(self.rules)):
            self.rules[i].rule_fire(3.14)

    def update(self):
        # get distance
        self.evalFIS()
        return

    def fini(self):
        print('Finishing...')

    def run(self):
        try:
            # Setup subscribers
            # rospy.Subscriber("loop", Loop, self.parse_boundary_sensors, queue_size=None)
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

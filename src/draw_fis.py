import numpy as np
import matplotlib.pyplot as plt

class draw_fis(object):

    def __init__(self, inputs, output):
        self.data = inputs

        list_a = []
        list_b = []

        for tuple in output:
            list_a.append(tuple[0])
            list_b.append(tuple[1])

        self.data.append(list_a)
        self.data.append(list_b)
        self.min = 16500
        self.max = 0

        self.input1_plots = []
        self.input2_plots = []

        self.initial_draw = False



    def plot(self):

        fig, plots = plt.subplots(4)

        x_points = []
        y_points = []

        for index, input in enumerate(self.data):

            x_min = 16500
            x_max = 0

            for mf in input:

                x_points, y_points, x_temp_min, x_temp_max = self.get_membership(mf)

                plots[index].plot(x_points,y_points, linewidth=1)

                if x_temp_min < x_min:
                    x_min = x_temp_min
                if x_temp_max > x_max:
                    x_max = x_temp_max

            plots[index].axis([x_min, x_max, 0, 1.05])

        plt.savefig("FIS.png")

    def plot_inference(self, rulesets, is_inverse, inputs, filename):

        if is_inverse:
            ruleset = rulesets[1]
        else:
            ruleset = rulesets[0]

        if not self.initial_draw:
            self.fig, self.plots = plt.subplots(15, 3)

        for index, rule in enumerate(ruleset):

            if not self.initial_draw:
                ante1_x, ante1_y, ante1_x_min, ante1_x_max = self.get_membership(rule[0][0])
                ante2_x, ante2_y, ante2_x_min, ante2_x_max = self.get_membership(rule[0][1])
                consq1_x, consq1_y, consq1_x_min, consq1_x_max = self.get_membership(rule[1])
                self.plots[index][0].plot(ante1_x, ante1_y)
                self.plots[index][0].axis([3000, 10500, 0, 1.05])
                self.plots[index][1].plot(ante2_x, ante2_y)
                self.plots[index][1].axis([-500, 500, 0, 1.05])
                self.plots[index][2].plot(consq1_x, consq1_y)
                self.plots[index][2].axis([-1, 1, 0, 1.05])

            input1_x, input1_y, in1_min, in1_max = self.get_membership(inputs[0])
            input2_x, input2_y, in2_min, in2_max = self.get_membership(inputs[1])

            input1_plot, = self.plots[index][0].plot(input1_x, input1_y)
            input2_plot, = self.plots[index][1].plot(input2_x, input2_y)

            self.input1_plots.append(input1_plot)
            self.input2_plots.append(input2_plot)

        #plt.tight_layout()
        plt.setp(self.plots, yticks=[0, 1])
        self.initial_draw = True
        plt.savefig("experiments/"+filename + ".png")
        self.remove_lines(self.input1_plots)
        self.remove_lines(self.input2_plots)

        self.input1_plots = []
        self.input2_plots = []




    def remove_lines(self, lines):
        for line in lines:
            line.remove()

    def get_membership(self, set):
        x_points = []
        y_points = []
        x_min = 16500
        x_max = 0

        (xs, ys) = set.get_mf_degrees(1)
        x_points.append(xs)
        y_points.append(ys)

        x_points = np.array(x_points)[0]
        y_points = np.array(y_points)[0]

        if min(x_points) < x_min:
            x_min = min(x_points)
        if max(x_points) > x_max:
            x_max = max(x_points)

        return x_points, y_points, x_min, x_max
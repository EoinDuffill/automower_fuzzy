#!/usr/bin/env python
import sys, math
import numpy as np
import matplotlib.pyplot as plt

from main import FIS
from draw_fis import draw_fis
from T1_set import T1_Gaussian, T1_Triangular, T1_RightShoulder, T1_LeftShoulder


def main():
    try:
        data = open(sys.argv[1], "r")
        print("Opening " + sys.argv[1] + "...")
    except IOError:
        print("Cannot find file: " + sys.argv[1])
        return
    except:
        print("Unexpected error occured")
        return

    try:
        step = int(sys.argv[2])
    except:
        print("No appropriate step given, defaulting to 10")
        step = 10

    print("Reading and Parsing Data...")



    line = data.readline()

    # List to hold x floats
    x_points = []
    # List of lists to hold all y floats (1 x point can have multiple y points associated with it)
    length = len(line.split("\n")[0].split(",")[1].split("\t"))
    y_points = [[] for _ in range(length)]

    while line:

        # Split the x point off before the comma
        line_x_point = line.split("\n")[0].split(",")[0].split("\t")
        # Split y points after the comma, and separate them from between the tab characters
        line_y_points = line.split("\n")[0].split(",")[1].split("\t")

        # append x point
        x_points.append(float(line_x_point[0]))
        # append each y point (on a given line) onto its own list
        for index, element in enumerate(line_y_points):
            y_points[index].append(float(line_y_points[index]))

        # read next line
        line = data.readline()


    print("Closing File...")
    data.close()
    print("Plotting points...")

    input1 = T1_Gaussian(0,1)
    input2 = T1_Gaussian(0,1)
    inputs = []
    fis = FIS(True)

    plot_quantity = math.ceil((len(y_points[0])*1.0)/step)

    print "0%"
    for index, set in enumerate(y_points[0]):
        if index % step == 0:
            input1 = T1_Gaussian(y_points[0][index], y_points[1][index])
            input2 = T1_Gaussian(y_points[2][index], y_points[3][index])
            inputs = [input1, input2]
            filename = "inference_" + str(index)

            fis.plots.plot_inference(fis.Rule_set, fis.inverse_rules, inputs, filename)
            print str ((((index / step) + 1) * 100) / plot_quantity) + "%"

if __name__ == "__main__":
    main()

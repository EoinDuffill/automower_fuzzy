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

        print len(list_b)

        self.data.append(list_a)
        self.data.append(list_b)

        print len(self.data)


    def plot(self):
        print "plotting"

        fig, plots = plt.subplots(4)

        x_points = []
        y_points = []



        for index, input in enumerate(self.data):

            x_min = 16500
            x_max = 0

            for mf in input:
                x_points = []
                y_points = []

                (xs, ys) = mf.get_mf_degrees()
                x_points.append(xs)
                y_points.append(ys)

                x_points = np.array(x_points)[0]
                y_points = np.array(y_points)[0]

                if index == 3:
                    print x_points
                    print y_points

                plots[index].plot(x_points,y_points, linewidth=1)

                if min(x_points) < x_min:
                    x_min = min(x_points)
                if max(x_points) > x_max:
                    x_max = max(x_points)

            plots[index].axis([x_min, x_max, 0, 1.05])

        plt.show()


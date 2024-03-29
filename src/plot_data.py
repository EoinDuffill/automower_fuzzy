#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt


def main():

	try:
		data = open(sys.argv[1],"r")
		print("Opening "+sys.argv[1]+"...")
	except IOError:
		print("Cannot find file: "+sys.argv[1])
		return
	except:
		print("Unexpected error occured")
		return

	print("Reading and Parsing Data...")
	line = data.readline()

	# List to hold x floats
	x_points = []
	# List of lists to hold all y floats (1 x point can have multiple y points associated with it)
	length= len(line.split("\n")[0].split(",")[1].split("\t"))
	y_points = [[] for _ in range(length)]

	while line:
		
		# Split the x point off before the comma
		line_x_point = line.split("\n")[0].split(",")[0].split("\t")
		# Split y points after the comma, and separate them from between the tab characters
		line_y_points = line.split("\n")[0].split(",")[1].split("\t")

		# append x point
		x_points.append(float(line_x_point[0]))
		# append each y point (on a given line) onto its own list
		for index,element in enumerate(line_y_points):
			y_points[index].append(float(line_y_points[index]))

		# read next line
		line = data.readline()
	
	print("Closing File...")
	data.close()
	print("Plotting points...")

	# Clear points and axis
	plt.cla()
	plt.clf()

	# vars for min and max bounds in x and y
	min_x_bounds = 0
	max_x_bounds = 0
	min_y_bounds = 0
	max_y_bounds = 0

	for list in y_points:
		x = np.array(x_points)
		y = np.array(list)
		# Plot parsed points
		plt.plot(np.array(x_points), np.array(list))

		print(sum(list) / len(list))

		# Find and update min/max bounds in x and y
		if np.amin(x) < min_x_bounds:
			min_x_bounds = np.amin(x)
		if np.amax(x) > max_x_bounds:
			max_x_bounds = np.amax(x)
		if np.amin(y) < min_y_bounds:
			min_y_bounds = np.amin(y)
		if np.amax(y) > max_y_bounds:
			max_y_bounds = np.amax(y)

	print(min_x_bounds, max_x_bounds)


	# axis sqaured off, uniform in x and y
	# plt.axis('equal')
	# Set axis based on min and max vals in x and y
	plt.axis([min_x_bounds, max_x_bounds, min_y_bounds, 1500])

	#font = {'fontname':'Times New Roman'}

	plt.suptitle('Simulated Magnetic Field Strengh around a lap of the lawn')
	plt.xlabel('Recorded Sensor Readings',fontsize=12)
	plt.ylabel('Magnetic Field Strength',fontsize=14)
	# Save figure to file
	print("Saving Plot...")
	plt.savefig('plot.png')


if __name__ == "__main__":
	main()

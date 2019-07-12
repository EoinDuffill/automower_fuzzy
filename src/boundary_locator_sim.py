#!/usr/bin/env python2
import rospy
import numpy as np

from am_driver.msg import Loop
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates

class Boundary_finder(object):
    def __init__(self):
        self.file = open("boundary_co-ords.txt", "a")
        self.update_rate = 50

        self.initialised = False
        self.ready = True
        self.last_reading = None


        # SIM
        # Get initial variables from simulator
        data = rospy.wait_for_message("gazebo/model_states", ModelStates)
        # get the index into the data for the automower
        index = data.name.index("automower")

        # initial x,y #SIM
        self.x = data.pose[index].position.x
        self.y = data.pose[index].position.y

    def parse_loop(self, data):
        if not self.initialised:
            if data.frontCenter > 0:
                self.ready = True
            else:
                self.ready = False
            self.initialised = True

        self.last_reading = data.frontCenter

    def update(self):
        # SIM
        # Get initial variables from simulator
        data = rospy.wait_for_message("gazebo/model_states", ModelStates)
        # get the index into the data for the automower
        index = data.name.index("automower")

        # initial x,y #SIM
        self.x = data.pose[index].position.x
        self.y = data.pose[index].position.y

        if self.initialised:
            if self.ready and self.last_reading < 0:
                self.file.write(str(self.x) + ", " + str(self.y) + "\n")
                print "Boundary found at: "+ str(self.x) +", "+ str(self.y)
                self.ready = False
            elif not self.ready and self.last_reading >= 0:
                self.ready = True

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
    rospy.init_node('boundary_finder')
    automower_fuzzy = Boundary_finder()
    automower_fuzzy.run()
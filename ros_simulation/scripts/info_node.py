#! /usr/bin/env python

import rospy
import math
import time

from ros_simulation.msg import Pos

class InfoNode:
    def __init__(self):
        self.frequency = rospy.get_param('publish_frequency')
        self.old_time = 0
        self.des_x = rospy.get_param('des_x')
        self.des_y = rospy.get_param('des_y')
        self.sub = rospy.Subscriber('/pos', Pos, self.callback)

    def callback(self, data):
        current_time = time.time() * 1000
        if current_time - self.old_time > 1000 / self.frequency:
            distance = math.sqrt((self.des_x - data.x)**2 + (self.des_y - data.y)**2)
            velocity = math.sqrt(data.vx**2 + data.vy**2)
            rospy.loginfo('Distance from desired position: %f', distance)
            rospy.loginfo('Average velocity: %f', velocity)
            self.old_time = current_time

def main():
    rospy.init_node('info_node')
    InfoNode()
    rospy.spin()

if __name__ == '__main__':
    main()


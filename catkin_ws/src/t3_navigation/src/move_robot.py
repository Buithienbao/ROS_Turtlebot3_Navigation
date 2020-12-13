#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveRobot:
    def __init__(self):
        rospy.init_node('MoveRobot') 
        # publish to /cmd_vel topic to send velocity information to the robot 
        self.pNode = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # subscribe to /scan topic to get laser information 
        self.sNode = rospy.Subscriber('/scan', LaserScan, self.callback)

        self.twistMsg = Twist()

        self.laserMsg = LaserScan()

        rospy.spin()

    # LaserScan Message callback
    def callback(self, msg):

        self.laserMsg = msg
        # If the distance between a robot and a wall is less than 1m, stop
        if self.getLaserDistance() < 1:
            self.twistMsg.linear.x = 0.0

        # Keep moving if the distance is greater than 1m.
        if self.getLaserDistance() > 1:
            self.twistMsg.linear.x = 0.5

        self.pNode.publish(self.twistMsg) 

    # Get laser distance value at 90 degree to know if there are any obstacle in front of the robot or not
    def getLaserDistance(self):
        if len(self.laserMsg.ranges) == 0:
            return 0
        return self.laserMsg.ranges[0]

            
# Create an object of class MoveRob         
obj1 = MoveRobot()

#! /usr/bin/env python
from geometry_msgs.msg import PoseStamped

class Location:

    def __init__(self, pose, name):
        self.positionX = pose.pose.position.x
        self.positionY = pose.pose.position.y
        self.name = name

    def stampedPose(self):
        pose = PoseStamped()
        pose.pose.position.x = self.positionX
        pose.pose.position.y = self.positionY
        return pose

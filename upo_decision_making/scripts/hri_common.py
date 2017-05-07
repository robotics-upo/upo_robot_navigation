#!/usr/bin/env python
"""
This file contains utility classes / methods that are common to all HRI actions
"""

import rospy as rp
import numpy as np
import tf
import re
import math
import yaml
import abc
import threading
import os
import sys
import pdb
from geometry_msgs.msg import TransformStamped, PoseStamped
#from spencer_tracking_msgs.msg import TrackedPersons
#from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class WaypointParser():
    def __init__(self, waypoint_defs_path):
        with open(waypoint_defs_path, 'r') as f:
            wayp_dict = yaml.load(f)

        self.wayp_dict = {}
        for (label, wp) in wayp_dict.items():
            x = wp['x']
            y = wp['y']
            theta = wp['theta']
            self.wayp_dict[label] = self.waypoint_to_ps(x, y, theta)

    def get_waypoint(self, label):
        return self.wayp_dict[label]

    def get_all_waypoints(self):
        return self.wayp_dict

    @staticmethod
    def waypoint_to_ps(x, y, theta):
        p = PoseStamped()
        p.header.stamp = rp.Time.now()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(theta/2.0)
        p.pose.orientation.w = math.cos(theta/2.0)
        return p


def to_stamped_transform(base_frame, target_frame, trans, rot):
    """
    Transforms a TF between two frames from (trans, rot) to TransformStamped
    """
    tfstamped = TransformStamped()
    tfstamped.child_frame_id = target_frame
    tfstamped.header.stamp = rp.Time.now()
    tfstamped.header.frame_id = base_frame
    tfstamped.transform.translation.x = trans[0]
    tfstamped.transform.translation.y = trans[1]
    tfstamped.transform.translation.z = trans[2]
    tfstamped.transform.rotation.x = rot[0]
    tfstamped.transform.rotation.y = rot[1]
    tfstamped.transform.rotation.z = rot[2]
    tfstamped.transform.rotation.w = rot[3]
    return tfstamped




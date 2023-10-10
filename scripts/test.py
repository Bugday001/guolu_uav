#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
给fast-planner依次发布目标点坐标
"""
import time
import re
import math
import rospy
import tf
import yaml
from rospkg import RosPack
import numpy as np
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty
from mavros_msgs.msg import RCIn


recordFile = open("/home/denext/catkin_ws/src/px4_offboard/params/record.yaml", 'r+')
recordData = yaml.safe_load(recordFile)
print(recordData)
recordData["index"] =  1+20

print(recordData)
recordFile.seek(0)
recordFile.truncate()
yaml.safe_dump( recordData,  recordFile)
recordFile.close()
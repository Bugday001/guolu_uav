#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
给fast-planner依次发布目标点坐标
"""
import time
import json
import re
import math
import rospy
import tf
from rospkg import RosPack
import numpy as np
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from std_msgs.msg import Empty
from std_msgs.msg import String
from mavros_msgs.msg import RCIn
from sendPath import sendPath

rp = RosPack()
# states
LAND = 0
toLAND = 1
FLY = 2
# 是否启用续飞功能
Resume = 0



class receive_command():
    def __init__(self):
        rospy.Subscriber("/mqtt/takeoff", String, self.takeoff_cb)
        rospy.Subscriber("/mqtt/flypath", String, self.flypath_cb)
        self.pathList = np.array([])
        self.sender = sendPath(np.array([[0,0,0,0]]), False)

    def takeoff_cb(self,msg):        
        print(msg.data)
        if msg.data=="takeoff" and len(self.pathList) != 0:
            self.sender.pathList = self.pathList
            self.sender.wait_server = True
            print("takeoff successfully!!")
            

    def flypath_cb(self,msg):
        print(msg.data)

        arr = np.fromstring(msg.data, dtype=float, sep=',')
        print(arr)
        point_num=int(len(arr)/4)

        self.pathList=np.reshape(arr, (point_num, 4))
        if(len(self.pathList) != 0):
            print("get path successfully!!")
            print(self.pathList)


if __name__=="__main__":
    rospy.init_node("sendPath")

    receiver=receive_command()

    rate = rospy.Rate(0.4) # 
    # while not rospy.is_shutdown():
    #     # state = sender.sendTargetPoint()
    #     if(sender.state==LAND):
    #         sender.landing()
    #         break
    #     rate.sleep()
    
    rospy.spin()

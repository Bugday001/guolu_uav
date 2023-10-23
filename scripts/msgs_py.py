#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
    mqtt messages
"""
import json

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from mavros_msgs.msg import State
from sensor_msgs.msg import Temperature

class sendJson():
    def __init__(self):
        self.pub = rospy.Publisher('/ping/ros', String, queue_size=10)
        self.state = {"time": 0.0, "position": [], "orientation": [], "battery": [],"temperature" : [] , "flystate" : []}
        
        rospy.Subscriber("/mavros/local_position/pose_transformed", PoseStamped, self.position_cb)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb)
        rospy.Subscriber("/mavros/state", State, self.flystate_cb)
        rospy.Subscriber("/mavros/imu/temperature_imu", Temperature, self.temperature_cb)
        rospy.Timer(rospy.Duration(1), self.time_cb, oneshot=False)
        
    def position_cb(self, msg):
        cur_position =  msg.pose.position
        cur_orientation =  msg.pose.orientation
        self.state["position"] = [cur_position.x, cur_position.y, cur_position.z]
        self.state["orientation"] = [cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]
        self.state["time"] = rospy.Time.now().to_sec()#msg.header.stamp.to_sec()

    def battery_cb(self, msg):
        self.state["battery"] = [msg.voltage,-(msg.current) , msg.percentage]

    def flystate_cb(self, msg):
        self.state["flystate"] = [msg.mode]
        if not msg.armed:
            self.state["flystate"] = ["LAND"]

    def temperature_cb(self, msg):
        self.state["temperature"] = [msg.temperature]
        
    def time_cb(self, event):
        json_string = json.dumps(self.state)
        self.pub.publish(str(json_string))

if __name__=="__main__":
    rospy.init_node("msgs_py")
    sender = sendJson()

    rospy.spin()

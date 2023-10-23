#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from visualization_msgs.msg import Marker, MarkerArray

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
rospy.init_node('register')

markerArray = MarkerArray()
count = 0

# 画一个在(1,2,3)的红色球
marker1 = Marker()
marker1.header.frame_id = "world"
marker1.type = marker1.SPHERE
marker1.action = marker1.ADD
marker1.scale.x = 0.2
marker1.scale.y = 0.2
marker1.scale.z = 0.2
marker1.color.a = 1.0
marker1.color.r = 1.0
marker1.color.g = 0.0
marker1.color.b = 0.0
marker1.pose.orientation.w = 1.0
marker1.id = count
count += 1
marker1.pose.position.x = 1.0
marker1.pose.position.y = 2.0
marker1.pose.position.z = 3.0

# 画一个在(-1,-2,-3)的蓝色球
marker2 = Marker()
marker2.header.frame_id = "world"
marker2.type = marker2.SPHERE
marker2.action = marker2.ADD
marker2.scale.x = 0.2
marker2.scale.y = 0.2
marker2.scale.z = 0.2
marker2.color.a = 1.0
marker2.color.r = 0.0
marker2.color.g = 0.0
marker2.color.b = 1.0
marker2.pose.orientation.w = 1.0
marker2.id = count
count += 1
marker2.pose.position.x = -1.0
marker2.pose.position.y = -2.0
marker2.pose.position.z = -3.0

# 将两个球添加到MarkerArray中，并发布它们
markerArray.markers.append(marker1)
markerArray.markers.append(marker2)

r = rospy.Rate (3) # 10hz
while not rospy.is_shutdown ():
    publisher.publish(markerArray)  
    r.sleep ()

rospy.spin()
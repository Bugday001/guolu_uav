#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import time
import re
import math
import rospy
import tf
from mavros_msgs.msg import PlayTuneV2

        


if __name__=="__main__":
    rospy.init_node("test")
    pub = rospy.Publisher("/mavros/play_tune", PlayTuneV2, queue_size=10)
    msg = PlayTuneV2()
    msg.format = 1
    msg.tune = 'CDECCDECEFG P4 EFG P4 G8A8G8F8EC G8A8G8F8EC DO2GO3C P4 DO2GO3C P4 '
    rate = rospy.Rate(1) # 
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    rospy.spin()

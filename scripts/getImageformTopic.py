#! /usr/bin/env python3
# -*- coding: UTF-8 -*-


import rospy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


import cv2
import time
import os
import numpy as np



class get():
    def __init__(self):
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        rospy.Subscriber("/hk/image_rect", Image, self.image_cb)
        #rospy.Subscriber("/hk_camera_node/image_raw", Image, self.image_cb)


        self.bridge = CvBridge()
        self.imgCnt = 0
        self.position = []
        self.cur_imagemsg=[]
        self.image_path="/home/denext/hk_image/"
        



        self.saveImg = []

        rospy.Timer(rospy.Duration(0.25), self.time_cb, oneshot=False)


    def image_cb(self,msg):
        self.cur_imagemsg=msg

        
     




        
    def position_cb(self, msg):
        cur_position =  msg.pose.position
        self.position = [cur_position.x, cur_position.y, cur_position.z]


    def time_cb(self, event):
        
        if self.cur_imagemsg.header:
            
            cv_img = self.bridge.imgmsg_to_cv2(self.cur_imagemsg, "rgb8")
            cv_img = np.ascontiguousarray(cv_img)
            image_name = self.image_path+str(self.imgCnt)+"_"+str(self.position)+".png"
            cv2.imwrite(image_name, cv_img,[cv2.IMWRITE_PNG_COMPRESSION,0]) 
            cv2.imshow("frame" , cv_img)
            cv2.waitKey(3)
            self.imgCnt += 1
    


if __name__=="__main__":
    rospy.init_node("msgs_py")
    geter=get()

    rospy.spin()

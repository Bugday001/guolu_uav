#! /usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import subprocess
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError

class sendVideo():
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_cb)
        width = 1920
        height = 1080
        self.new_width = 640
        self.new_heigh = 480
        fps = 5
        self.bridge = CvBridge()
        push_url_video = "rtsp://192.168.0.83:8554/stream"
        command = ['ffmpeg',
                                '-y', '-an',
                                '-f', 'rawvideo',
                                '-vcodec','rawvideo',
                                '-pix_fmt', 'bgr24', #像素格式
                                '-s', "{}x{}".format(self.new_width, self.new_heigh),
                                '-r', str(fps),
                                '-i', '-',
                                '-f', 'rtsp',
                                push_url_video] # rtsp rtmp
        self.pipe1 = subprocess.Popen(command, shell=False, stdin=subprocess.PIPE)
        # 创建视频捕获对象
        self.bridge = CvBridge()

        
    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # 显示Opencv格式的图像
        new_image = cv2.resize(cv_image, (self.new_width, self.new_heigh))
        self.pipe1.stdin.write(new_image.tostring())



if __name__=="__main__":
    rospy.init_node("msgs_py")
    sender = sendVideo()

    rospy.spin()

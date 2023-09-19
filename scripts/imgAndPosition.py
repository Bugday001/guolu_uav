#! /usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import subprocess
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import cv2
import time



class sendJson():
    def __init__(self):
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        self.pub = rospy.Publisher('/ping/primitive', String, queue_size=1)
        self.cap = cv2.VideoCapture(0)  # 0表示默认摄像头，如果有多个摄像头，可以尝试不同的索引值
        self.imgCnt = 0
        self.position = []
        self.ret = 0
        
        push_url_video = "rtsp://127.0.0.1:8554/local/video"

        fps = float(self.cap.get(5))
        width = int(self.cap.get(3))
        height = int(self.cap.get(4))
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", width, height)
        command = ['ffmpeg',
                                '-y', '-an',
                                '-f', 'rawvideo',
                                '-vcodec','rawvideo',
                                '-pix_fmt', 'bgr24', #像素格式
                                '-s', "{}x{}".format(width, height),
                                '-r', str(fps),
                                '-i', '-',
                                '-f', 'rtsp',
                                push_url_video] # rtsp rtmp
        self.pipe1 = subprocess.Popen(command, shell=False, stdin=subprocess.PIPE)
        cv2.namedWindow("image")     # 创建一个image的窗口
        # 创建视频捕获对象
        
        self.saveImg = []
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            print("无法打开摄像头")
            exit()
        # rospy.Timer(rospy.Duration(1), self.time_cb, oneshot=False)
        self.run()

    def run(self):
        # 开始采集和推流
        while True:# 采集一帧图像
            self.ret, self.frame = self.cap.read()
            if self.ret:# 通过FFmpeg编码和推流
                # self.saveImg = self.frame.copy()   
                self.pipe1.stdin.write(self.frame.tostring())

        
    def position_cb(self, msg):
        cur_position =  msg.pose.position
        self.position = [cur_position.x, cur_position.y, cur_position.z]


    def time_cb(self, event):
        # 读取一帧图像
        if self.ret:
            name = "../imgs/"+str(self.imgCnt)+"_"+str(self.position)+".jpg"
            cv2.imwrite(name, self.saveImg)
            self.imgCnt += 1


if __name__=="__main__":
    rospy.init_node("msgs_py")
    sender = sendJson()

    rospy.spin()

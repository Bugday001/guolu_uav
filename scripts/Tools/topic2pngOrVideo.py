#!/usr/bin/env python3
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
fps = 24  # 视频每秒24帧
size = (1080, 720)  # 需要转为视频的图片的尺寸

cam0_path  = '/home/denext/catkin_ws/src/px4_offboard/imgs/'    # 已经建立好的存储cam0 文件的目录
pose = [0,0,0]
video = cv2.VideoWriter("VideoTest1.mp4", cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
count = 0


def poseCallback(msg):
    cur_position =  msg.pose.position
    pose[0] = cur_position.x
    pose[1] = cur_position.y
    pose[2] = cur_position.z


def callback(data):
    # define picture to_down' coefficient of ratio
    global count,bridge, video
    count += 1
    cv_img = bridge.imgmsg_to_cv2(data, "mono8")
    timestr = "%.6f" %  data.header.stamp.to_sec()
            #%.6f表示小数点后带有6位，可根据精确度需要修改；
    image_name = timestr+str(pose)+ ".png" #图像命名：时间戳.jpg
    cv2.imwrite(cam0_path + image_name, cv_img)  #保存；
    # 视频保存不能使用
    # video.write(cv_img)
    
 
def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/pylon_camera_node/image_raw', Image, callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, poseCallback)
    rospy.spin()

if __name__ == '__main__':
    displayWebcam()
 
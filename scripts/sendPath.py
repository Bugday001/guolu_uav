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
from mavros_msgs.srv import CommandTOL, CommandLong
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty
from mavros_msgs.msg import RCIn ,State
from visualization_msgs.msg import Marker, MarkerArray


rp = RosPack()
# states
LAND = 0
toLAND = 1
FLY = 2
# 是否启用续飞功能
Resume = 0
class sendPath():
    def __init__(self, pathList, wait_server=True):
        self.TH = 0.30
        self.state = FLY
        self.pathList = pathList
        self.landingPosition = [0, 0, 1, 0]
        self.logFile = open(rp.get_path("px4_offboard") + "/logs/"+str(time.time())+".log","a",encoding="UTF-8")
        self.recordFile = open(rp.get_path("px4_offboard") + "/params/record.yaml", 'r+')
        self.tf_transform_matrix = np.array(rospy.get_param("/transformation")).reshape(4,4)
        self.recordData = yaml.safe_load(self.recordFile)
        self.lastPWM = -1
        self.lastUpdateTime = rospy.Time.now()
        self.cur_position = Point()
        self.wait_server = wait_server
        # 发布tf
        self.tf_tansform_pub = tf.TransformBroadcaster()
        if Resume:
            self.i = self.recordData["index"]
            self.targetPoint = self.pathList[self.i]
        else:
            self.i = 0
            self.targetPoint = self.pathList[0]
        self.init()
        self.pwm_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.island = False
        self.isOffboard = True
        self.TMatrix = np.eye(4)
        self.readTMatrix()  # 读取变换矩阵
        self.reformPath()
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_cb)
        rospy.Subscriber("mavros/state", State, self.stateCallback)
        self.waypoint_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size=10)
        self.transformed_pose_pub = rospy.Publisher("/mavros/local_position/pose_transformed", PoseStamped, queue_size=10)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, buff_size=1)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb)
        self.marker_pub = rospy.Publisher('/target_visual_point', MarkerArray, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.Timer_callback)
        
        self.rc_history = RCIn()

    def __del__(self):
        self.logFile.close()
        self.recordFile.close()

    def init(self):
        pass

    def Timer_callback(self, event):
        self.tf_publish()
        self.targets_pub()

    def stateCallback(self, msg):
        self.current_state = msg
        if self.current_state.mode == 'OFFBOARD' and self.wait_server:
            rospy.Subscriber("/planning/request_traget", Empty, self.sendTargetPoint, buff_size=1)
            self.isOffboard = True

    def matrix_to_euler_rad(self, matrix):
        q = tf.transformations.quaternion_from_matrix(matrix)
        eulers = tf.transformations.euler_from_quaternion(q, axes='sxyz')
        return eulers

    def reformPath(self):
        """
        进行修正，修正导航点和航向，~~最后一个是着陆的导航点，可以选择不进行变换~~
        """
        # 修正导航点
        T = self.TMatrix
        R = T[0:3,0:3]
        t = T[0:3, 3:4]
        loc = self.pathList[:, 0:3].T
        loc = np.dot(R.T, (loc-t))
        self.pathList[:, 0:3] = loc.T
        # 修正航向
        eulers = self.matrix_to_euler_rad(T)
        self.pathList[:, 3:4] -= eulers[2]
        print("path after fix: ")
        print(self.pathList)
    
    def readTMatrix(self):
        """
        读取初始姿态, 用于pathList的修正
        """
        path = rp.get_path("px4_offboard") + "/params/Transformation_maxtix.yaml"
        # 数据可能有多个空格分割，不能直接用np的库函数
        file = open(path, "r")
        lines = file.readlines()
        data = np.zeros([4, 4], np.float)
        cnt = 0
        for each in lines:
            i = 0
            while each[i]==' ':
                i += 1
            each = each[i:].replace('\n', '')
            nums = np.fromstring(each, dtype=float, sep=" ")
            data[cnt] = nums
            cnt += 1
        file.close()
        self.TMatrix = data
        print("变换矩阵：")
        print(data)
    
    def rc_cb(self, msg):
        """
        1950 1050
        """
        # 按键返航
        if len(self.rc_history.channels) == 0:
            self.rc_history = msg
            return
        if self.rc_history.channels[7] == 1050 and msg.channels[7]==1950 and self.state == FLY:  # 对应B按键
            self.state = LAND
            self.logFile.write(str(rospy.Time.now().to_sec())+" "+"RC LANDING"+"\n")
            print("RC LANDING")
        pwm = ((msg.channels[11]-1050.0)/900.0*2.0)+(-1)
        if abs(pwm-self.lastPWM) >= 0.1:
            rospy.wait_for_service('/mavros/cmd/command')
            # try:
            # 制作服务的handle
            cmd = CommandLong()
            cmd.broadcast = False
            cmd.command = 187
            cmd.confirmation = True
            cmd.param1 = pwm
            cmd.param2 = 0 # 参数2
            cmd.param3 = 0 # 参数3
            cmd.param4 = 0 # 参数4
            cmd.param5 = 0 # 参数5
            cmd.param6 = 0 # 参数6
            cmd.param7 = 0 # 参数7
            # 调用服务
            resp = self.pwm_srv(False, 187, True, pwm, 0,0,0,0,0,0)
            # except(rospy.ServiceException, e):
            # print("Service call failed: %s"%e)
        self.lastPWM = pwm
        self.rc_history = msg
    
    def battery_cb(self, msg):
        if msg.voltage < 18.5:
            self.state = LAND
            self.logFile.write(str(rospy.Time.now().to_sec())+" "+"LOW VOLTAGE LANDING"+"\n")
            print("LOW VOLTAGE LANDING")

    def reachGoal(self):
        """
        判断飞机当前位置和目标位置差是否都小于阈值
        """
        return abs(self.cur_position.x-self.targetPoint[0])<self.TH and \
        abs(self.cur_position.y-self.targetPoint[1])<self.TH and \
        abs(self.cur_position.z-self.targetPoint[2])<self.TH

    def position_cb(self, msg):
        """
        判断是否达到导航点，达到则加一
        """
        self.cur_position =  msg.pose.position
        if  self.i<len(self.pathList)-1 and self.reachGoal():
            print("add!!!!!!!!!!!!!!!!", self.i)
            self.i += 1
            self.targetPoint = self.pathList[self.i]
            if self.state == LAND:
                self.recordData["index"] = self.i-1
                print(self.recordData)
                self.recordFile.seek(0)
                self.recordFile.truncate()
                yaml.safe_dump(self.recordData, self.recordFile)
                self.i = len(self.pathList)-1
                self.targetPoint = self.landingPosition
        
        # ------------------------------发布锅炉坐标系下的无人机位姿态-------------------------
        T = self.TMatrix
        R = T[0:3,0:3]
        t = T[0:3, 3:4].T[0] # 1x3
        loc = np.array([self.cur_position.x, self.cur_position.y, self.cur_position.z])
        quatern = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        loc_matirx = tf.transformations.quaternion_matrix(quatern)[0:3, 0:3]
        loc_matirx = np.dot(R, loc_matirx)
        a = np.eye(4)
        a[0:3,0:3] = loc_matirx
        new_quatern = tf.transformations.quaternion_from_matrix(a)
        loc = ((R @ (loc)) )+t  # np.dot(R, (loc+t))
        new_pose = msg
        # print(loc)
        new_pose.pose.position.x = loc[0]
        new_pose.pose.position.y = loc[1]
        new_pose.pose.position.z = loc[2]
        # print(new_quatern)
        new_pose.pose.orientation.x = new_quatern[0]
        new_pose.pose.orientation.y = new_quatern[1]
        new_pose.pose.orientation.z = new_quatern[2]
        new_pose.pose.orientation.w = new_quatern[3]
        new_pose.header.frame_id = "global_frame"
        # print(new_pose)
        self.transformed_pose_pub.publish(new_pose)

    def sendTargetPoint(self, msg):
        """
            发送导航点及航向信息
            接受fast-planner的topic，回调
            可以修改为service
        """
        if self.isOffboard==False:
            return
        print("tagert point: " + str(self.targetPoint))
        self.logFile.write(str(rospy.Time.now().to_sec())+" "+str(self.targetPoint)+"\n")
        if len(self.targetPoint) == 0:
            return 1
        time_now = rospy.Time.now()
        init_path = Path()
        # 到达最后一个导航点
        if  self.reachGoal() and self.i==len(self.pathList)-1:
            if self.island:
                self.state = LAND
                # self.landing()
            else:
                self.targetPoint = self.landingPosition
                self.island = True
            return 
                
        
        init_path.header.stamp = time_now
        init_path.header.frame_id = "world"

        current_point = PoseStamped()
        current_point.header.frame_id = "camera_init"
        current_point.header.stamp = time_now
        current_point.pose.position.x = self.targetPoint[0]
        current_point.pose.position.y = self.targetPoint[1]
        current_point.pose.position.z = self.targetPoint[2]
        #角度
        # self.targetPoint[3] = 0
        orientation = quaternion_from_euler(0, 0, self.targetPoint[3])
        current_point.pose.orientation.x = orientation[0]
        current_point.pose.orientation.y = orientation[1]
        current_point.pose.orientation.z = orientation[2]
        current_point.pose.orientation.w = orientation[3]
        init_path.poses.append(current_point)
        self.waypoint_pub.publish(init_path)

    def landing(self):
        """
        返航着陆
        """
        print("LAND")
        self.logFile.write(str(rospy.Time.now().to_sec())+" "+"Call landing service"+"\n")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            # 制作服务的handle
            land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            #调用服务
            resp = land_srv.call()
        except(rospy.ServiceException, e):
            print("Service call failed: %s"%e)
    
    def tf_publish(self):
        """
        发布tf
        """
        position = -self.TMatrix[0:3, 3:4]
        R = self.TMatrix[0:3,0:3].T
        position = R @ position
        self.tf_tansform_pub.sendTransform(position,
                              tf.transformations.quaternion_from_matrix(self.TMatrix.T),
                              rospy.Time.now(),
                              "global_frame",
                              "world")

    def targets_pub(self):
        markers = MarkerArray()
        a = 0
        for point in self.pathList:
            marker2 = Marker()
            marker2.header.frame_id = "world"
            marker2.type = marker2.SPHERE
            marker2.action = marker2.ADD
            marker2.scale.x = 0.5
            marker2.scale.y = 0.5
            marker2.scale.z = 0.5
            if a > self.i:
                marker2.color.a = 1.0
                marker2.color.r = 1.0
                marker2.color.g = 1.0
                marker2.color.b = 0.0
            elif a == self.i:
                marker2.color.a = 1.0
                marker2.color.r = 0.0
                marker2.color.g = 1.0
                marker2.color.b = 0.0
            else:
                marker2.color.a = 1.0
                marker2.color.r = 1.0
                marker2.color.g = 0.0
                marker2.color.b = 0.0
            marker2.pose.orientation.w = 1.0
            marker2.id = a
            marker2.pose.position.x = point[0]
            marker2.pose.position.y = point[1]
            marker2.pose.position.z = point[2]
            a += 1
            markers.markers.append(marker2)
        self.marker_pub.publish(markers)


if __name__=="__main__":
    # pathList = np.array([[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0],[0,1,1,0],[0,0,1,0]])
    pathList = np.array([[0,4,1,0], [0,4,2,0], [0,0,2,0],[0,0,3,0],
                                [0,4,3,0], [0,4,4,0], [0,0,4,0]])
    # pathList = np.array([[9,6,4,0],[-9,6,1,3.14/2],[-9,-6,3,3.14],[10,-6,3,-3.14/2],
    #                             [9,6,4,0],[-9,6,1,3.14/2],[-9,-6,3,3.14],[10,-6,3,-3.14/2]])
    rospy.init_node("sendPath")
    sender = sendPath(pathList.astype(np.float64))
    rospy.spin()

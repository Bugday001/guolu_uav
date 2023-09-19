#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""@trajectory_msg_converter.py
直接将fast-planner发布的轨迹传递给mavros，控制px4
使用/mavros/setpoint_raw/local，发布position, vel, acc
"""


import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist, PoseStamped
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


class MessageConverter:
    def __init__(self):
        rospy.init_node('point2mavros')
        self.position_raw = PositionTarget()
        self.current_state = State()
        self.target_point = PoseStamped()
        # 初始化起飞高度
        self.target_point.pose.position.x = 0.2
        self.target_point.pose.position.y = 0
        self.target_point.pose.position.z = 1
        # raw position init
        self.position_raw.position.x = 0
        self.position_raw.position.y = 0
        self.position_raw.position.z = 1
        self.position_raw.yaw = 0
        self.position_raw.coordinate_frame = 1
        self.position_raw.type_mask = PositionTarget.IGNORE_YAW_RATE

        self.target_att = AttitudeTarget()
        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/ref_traj')

        #  mavros/setpoint_raw/attitude
        self.position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.position_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
        rospy.Subscriber("mavros/state", State, callback = self.stateCallback)
        self.isOffboard = False
        rospy.Timer(period = rospy.Duration(0.1), callback = self.cmdloopCallback)
        rospy.spin()

    def stateCallback(self, msg):
        self.current_state = msg
        if self.current_state.mode == 'OFFBOARD':
            self.isOffboard = True

    def cmdloopCallback(self, event):
        self.position_raw_pub.publish(self.position_raw)
        # self.position_pub.publish(self.target_point)
    
    def fastPlannerTrajCallback(self, msg):
        if self.isOffboard == False:
            return
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot
        vel.angular.z = msg.yaw_dot
        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        self.target_point.header = msg.header
        self.target_point.pose.orientation.x = q[0]
        self.target_point.pose.orientation.y = q[1]
        self.target_point.pose.orientation.z = q[2]
        self.target_point.pose.orientation.w = q[3]
        self.target_point.pose.position = msg.position

        # all in one
        point = Point()
        point.x = msg.position.x
        point.y = msg.position.y
        point.z = msg.position.z
        vel_vector = Vector3()
        vel_vector.x = msg.velocity.x
        vel_vector.y = msg.velocity.y
        vel_vector.z = msg.velocity.z

        self.position_raw.header = msg.header
        self.position_raw.position = point
        self.position_raw.velocity = vel_vector
        self.position_raw.acceleration_or_force = msg.acceleration
        self.position_raw.yaw = msg.yaw


        # self.target_att.header = msg.header
        # self.target_att.body_rate.x = cmd(0);
        # self.target_att.body_rate.y = cmd(1);
        # self.target_att.body_rate.z = cmd(2);
        # self.target_att.type_mask = 128;  // Ignore orientation messages
        # self.target_att.orientation.w = target_attitude(0);
        # self.target_att.orientation.x = target_attitude(1);
        # self.target_att.orientation.y = target_attitude(2);
        # self.target_att.orientation.z = target_attitude(3);
        # self.target_att.thrust = cmd(3);
        

if __name__ == '__main__':
    obj = MessageConverter()
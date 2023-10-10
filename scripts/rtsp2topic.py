#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Initialize ROS node
rospy.init_node('video_publisher', anonymous=True)

# Create a publisher for the video stream
video_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)

# Initialize OpenCV video capture
cap = cv2.VideoCapture('rtsp://192.168.0.197:8554/gg')
# Initialize CvBridge for image conversion
bridge = CvBridge()

# Loop to read and publish video frames
while not rospy.is_shutdown():
    # Read a frame from the video
    ret, frame = cap.read()
    
    # retval, img_buffer = cv2.imencode(".jpg", frame, params = [cv2.IMWRITE_JPEG_QUALITY, 10])
    # img_decode = cv2.imdecode(np.frombuffer(img_buffer, np.uint8), 1)
    # new_image = cv2.resize(img_decode, (360, 240), interpolation=cv2.INTER_LINEAR)
    # Convert the frame to a ROS image message
    if ret:
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
    else:
        rospy.logerr("error rtsp2topic")
    
    # Publish the ROS image message
    video_pub.publish(ros_image)

# Release the OpenCV video capture and shutdown ROS node
cap.release()
rospy.shutdown()

#!/usr/bin/env python

import numpy as np 
import rospy
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
from sensor_msgs.msg import Image
import geometry_msgs.msg as geo_msg
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
   
#TODO: read params
image = mpimg.imread("/home/abdulla/codes/event_vision_ws/src/rpg_emvs/1.png")
#calibration files directory 
cal_mtx_dir = "/home/abdulla/codes/event_vision_ws/calibration_images/cal_mtx.sav"
cal_dist_dir = "/home/abdulla/codes/event_vision_ws/calibration_images/cal_dist_dir"
input_image_topic = "dvs/image_raw"

rospy.init_node('aruco_localizer', anonymous=True)
rate = rospy.Rate(100) # 100hz

#load calibration parameters
mtx = pickle.load(open(cal_mtx_dir, 'rb'))
dist = pickle.load(open(cal_dist_dir, 'rb'))
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

#define ros publishers and subscribers
aruco_image_publisher = rospy.Publisher('aruco_detection', Image, queue_size=10)
aruco_pose_publisher = rospy.Publisher('optitrack/davis', geo_msg.PoseStamped, queue_size=10)

camera_pose = geo_msg.PoseStamped()

initialized = False

initial_rot = []
initial_trans = []

def image_callback(ros_image):
    global initialized, initial_rot, initial_trans

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')


    #converting to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    corners = []
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners:
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        size_of_marker =  0.181 # side lenght of the marker in meter
        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

        frame_markers = aruco.drawAxis(frame_markers, mtx, dist, rvecs, tvecs, size_of_marker)
        aruco_image = bridge.cv2_to_imgmsg(frame_markers, encoding="bgr8")
        aruco_image_publisher.publish(aruco_image)

        rvecs = np.reshape(rvecs, (3,))
        tvecs = np.reshape(tvecs, (3,))


        cam_to_aruco_R = Rotation.from_rotvec(rvecs)
        cam_to_aruco_dcm = cam_to_aruco_R.as_dcm()
        aruco_to_cam_dcm = cam_to_aruco_dcm.transpose()
        aruco_to_cam_position = - np.matmul(aruco_to_cam_dcm, tvecs)
        aruco_to_cam_R = Rotation.from_dcm(aruco_to_cam_dcm)

        if not initialized:
            initial_trans = aruco_to_cam_position
            initial_rot = aruco_to_cam_dcm
        
        aruco_to_cam_position = np.matmul(aruco_to_cam_dcm, aruco_to_cam_position - initial_trans)  
        aruco_to_cam_R = Rotation.from_dcm(np.matmul(initial_rot, aruco_to_cam_dcm.transpose()))
            
            
        aruco_to_cam_quat = aruco_to_cam_R.as_quat()

        
        camera_pose.header.frame_id = "aruco_link"
        camera_pose.header.stamp = rospy.Time.now()

        camera_pose.pose.position.x = aruco_to_cam_position[0]
        camera_pose.pose.position.y = aruco_to_cam_position[1]
        camera_pose.pose.position.z = aruco_to_cam_position[2]

        camera_pose.pose.orientation.x = aruco_to_cam_quat[0]
        camera_pose.pose.orientation.y = aruco_to_cam_quat[1]
        camera_pose.pose.orientation.z = aruco_to_cam_quat[2]
        camera_pose.pose.orientation.w = aruco_to_cam_quat[3]


        initialized = True

        return initialized
        


rospy.Subscriber(input_image_topic, Image, image_callback)

while not rospy.is_shutdown():
    try:
        input_image = rospy.wait_for_message(input_image_topic, Image, timeout=0.05)
        initialized = image_callback(input_image)
    except:
        pass
    if initialized:
        aruco_pose_publisher.publish(camera_pose)
    
    rate.sleep()







#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import cv2.aruco as aruco
import math
from math import atan2,degrees



class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

               
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
                

	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
		Detected_ArUco_markers= self.detect_aruco(cv_image)
		ids, angles, centre = self.Calculate_orientation_in_degree(Detected_ArUco_markers)
		r = rospy.Rate(10)
		self.marker_msg.id = ids
		self.marker_msg.x = centre[0]
		self.marker_msg.y = centre[1]
		self.marker_msg.yaw = angles[ids]
		print(self.marker_msg)
		
		self.publish_data(self.marker_msg)
		print('Published!')
		r.sleep()
		
		
		
	def publish_data(self, marker_msg):
		self.marker_pub.publish(marker_msg)
		
	def detect_aruco(self,img):
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		parameters = aruco.DetectorParameters_create()
		parameters.adaptiveThreshWinSizeMin = 3
		parameters.adaptiveThreshWinSizeMax = 400
		Detected_ArUco_markers={}
	       
		# getting the corners, ids and rejected marker values
		corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
		if np.any(ids!= None):
			ids = ids.flatten()
			for (markerCorner, markerID) in zip(corners, ids):
				markerCorner = markerCorner.reshape((4, 2))
				Detected_ArUco_markers[markerID] = markerCorner

		return Detected_ArUco_markers
			
	def Calculate_orientation_in_degree(self, Detected_ArUco_markers):
		ArUco_marker_angles = {}
		for ids, corner in Detected_ArUco_markers.items():
			topLeft, topRight, bottomRight, bottomLeft = corner
			bottomRight = (np.float64(bottomRight[0]), np.float64(bottomRight[1]))
			topRight = (np.float64(topRight[0]), np.float64(topRight[1]))
			bottomLeft = (np.float64(bottomLeft[0]), np.float64(bottomLeft[1]))
			topLeft = (np.float64(topLeft[0]), np.float64(topLeft[1]))
		
			# compute the center (x, y)-coordinates of the ArUco marker
			cX = np.float64((topLeft[0] + bottomRight[0]) / 2)
			cY =np.float64((topLeft[1] + bottomRight[1]) / 2)
			centre = (cX, cY)
			# Since angle is atan2(-y,x), then converting that to degrees
			top_right_angle = (math.degrees(math.atan2(-corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]))) % 360
			angle = (top_right_angle + 45) % 360
			# ArUco_marker_angles[ids] = int(top_right_angle)
			ArUco_marker_angles[ids] = np.float64(angle)
			ids = np.int8(ids)

		return ids, ArUco_marker_angles, centre	


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()

#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time
import imutils
import math
from math import atan2,degrees


# detects ids and corners of all aruco markers
def detect_ArUco(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	parameters.adaptiveThreshWinSizeMin = 3
	parameters.adaptiveThreshWinSizeMax = 400
	Detected_ArUco_markers={}
       
	# getting the corners, ids and rejected marker values
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	print(ids)
	if np.any(ids!= None):
		ids = ids.flatten()
		for (markerCorner, markerID) in zip(corners, ids):
			markerCorner = markerCorner.reshape((4, 2))
			Detected_ArUco_markers[markerID] = markerCorner
	
	
	return Detected_ArUco_markers

# calculates orientation of images relative to the scale
def Calculate_orientation_in_degree(Detected_ArUco_markers):
	ArUco_marker_angles = {}
	
	if len(Detected_ArUco_markers) > 0:
		# loop over the detected ArUCo corners
		for markerID, markerCorner in Detected_ArUco_markers.items():
			if markerID!= None:
				topLeft, topRight, bottomRight, bottomLeft = markerCorner
				# convert each of the (x, y)-coordinate pairs to integers
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				# compute the center (x, y)-coordinates of the ArUco marker
				cX = int((topLeft[0] + bottomRight[0]) / 2)
				cY = int((topLeft[1] + bottomRight[1]) / 2)
				centre = (cX, cY)
				
				# calculate the midpoint between top left and top right corners
				mX = (int(topRight[0]) + int(topLeft[0]))//2
				mY = (int(topRight[1]) + int(topLeft[1]))//2
				
				# for orientation	
				# returns the arc tangent in radians
				angle_in_radians = math.atan2( (cY - mY),( mX - cX))
				
				#convert to degrees
				angle_in_degrees = int(math.degrees(angle_in_radians))
				

				# if relativeX < 0 and relativeY >= 0 ====> remains same
				if (mX - cX) < 0 and (cY - mY) >= 0:
					angle_in_degrees = angle_in_degrees
				
				# if relativeX < 0 and relativeY < 0 ====> 360 + theta
				elif (mX - cX) < 0 and (cY - mY) < 0:
					angle_in_degrees = 360 + angle_in_degrees
				
				# if relativeX > 0 and relativeY < 0 ====> 360 + theta
				elif (mX - cX) > 0 and (cY - mY) < 0:
					angle_in_degrees = 360 + angle_in_degrees		
				
				
				ArUco_marker_angles[markerID] = angle_in_degrees


	return ArUco_marker_angles	

# marks all the detected and calculated things
def mark_ArUco(img,Detected_ArUco_markers,angle):
	if len(Detected_ArUco_markers) > 0:
		for markerID, markerCorner in Detected_ArUco_markers.items():
			topLeft, topRight, bottomRight, bottomLeft = markerCorner
			# convert each of the (x, y)-coordinate pairs to integers
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			
			# marks all the corners with respective colours
			cv2.circle(img, topLeft, 4, (128, 128, 128), -1)
			cv2.circle(img, bottomRight,4, (255, 0, 255), -1)
			cv2.circle(img, topRight, 4,(0, 255, 0), -1)
			cv2.circle(img, bottomLeft, 4,(255, 255, 255), -1)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2)
			cY = int((topLeft[1] + bottomRight[1]) / 2)
			centre = (cX, cY)
			
			# marks the center
			cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
			
			mX = (int(topRight[0]) + int(topLeft[0]))//2
			mY = (int(topRight[1]) + int(topLeft[1]))//2
			midp = (mX, mY)
			
			# draw the marker for the blue line
			cv2.line(img, midp, centre, (255, 0, 0), 2)

			
			# for showing id number
			cv2.putText(img, str(markerID),
				(cX, cY - 15), cv2.FONT_HERSHEY_SIMPLEX,
				1, (0, 0, 255), 2)
			
			# for showing calculated angles for each marker	
			for ids, angle_in_degrees in angle.items(): 
				if ids== markerID:
					cv2.putText(img, str(angle_in_degrees),
					(int((topRight[0])), int((topRight[1])) - 15), cv2.FONT_HERSHEY_SIMPLEX,
					1, (0, 255, 0), 2)   

	return img


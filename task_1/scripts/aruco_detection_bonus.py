import pafy
import cv2
import time
from imutils.video import VideoStream
from aruco_library import *

cap = cv2.VideoCapture('input3.mp4')

if (cap.isOpened()== False): 
  print("Error opening video stream or file")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
   
size = (frame_width, frame_height)
result = cv2.VideoWriter('filename.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, size)
                         
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  
  if ret == True:

    # Display the resulting frame
    Detected_ArUco_markers = detect_ArUco(frame)						## detecting ArUco ids and returning ArUco dictionary
    angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statement
    img = mark_ArUco(frame,Detected_ArUco_markers,angle)					## marking the parameters of aruco which are mentioned in the problem statement	
    cv2.putText(img, 'SS_1141', (frame_width- 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
			                                                           	## showing each frame
    cv2.imshow('Frame',frame)     
    
    result.write(frame)
    
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break

  # Break the loop
  else: 
    break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()

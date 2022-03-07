#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Gripper, GripperResponse, GripperRequest

class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
        


    
    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

   
    def offboard_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException as e:
            print ("Service take off call failed: %s"%e)
            
            
    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
       
    def boxHandler(self, gripStatus):
        rospy.wait_for_service('/activate_gripper')
        try:
            pickupService = rospy.ServiceProxy('/activate_gripper', Gripper) 
            if gripStatus == 'pickUp':
                resp = pickupService(True)
            elif gripStatus == 'dropOff':
                resp = pickupService(False)
            print ("Response: %s"%(resp.result))
            return resp.result
        except rospy.ServiceException as e:
            print ("Service picking up call failed: %s"%e)        



class stateMoniter:

    def __init__(self):
        self.state = State()
    
        self.sp = PositionTarget()
       
        self.sp.type_mask = int('010111111000', 2)
     
        self.sp.coordinate_frame = 1
        
        self.local_pos = Point(0.0, 0.0, 0.0)
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg
    
    # Create more callback functions for other subscribers  
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z   
        
    def gripCb(self, msg):
        self.val= msg.data
     

    def image_callback(self, data):
        try:
            self.bridge = CvBridge() 
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

	 
        except CvBridgeError as e:
            print("nvdikbvedsivs")
            
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

## detecting ArUco ids and returning ArUco dictionary
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 400
        Detected_ArUco_markers={}
       
	
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        print(ids)
        if np.any(ids!= None):
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                    markerCorner = markerCorner.reshape((4, 2))
                    Detected_ArUco_markers[markerID] = markerCorner
						
        if len(Detected_ArUco_markers) > 0:	
            print("Detected")
            self.detect = "YES"
            print(self.detect)
        else:
            self.detect = "NO"
                    
def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 3.0, 0.0, 0.0, 3.0],
        [9.0, 0.0, 3.0, 3.0, 0.0, 3.0],
        [9.0, 0.0, 0.0, 0.0, 0.0, 3.0]
    ] 

    current_setPoint_index = 0

   

    # Create empty message containers 
    pos = PoseStamped()
    pos.pose.position.x = setpoints[current_setPoint_index][0]
    pos.pose.position.y = setpoints[current_setPoint_index][1]
    pos.pose.position.z = setpoints[current_setPoint_index][2]

    # Set your velocity here
    vel = Twist()
    vel.linear.x = setpoints[current_setPoint_index][3]
    vel.linear.y = setpoints[current_setPoint_index][4]
    vel.linear.z = setpoints[current_setPoint_index][5]
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    # Similarly initialize other subscribers
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.posCb)
    rospy.Subscriber("/gripper_check", String, stateMt.gripCb)
    rospy.Subscriber("/iris/camera/image_raw", Image, stateMt.image_callback) 


    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    # Publish the setpoints 
    while not rospy.is_shutdown():

  
        if current_setPoint_index < len(setpoints) - 1:
            print(current_setPoint_index)
            if abs(stateMt.local_pos.x - setpoints[current_setPoint_index][0]) < 0.013 and abs(stateMt.local_pos.y - setpoints[current_setPoint_index][1]) < 0.085 and abs(stateMt.local_pos.z - setpoints[current_setPoint_index][2]) < 0.12:
               
                if(stateMt.detect =="YES"):
                    ofb_ctl.setAutoLandMode()
                    print ("Entered auto land!")
   
                # elif(current_setPoint_index== 3 or current_setPoint_index==4):
                #     ofb_ctl.setAutoLandMode()
                #     print ("Entered auto land!")
                #     if (len(setpoints) > 4):
                #         if current_setPoint_index ==3:
                #             resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                #             if resp:
                #                 print("Picked up")
                #                 current_setPoint_index += 1
                #                 continue
                #         if current_setPoint_index ==5:
                #             resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                #             if not resp:
                #                 print("Dropped Off")
                #                 current_setPoint_index += 1
                #                 setpoints = setpoints[:1] + setpoints[4:]
                #                 setpoints.reverse()
                #                 print(setpoints)
                #                 current_setPoint_index= 0                      
                current_setPoint_index += 1
     
                          
            pos.pose.position.x = setpoints[current_setPoint_index][0]
            pos.pose.position.y = setpoints[current_setPoint_index][1]
            pos.pose.position.z = setpoints[current_setPoint_index][2]
            
            vel.linear.x = setpoints[current_setPoint_index][3]
            vel.linear.y = setpoints[current_setPoint_index][4]
            vel.linear.z = setpoints[current_setPoint_index][5]

        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

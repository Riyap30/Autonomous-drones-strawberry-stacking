#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Gripper, GripperResponse, GripperRequest

setpoints, current_waypoint = [], []
flag = 1

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
        # if gripStatus == 'pickUp':
        #     rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        #     attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        #     attach_srv.wait_for_service()
        #     rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        #     # Link them
        #     rospy.loginfo("Attaching iris and strawberry_box")
        #     req = AttachRequest()
        #     req.model_name_1 = "iris"
        #     req.link_name_1 = "base_link"
        #     req.model_name_2 = "strawberry_box"
        #     req.link_name_2 = "link"

        #     attach_srv.call(req)

        # elif gripStatus == 'dropOff':
        #     rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        #     attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        #     attach_srv.wait_for_service()
        #     rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        #     # Link them
        #     rospy.loginfo("Detaching iris and strawberry_box")
        #     req = AttachRequest()
        #     req.model_name_1 = "iris"
        #     req.link_name_1 = "base_link"
        #     req.model_name_2 = "strawberry_box"
        #     req.link_name_2 = "link"

        #     attach_srv.call(req)

        rospy.wait_for_service('/activate_gripper')
        try:
            pickupService = rospy.ServiceProxy('/activate_gripper', Gripper) 
            if gripStatus == 'pickUp':
                gripper_resp = pickupService(True)
            elif gripStatus == 'dropOff':
                gripper_resp = pickupService(False)
            print ("Gripper Response: %s \n"%(gripper_resp.result))
            return gripper_resp.result
        except rospy.ServiceException as e:
            print ("Service gripper function call failed: %s"%e)
    



class stateMoniter:

    def __init__(self):
        self.state = State()
    
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        self.sp.coordinate_frame = 1
        
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_orientation = []

        self.bridge = CvBridge()

        self.droneOverBox = False       #   0 --> No picked
                                        #   1 --> visible but not picked
                                        #   2 --> picked
                                        #   3 --> dropped

        self.droneCarrierStatus = 0

        self.matrix_coefficients = np.float32([[238.3515418007097, 0.0, 200.5], [0.0, 238.3515418007097, 200.5], [0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.float32([0.0, 0.0, 0.0, 0.0, 0.0])

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg
    
    # Create more callback functions for other subscribers  
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

        self.local_orientation = msg.pose.orientation
        
    def gripCb(self, msg):
        self.val= msg.data
   

    def getArucoLocation(self, img):

        global current_waypoint

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 400
            
        # getting the corners, ids and rejected marker values
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if ids != None:
            print(f"Ids detected: {ids}, Location: {corners[0][0]}")
            ids = ids.flatten()
            id = ids[0]
            location = corners[0]
            rvec, tvec, rejectedPoints = cv2.aruco.estimatePoseSingleMarkers(location, 0.06, self.matrix_coefficients, self.distortion_coefficients)
            
            location = location[0]

            rot_mat = np.matrix(cv2.Rodrigues(rvec)[0])
            transform1, transform2, center = np.zeros((4, 4)), np.zeros((4, 4)), np.array([0.0, 0.0, 0.0, 1.0]) #[(location[0][0]+location[2][0])//2, (location[0][1]+location[2][1])//2, 0, 1] #[(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1]
            
            transform1[:-1, :-1] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 1.0]]
            transform1[:, 3] = [self.local_pos.x, self.local_pos.y, self.local_pos.z, 1]
            
            transform2[:-1, :-1] = rot_mat
            transform2[:, 3] = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 1.0]

            result = np.dot(np.dot(transform1, transform2), np.transpose(center))

            print(f"Result: {result}")
            
            if flag!=0:
                self.droneCarrierStatus = 1
                
                if self.droneCarrierStatus> 0:
                    if len(current_waypoint) > 0 and abs(result[0] - current_waypoint[0]) < 0.01:
                        current_waypoint = [result[0], 0.0, 0.0]
                    else:
                        current_waypoint = [result[0], 0.0, 3.0]
                    
                    if abs(self.local_pos.z - result[2]) < 0.2:
                        print("Trying to pick up the parcel")
                        self.droneCarrierStatus = 2
         


            # if not self.droneOverBox:
            #     self.droneOverBox = True

            #     global setpoints

            #     setpoints.insert(2, [6.5, 0, 3, 3.0, 0, 0])
            #     setpoints.insert(3, [6.5, 0, 0, 0, 0, 3.0])

            #     print(f"Added setpoints box coordinates.... \n{setpoints}")

    def droneCamCallback(self, img):
        # print("Running callback.....")
        cv_img = []
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as err:
            print(f"Error while converting to cv2 image - {err}")
        
        if len(cv_img) > 0:
            self.getArucoLocation(cv_img)
            # print(f"Aruco marker at location: {arucoLocation}")


def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    global setpoints, current_waypoint
    setpoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 3.0],
        [0.0, 0.0, 3.0, 0.0, 0.0, 3.0], # taking off
        [9.0, 0.0, 3.0, 3.0, 0.0, 0.0], # travelling to drop point
        [9.0, 0.0, 0.0, 0.0, 0.0, 3.0], # dropping point
        [9.0, 0.0, 3.0, 0.0, 0.0, 3.0],#back up
        [0.0, 0.0, 3.0, 3.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #initial
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
    rospy.Subscriber("/iris/camera/image_raw", Image, stateMt.droneCamCallback)

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
        resp = ""

        if current_setPoint_index <len(setpoints):

            if stateMt.droneCarrierStatus == 2:
                print("Trying to pick up the box")
                resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                while not resp:
                    print("Couldn't pick up, attempting again")
                    resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                else:
                    print("Strawberry box picked up")
                    stateMt.droneCarrierStatus = 3

            elif stateMt.droneCarrierStatus == 3 and current_setPoint_index==4:
                global flag
                print("Trying to dop off the box")
                resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                while resp:
                    print(".", end="")
                    resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                else:
                    print("Box dropped at location")
                    stateMt.droneCarrierStatus = 0
                    flag =0

            if abs(stateMt.local_pos.x - setpoints[current_setPoint_index][0]) < 0.013 and abs(stateMt.local_pos.y - setpoints[current_setPoint_index][1]) < 0.085 and abs(stateMt.local_pos.z - setpoints[current_setPoint_index][2]) < 0.12:
                             
                # if len(setpoints) > 4:
                    # resp = ""
                    # if current_setPoint_index == 3:
                    # if stateMt.droneCarrierStatus == 2:
                    #     print("Trying to pick up the box")
                    #     resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                    #     while not resp:
                    #         print(".", end="")
                    #         resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                    #     else:
                    #         print("Strawberry box picked up")
                    #         stateMt.droneCarrierStatus = 3
                    # # elif current_setPoint_index == 5:
                    # elif stateMt.droneCarrierStatus == 4:
                    #     print("Trying to dop off the box")
                    #     resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                    #     while resp:
                    #         print(".", end="")
                    #         resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                    #     else:
                    #         print("Box dropped at location")
                    #         stateMt.droneCarrierStatus = 0
                    # if resp and resp != "":
                    #     print("Picked up %s"%current_setPoint_index)
                    # elif not resp and resp != "":
                    #     print("Dropped off %s"%current_setPoint_index)
                if current_setPoint_index<6:
                    current_setPoint_index += 1

            print(f"Current setpoint: {current_setPoint_index}")
           
            if stateMt.droneCarrierStatus == 1:                          
                pos.pose.position.x = current_waypoint[0]
                pos.pose.position.y = current_waypoint[1]
                pos.pose.position.z = current_waypoint[2]
                
                vel.linear.x = 3.0 if abs(current_waypoint[0] - setpoints[current_setPoint_index-1][0]) > 0.2 else 0.0 #-3.0 if current_waypoint[0] - setpoints[current_setPoint_index-1][0] < -0.2 else 3.0 if current_waypoint[0] - setpoints[current_setPoint_index-1][0] > 0.2 else 0.0
                vel.linear.y = 3.0 if abs(current_waypoint[1] - setpoints[current_setPoint_index-1][1]) > 0.2 else 0.0
                vel.linear.z = 3.0 if abs(current_waypoint[2] - setpoints[current_setPoint_index-1][2]) > 0.2 else 0.0
     
            else:
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
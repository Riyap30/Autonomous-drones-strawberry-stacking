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

setpoints, pickup_waypoint, strawberryBoxRowCoordinates = [], [], []
current_setPoint_index = 0

# Truck information
# For all truck indices purposes the red truck has index 0 and the blue one index 1
rows, columns = 4, 3
firstCellCoordinates = [[57.5, 63.75, 1.7], [14.85, -8.4, 1.7]]
deliveredBoxCount = [0, 0]
cellDimensions = [0.85, 1.23] # dimensio format - (length, width)
currentBoxIndex = -1 # 0 --> red, 1 --> blue

class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('edrone1/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

   
    def offboard_set_mode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException as e:
            print ("Service take off call failed: %s"%e)
            
    def setAutoLandMode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
    

    def boxHandler(self, gripStatus): 

        rospy.wait_for_service('/edrone1/activate_gripper')
        try:
            pickupService = rospy.ServiceProxy('/edrone1/activate_gripper', Gripper) 
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

        self.droneCarrierStatus = 0     #   0 --> Not visible 
                                        #   1 --> visible but unpicked
                                        #   2 --> locked in position
                                        #   3 --> picked
                                        #   4 --> dropped
                                        #   5 --> trajectory resumed

        self.matrix_coefficients = np.float32([[238.3515418007097, 0.0, 200.5], [0.0, 238.3515418007097, 200.5], [0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.float32([0.0, 0.0, 0.0, 0.0, 0.0])

        self.blue_L_limit=np.array([98,50,50]) 
        self.blue_U_limit=np.array([139,255,255]) 

        self.red_L_limit=np.array([0,89,22]) 
        self.red_U_limit=np.array([255,255,255])

        
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
   

    def setBoxColorIndex(self, img):
        global currentBoxIndex

        hasBlue, hasRed = 0, 0

        hsv =cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        b_mask=cv2.inRange(hsv, self.blue_L_limit, self.blue_U_limit)
        r_mask = cv2.inRange(hsv, self.red_L_limit, self.red_U_limit)

        hasBlue = np.sum(b_mask)
        hasRed = np.sum(r_mask)

        if hasBlue > hasRed:
            print(f"Blue color detected")
            currentBoxIndex = 1
        if hasRed > hasBlue:
            print(f"Red color detected")
            currentBoxIndex = 0

    def detectBoxLocation(self, img):

        global pickup_waypoint

        frameCenter = [img.shape[1]//2, img.shape[0]//2]

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

            arucoCenter = [(location[0][0] + location[2][0])//2, (location[0][1] + location[2][1])//2]

            rot_mat = np.matrix(cv2.Rodrigues(rvec)[0])
            transform1, transform2, center = np.zeros((4, 4)), np.zeros((4, 4)), np.array([0.0, 0.0, 0.0, 1.0]) #[(location[0][0]+location[2][0])//2, (location[0][1]+location[2][1])//2, 0, 1] #[(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1]
            
            transform1[:-1, :-1] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 1.0]]
            transform1[:, 3] = [self.local_pos.x, self.local_pos.y, self.local_pos.z, 1]
            
            transform2[:-1, :-1] = rot_mat
            transform2[:, 3] = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 1.0]

            result = np.dot(np.dot(transform1, transform2), np.transpose(center))

            print(f"Bounding box coordinates: {location[0]}, {location[2]}, frame center: {frameCenter}, aruco marker center: {arucoCenter}, Box difference from center: {[frameCenter[0] - arucoCenter[0], frameCenter[1] - arucoCenter[1]]}")
            print(f"Result: {result}")

            # If the box is detected for the first time, storing the setpoint to be continued from later
            if self.droneCarrierStatus == 0:
                setpoints.insert(current_setPoint_index, [self.local_pos.x, self.local_pos.y, self.local_pos.z, 0.0, 0.0, 0.0])
            
                self.droneCarrierStatus = 1

            if abs(frameCenter[0] - arucoCenter[0]) < 5.0:
                self.droneCarrierStatus = 2
                pickup_waypoint = [result[0], -12 , 0.2, 0, 0, 0]
                # cv2.imshow("Live stream", img)
                # cv2.waitKey(1)
                self.setBoxColorIndex(img) # cropped_img = img[y_start:y_end, x_start:x_end]
            else:
                pickup_waypoint = [result[0], -12, 3.0, 0, 0, 0]


    def droneCamCallback(self, img):
        cv_img = []
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as err:
            print(f"Error while converting to cv2 image - {err}")
        
        if len(cv_img) > 0 and self.droneCarrierStatus < 2:
            self.detectBoxLocation(cv_img)


def main():

    stateMt = [stateMoniter(), stateMoniter()]
    ofb_ctl = [offboard_control(), offboard_control()]
   
   # Initialize publishers
    local_pos_pub = [rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10),
     rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10) ]
    local_vel_pub = [rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10),
     rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)]
    # Specify the rate 
    rate = rospy.Rate(20.0)

    global setpoints, current_setPoint_index, currentBoxIndex, rows, firstCellCoordinates

    # A list of setpoints
    setpoints = [
     [0, 0, 3.0, 0.0, 0.0, 3.0], # taking off
        [0, -12, 3.0, 0.0, 3.0, 0.0], #row 13 red box left 
         #detects picks drops comes back
        [58.2, -12, 3.0, 3.0, 0.0, 0.0], #row 13 red box right
        [58.52, -29.2, 3.0, 0.0, 3.0, 0.0] #row 8 red box right
         #detects picks drops
    ] 
    dropGridLocation, dropSetpoint = [], []

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

    def setPublishSetpoint(setpoint):
        pos.pose.position.x = setpoint[0]
        pos.pose.position.y = setpoint[1]
        pos.pose.position.z = setpoint[2]
                
        vel.linear.x = setpoint[3]
        vel.linear.y = setpoint[4]
        vel.linear.z = setpoint[5]

    # rospy.Subscriber("/edrone0/mavros/state",State, stateMt[0].stateCb)
    # rospy.Subscriber("/edrone0/mavros/local_position/pose", PoseStamped, stateMt[0].posCb)
    # rospy.Subscriber("/edrone0/gripper_check", String, stateMt[0].gripCb)
    # rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt[0].droneCamCallback)   
    rospy.Subscriber("/edrone1/mavros/state",State, stateMt[1].stateCb)
    rospy.Subscriber("/edrone1/mavros/local_position/pose", PoseStamped, stateMt[1].posCb)
    rospy.Subscriber("/edrone1/gripper_check", String, stateMt[1].gripCb)
    rospy.Subscriber("/edrone1/camera/image_raw", Image, stateMt[1].droneCamCallback)

 
    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub[1].publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt[1].state.armed:
        ofb_ctl[1].setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt[1].state.mode=="OFFBOARD":
        ofb_ctl[1].offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    # Publish the setpoints 
    while not rospy.is_shutdown():
        print(stateMt[1].droneCarrierStatus)
        resp = ""

        if current_setPoint_index < len(setpoints)-1:

            if stateMt[1].droneCarrierStatus == 1 or (stateMt[1].droneCarrierStatus == 2 and stateMt[1].local_pos.z > 0.25):
                pickup_waypoint[3] = 1.0 if abs(stateMt[1].local_pos.x - pickup_waypoint[0]) > 0.05 else 0
                pickup_waypoint[4] = 1.0 if abs(stateMt[1].local_pos.y - pickup_waypoint[1]) > 0.05 else 0
                pickup_waypoint[5] = 1.0 if abs(stateMt[1].local_pos.z - pickup_waypoint[2]) > 0.05 else 0
                setPublishSetpoint(pickup_waypoint)
                
            elif stateMt[1].droneCarrierStatus == 2 and stateMt[1].local_pos.z < 0.25:
                print("Trying to pick up the strawberry box")
                resp = ofb_ctl[1].boxHandler(gripStatus='pickUp')
                while not resp:
                    print("Couldn't pick up, attempting again")
                    resp = ofb_ctl[1].boxHandler(gripStatus='pickUp')
                else:
                    print("Strawberry box picked up")
                    dropGridLocation = [deliveredBoxCount[currentBoxIndex]//rows, deliveredBoxCount[currentBoxIndex]%rows] # gives grid coordinates in form (row, column)
                    #dropSetpoint = [firstCellCoordinates[currentBoxIndex][0]+cellDimensions[1]*(dropGridLocation[0]+0.5), firstCellCoordinates[currentBoxIndex][1]+cellDimensions[0]*(dropGridLocation[1]+0.5), 4.0, 0.0, 0.0, 0.0]
                    dropSetpoint = [firstCellCoordinates[currentBoxIndex][0],  firstCellCoordinates[currentBoxIndex][1], 5.0, 0.0, 0.0, 0.0]
                    stateMt[1].droneCarrierStatus = 3
            
            elif stateMt[1].droneCarrierStatus == 3:
                if abs(stateMt[1].local_pos.x - dropSetpoint[0]) < 0.01 and abs(stateMt[1].local_pos.y - dropSetpoint[1]) < 0.01 and len(dropSetpoint) > 0:
                    dropSetpoint[2] = (deliveredBoxCount[currentBoxIndex]//rows + 1)*1.7
                    dropSetpoint[5] = 3.0 if abs(stateMt[1].local_pos.z - dropSetpoint[2]) > 0.01 else 0
                    print(dropSetpoint)
                    if abs(stateMt[1].local_pos.z - dropSetpoint[2]) < 3: 
                        stateMt[1].droneCarrierStatus= 4
            
                elif not (abs(stateMt[1].local_pos.x - dropSetpoint[0]) < 0.01 and abs(stateMt[1].local_pos.y - dropSetpoint[1]) < 0.01 and len(dropSetpoint) > 0):
                    dropSetpoint[3] = 3.0 if abs(stateMt[1].local_pos.x - dropSetpoint[0]) > 0.01 else 0
                    dropSetpoint[4] = 3.0 if abs(stateMt[1].local_pos.y - dropSetpoint[1]) > 0.01 else 0
                    dropSetpoint[5] = 3.0 if abs(stateMt[1].local_pos.z - dropSetpoint[2]) > 0.01 else 0
                setPublishSetpoint(dropSetpoint)
            
            elif stateMt[1].droneCarrierStatus == 4:
                print("Trying to dop off the box")
                resp = ofb_ctl[1].boxHandler(gripStatus='dropOff')
                print(resp)
                while resp:
                    print(".", end="")
                    resp = ofb_ctl[1].boxHandler(gripStatus='dropOff')
                else:
                    print("Box dropped at location")
                    stateMt[1].droneCarrierStatus = 5
                    setPublishSetpoint(setpoints[current_setPoint_index])

            elif abs(stateMt[1].local_pos.x - setpoints[current_setPoint_index][0]) < 0.013 and abs(stateMt[1].local_pos.y - setpoints[current_setPoint_index][1]) < 0.085 and abs(stateMt[1].local_pos.z - setpoints[current_setPoint_index][2]) < 0.12:
                if stateMt[1].droneCarrierStatus == 5:
                   print("Resuming path.....")
                   stateMt[1].droneCarrierStatus = 0
                   setpoints.remove(setpoints[current_setPoint_index])
                else:
                    current_setPoint_index += 1
                setPublishSetpoint(setpoints[current_setPoint_index])

            print(f"Current setpoint: {current_setPoint_index}")
           
        local_pos_pub[1].publish(pos)
        local_vel_pub[1].publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

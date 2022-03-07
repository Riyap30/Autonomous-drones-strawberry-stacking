#!/usr/bin/env python3
from calendar import c
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from std_msgs.msg import UInt8
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Gripper, GripperResponse, GripperRequest
from threading import Thread

setpoints, strawberryBoxRowCoordinates = [], []
current_setPoint_index= [0, 0]
# Truck information
# For all truck indices purposes the red truck has index 0 and the blue one index 1
rows, columns = 4, 3
firstCellCoordinates = [[56.5,64.75,1.7], [13.85, -7.4,1.7]] #coordinates of the initial truck cell
drone_setpoint_offset = [[1, -1, 0], [1, -61, 0]] #offset values for calculation of local coordinates
cellDimensions = [0.85, 1.23] #cell height and width
currentBoxIndex = [-1, -1] # will be used to set the box color values for both the drones
firstPickedDroneIndex = -1 # pick up status of the same colored boxes to avoid drone collision

class offboard_control:

    def __init__(self, dIndex=0):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
        self.droneIndex = dIndex

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('edrone'+str(self.droneIndex)+'/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('edrone'+str(self.droneIndex)+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

   
    def offboard_set_mode(self):
         # Calling to /mavros/set_mode to set offboard mode print fail message on failure
        rospy.wait_for_service('edrone'+str(self.droneIndex)+'/mavros/set_mode') # Waiting untill the service starts 
        try:
            setModeService = rospy.ServiceProxy('edrone'+str(self.droneIndex)+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException as e:
            print ("Service take off call failed: %s"%e)
            
    def setAutoLandMode(self):
        # Calling to /mavros/set_mode for auto landing print fail message on failure
        rospy.wait_for_service('edrone'+str(self.droneIndex)+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('edrone'+str(self.droneIndex)+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode="AUTO.LAND")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
    

    # deals with the picking and dropping of boxes
    def boxHandler(self, gripStatus): 

        rospy.wait_for_service('/edrone'+str(self.droneIndex)+'/activate_gripper')
        try:
            pickupService = rospy.ServiceProxy('/edrone'+str(self.droneIndex)+'/activate_gripper', Gripper) 
            if gripStatus == 'pickUp':
                gripper_resp = pickupService(True)
            elif gripStatus == 'dropOff':
                gripper_resp = pickupService(False)
            # print ("Gripper Response: %s \n"%(gripper_resp.result))
            return gripper_resp.result
        except rospy.ServiceException as e:
            print ("Service gripper function call failed: %s"%e)
    



class stateMoniter:

    def __init__(self, setPts = [], dIndex=0):
        self.state = State()
        self.droneIndex = dIndex
        self.pickup_waypoint = [] #waypoint where the box is detected
        self.deliveredBoxCount = [0, 0] #number of boxes delivered 
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        self.sp.coordinate_frame = 1
        self.dropGridLocation= [] #grid location used for dropping
        self.dropSetpoint = [] #actual drop setpoint calculation
        

        
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_orientation = []

        self.setPoints = setPts

        self.bridge = CvBridge()

        self.droneCarrierStatus = 0     #   0 --> Not visible 
                                                        #   1 --> visible but unpicked
                                                        #   2 --> picked
                                                        #   3 --> going to the drop point
                                                        #   4 --> checking drop location
                                                        #   5 --> dropped
                                                        #   6 --> trajectory resumed

        self.matrix_coefficients = np.float32([[238.3515418007097, 0.0, 200.5], [0.0, 238.3515418007097, 200.5], [0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.float32([0.0, 0.0, 0.0, 0.0, 0.0])

        # hsv values for blue
        self.blue_L_limit=np.array([98,50,50]) 
        self.blue_U_limit=np.array([139,255,255]) 

        # hsv values for red
        self.red_L_limit=np.array([0,33,92]) 
        self.red_U_limit=np.array([7,255,255])

        
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

        hasBlue, hasRed = 0, 0
        hsv =cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        b_mask=cv2.inRange(hsv, self.blue_L_limit, self.blue_U_limit) #masking the blue color
        r_mask = cv2.inRange(hsv, self.red_L_limit, self.red_U_limit) #masking the red color

        hasBlue = np.sum(b_mask)
        hasRed = np.sum(r_mask)
        
        print("red ", hasRed)
        print("blue ", hasBlue)
        if hasBlue >hasRed:
            print(f"Blue color detected")
            currentBoxIndex[self.droneIndex] = 1
        if hasRed > hasBlue:
            currentBoxIndex[self.droneIndex] =0 
        print(self.droneIndex)
        # print(self.currentBoxIndex[self.droneIndex])
    def detectBoxLocation(self, img):


        frameCenter = [img.shape[1]//2, img.shape[0]//2]

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 400
            
        # getting the corners, ids and rejected marker values
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if type(ids) == type(np.array([])):
            print(f"Ids detected: {ids}, Location: {corners[0][0]}")
            ids = ids.flatten()
            id = ids[0]
            location = corners[0]
            rvec, tvec, rejectedPoints = cv2.aruco.estimatePoseSingleMarkers(location, 0.06, self.matrix_coefficients, self.distortion_coefficients)
            
            location = location[0]

            arucoCenter = [(location[0][0] + location[2][0])//2, (location[0][1] + location[2][1])//2] #center of the aruco marker

            #transforming camera coordinates to gazebo coordinates
            rot_mat = np.matrix(cv2.Rodrigues(rvec)[0])
            transform1, transform2, center = np.zeros((4, 4)), np.zeros((4, 4)), np.array([0.0, 0.0, 0.0, 1.0]) #[(location[0][0]+location[2][0])//2, (location[0][1]+location[2][1])//2, 0, 1] #[(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1]
            
            transform1[:-1, :-1] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 1.0]]
            transform1[:, 3] = [self.local_pos.x, self.local_pos.y, self.local_pos.z, 1]
            
            transform2[:-1, :-1] = rot_mat
            transform2[:, 3] = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 1.0]

            result = np.dot(np.dot(transform1, transform2), np.transpose(center)) #converted result for the center of the marker

            print(f"Bounding box coordinates: {location[0]}, {location[2]}, frame center: {frameCenter}, aruco marker center: {arucoCenter}, Box difference from center: {[frameCenter[0] - arucoCenter[0], frameCenter[1] - arucoCenter[1]]}")
            print(f"Result: {result}")

            # If the box is detected for the first time, storing the setpoint to be continued from later
            if self.droneCarrierStatus == 0:
    
            
                self.droneCarrierStatus = 1
         
            if abs(frameCenter[0] - arucoCenter[0]) < 5.0: # centering the drone over the box
                self.droneCarrierStatus = 2

                self.pickup_waypoint = [result[0], setpoints[self.droneIndex][current_setPoint_index[self.droneIndex]][1], 0, 0, 0, 0]#point where the box is detected

            
                self.setBoxColorIndex(img[int(arucoCenter[1] - 50):int(arucoCenter[1] + 50), int(arucoCenter[0] - 50):int(arucoCenter[1] + 50)]) 

            else:       
                self.pickup_waypoint = [result[0], setpoints[self.droneIndex][current_setPoint_index[self.droneIndex]][1], 3.0, 0, 0, 0]#point where the box is detected
                print("drone", self.droneIndex)
                print("waypoints", self.pickup_waypoint)


    # deals with the camera capture and further operations
    def droneCamCallback(self, img):
        cv_img = []
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as err:
            print(f"Error while converting to cv2 image - {err}")
        
        if len(cv_img) > 0 and self.droneCarrierStatus < 2:
            self.detectBoxLocation(cv_img)

    #subscribing the row indexes of the boxes
    def droneSpawn(self, msg):
        self.ind= msg.data
        print("Row index: ", self.ind)
        if (self.ind <= 7): # drone0 deals with the first 7 rows
            setpoints[0].append( [0.0, (self.ind-1)*4, 3.0, 0.0, 0.2, 0.0]) # start row setpoint calculation based on row index
            setpoints[0].append([58.2, (self.ind-1)*4, 3.0, 0.2, 0.0, 0.0])# end row setpoint calculation based on row index
        else: # drone1 deals with the other rows
            setpoints[1].append( [0.0, -(60- (self.ind-1)*4), 3.0, 0.0, 0.2, 0.0])# start row setpoint calculation based on row index
            setpoints[1].append( [58.2, -(60- (self.ind-1)*4), 3.0, 0.2, 0.0, 0.0])# end row setpoint calculation based on row index
        # print(setpoints)
    
def processDroneMovement(droneIndex, rate, local_pos_pub, local_vel_pub, stateMt, ofb_ctl, pos, vel, setPublishSetpoint, setPublishSetpointTwo):
    
    current_setPoint_index


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


    while not rospy.is_shutdown():
  

        resp = ""

        if current_setPoint_index[droneIndex] < len(setpoints[droneIndex])-1:

            if stateMt.droneCarrierStatus == 1 or (stateMt.droneCarrierStatus == 2 and stateMt.local_pos.z > 0.25):
                # going closer to the box once detected
                stateMt.pickup_waypoint[3] = 0.2 if abs(stateMt.local_pos.x - stateMt.pickup_waypoint[0]) > 0.05 else 0
                stateMt.pickup_waypoint[4] = 0.2 if abs(stateMt.local_pos.y - stateMt.pickup_waypoint[1]) > 0.05 else 0
                stateMt.pickup_waypoint[5] = 0.2 if abs(stateMt.local_pos.z - stateMt.pickup_waypoint[2]) > 0.05 else 0
                setPublishSetpointTwo(stateMt.pickup_waypoint, droneIndex)


            elif stateMt.droneCarrierStatus == 2 and stateMt.local_pos.z < 0.25:
                # print("Trying to pick up the strawberry box")
                resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                while not resp:
                    # print("Couldn't pick up, attempting again")
                    resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                else:
                    global firstPickedDroneIndex
                    print("Strawberry box picked up")
                    print("color", currentBoxIndex[droneIndex])
                    if currentBoxIndex[0] ==currentBoxIndex[1]:
                        if firstPickedDroneIndex== -1:
                            firstPickedDroneIndex = droneIndex # indicates one drone has pixed the same color box
                            stateMt.dropGridLocation = [stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]//rows, stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]%rows]  # gives grid coordinates in form (row, column) gives 
                        else:
                            stateMt.dropGridLocation = [(stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]+1)//rows, (stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]+1)%rows]   # gives grid coordinates in form (row, column)
 

                    else:
                        stateMt.dropGridLocation = [stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]//rows, stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]%rows]
                    #calculating drop location
                    stateMt.dropSetpoint = [firstCellCoordinates[currentBoxIndex[droneIndex]][0]+drone_setpoint_offset[droneIndex][0]+cellDimensions[0]*(stateMt.dropGridLocation[0]),  firstCellCoordinates[currentBoxIndex[droneIndex]][1] + drone_setpoint_offset[droneIndex][1] +cellDimensions[1]*(stateMt.dropGridLocation[1]), 5.0+drone_setpoint_offset[droneIndex][0], 0.0, 0.0, 0.0]
                    print(stateMt.dropGridLocation, stateMt.dropSetpoint)
                    stateMt.droneCarrierStatus = 3
            
            elif stateMt.droneCarrierStatus == 3:
                if currentBoxIndex[0] == currentBoxIndex[1] and firstPickedDroneIndex!= droneIndex: 
                    #waiting point calculation beside the truck 
                    # the other drone waits at the calculated point until the first drone drops the box
                    # avoids drone collision 
                    waitPoint = [firstCellCoordinates[currentBoxIndex[droneIndex]][0] + drone_setpoint_offset[droneIndex][0] - 5.0, firstCellCoordinates[currentBoxIndex[droneIndex]][1] + drone_setpoint_offset[droneIndex][1] - 5.0, stateMt.dropSetpoint[2], 0.0, 0.0, 0.0]
                    waitPoint[3] = 0.2 if abs(stateMt.local_pos.x - waitPoint[0]) > 0.05 else 0
                    waitPoint[4] = 0.2 if abs(stateMt.local_pos.y - waitPoint[1]) > 0.05 else 0
                    waitPoint[5] = 0.2 if abs(stateMt.local_pos.z - waitPoint[2]) > 0.05 else 0
                    setPublishSetpointTwo(waitPoint, 1)
                else:
                    # going to the drop location
                    if abs(stateMt.local_pos.x - stateMt.dropSetpoint[0]) < 0.01 and abs(stateMt.local_pos.y - stateMt.dropSetpoint[1]) < 0.01 and len(stateMt.dropSetpoint) > 0:
                        stateMt.dropSetpoint[2] = (stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]]//rows + 1)*1.7
                        stateMt.dropSetpoint[3] = 0.0
                        stateMt.dropSetpoint[4]= 0.0
                        stateMt.droneCarrierStatus = 4
                
                    elif not (abs(stateMt.local_pos.x - stateMt.dropSetpoint[0]) < 0.01 and abs(stateMt.local_pos.y - stateMt.dropSetpoint[1]) < 0.01 and len(stateMt.dropSetpoint) > 0):
                        stateMt.dropSetpoint[3] = 1.0 if abs(stateMt.local_pos.x - stateMt.dropSetpoint[0]) > 0.01 else 0
                        stateMt.dropSetpoint[4] = 1.0 if abs(stateMt.local_pos.y - stateMt.dropSetpoint[1]) > 0.01 else 0
                        stateMt.dropSetpoint[5] = 1.0 if abs(stateMt.local_pos.z - stateMt.dropSetpoint[2]) > 0.01 else 0
                    setPublishSetpointTwo(stateMt.dropSetpoint, droneIndex)
            
            elif stateMt.droneCarrierStatus == 4:
                # checking z before dropping
                stateMt.dropSetpoint[5] = 3.0 if abs(stateMt.local_pos.z - stateMt.dropSetpoint[2]) > 0.01 else 0.0
                print(stateMt.dropSetpoint)
                if abs(stateMt.local_pos.z - stateMt.dropSetpoint[2]) < 2: 
                    stateMt.droneCarrierStatus= 5
                setPublishSetpointTwo(stateMt.dropSetpoint, droneIndex)
       
            elif stateMt.droneCarrierStatus == 5:
                print("Trying to dop off the box")
                resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                print(resp)
                while resp:
                    print(".", end="")
                    resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                else:
                    print("Box dropped at location")
                    stateMt.droneCarrierStatus = 6
                    currentBoxIndex[droneIndex]= -1 
                    stateMt.deliveredBoxCount[currentBoxIndex[droneIndex]] +=1 #increase delivered count after dropping
                    # setPublishSetpointTwo([stateMt.dropSetpoint[0], stateMt.dropSetpoint[1], 5.0, 0, 0, 3.0], droneIndex)
                    setPublishSetpointTwo(setpoints[droneIndex][current_setPoint_index[droneIndex]], droneIndex)

            elif abs(stateMt.local_pos.x - setpoints[droneIndex][current_setPoint_index[droneIndex]][0]) < 0.1 and abs(stateMt.local_pos.y - setpoints[droneIndex][current_setPoint_index[droneIndex]][1]) < 0.085 and abs(stateMt.local_pos.z - setpoints[droneIndex][current_setPoint_index[droneIndex]][2]) < 0.12 and -0.1301533 <= stateMt.local_orientation.x <= 0.1301533 and -0.1301533 <= stateMt.local_orientation.y <= 0.1301533:
                print("innnnnn")
                if stateMt.droneCarrierStatus == 6:
                    print("Resuming path.....")
                    stateMt.droneCarrierStatus = 0
                    firstPickedDroneIndex = -1
                
                current_setPoint_index[droneIndex] += 1
                setPublishSetpointTwo(setpoints[droneIndex][current_setPoint_index[droneIndex]], droneIndex)


            print(f"Current setpoint: {current_setPoint_index[droneIndex]}")
            # print(setpoints[1])
           
        # Publish the setpoints 
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()


def main():
    global setpoints
    setpoints = [
        # setpoints for eDrone0
        [ 
            [0, 0, 3, 3.0, 0.0, 3.0], # taking off
        
        ], 
        # setpoints for eDrone1
        [

            #setpoints for eDrone1
         [0, 0, 3.0, 3.0, 0.0, 3.0], # taking off
    
        ]
    ]
    stateMts = [stateMoniter(setpoints[0], 0), stateMoniter(setpoints[1], 1)]
    ofb_ctls = [offboard_control(0), offboard_control(1)]
   

    # Initialize publishers
    local_pos_pubs = [rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10),
    rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10) ]
    local_vel_pubs = [rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10),
    rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)]
    # Specify the rate 
    rate = rospy.Rate(20.0)



    # Create empty message containers 
    pos = [PoseStamped(), PoseStamped()]
    vel = [Twist(), Twist()]
    for i in range(2):
        pos[i].pose.position.x = stateMts[i].setPoints[current_setPoint_index[i]][0]
        pos[i].pose.position.y = stateMts[i].setPoints[current_setPoint_index[i]][1]
        pos[i].pose.position.z = stateMts[i].setPoints[current_setPoint_index[i]][2]
        vel[i].linear.x = stateMts[i].setPoints[current_setPoint_index[i]][3]
        vel[i].linear.y = stateMts[i].setPoints[current_setPoint_index[i]][4]
        vel[i].linear.z = stateMts[i].setPoints[current_setPoint_index[i]][5]
        

    def setPublishSetpoint(setpoint):
        pos.pose.position.x = setpoint[0]
        pos.pose.position.y = setpoint[1]
        pos.pose.position.z = setpoint[2]
                
        vel.linear.x = setpoint[3]
        vel.linear.y = setpoint[4]
        vel.linear.z = setpoint[5]

    def setPublishSetpointTwo(setpoint, i):
        pos[i].pose.position.x = setpoint[0]
        pos[i].pose.position.y = setpoint[1]
        pos[i].pose.position.z = setpoint[2]
                
        vel[i].linear.x = setpoint[3]
        vel[i].linear.y = setpoint[4]
        vel[i].linear.z = setpoint[5]

    #Subscribers
    rospy.Subscriber("/edrone0/mavros/state",State, stateMts[0].stateCb)
    rospy.Subscriber("/edrone1/mavros/state",State, stateMts[1].stateCb)

    rospy.Subscriber("/edrone0/mavros/local_position/pose", PoseStamped, stateMts[0].posCb)
    rospy.Subscriber("/edrone1/mavros/local_position/pose", PoseStamped, stateMts[1].posCb)

    rospy.Subscriber("/edrone0/gripper_check", String, stateMts[0].gripCb)
    rospy.Subscriber("/edrone1/gripper_check", String, stateMts[1].gripCb)

    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMts[0].droneCamCallback)
    rospy.Subscriber("/edrone1/camera/image_raw", Image, stateMts[1].droneCamCallback)

    rospy.Subscriber("/spawn_info", UInt8, stateMts[0].droneSpawn)

    #multithreading for simultaneous drone movement
    droneThreads = [Thread(target=processDroneMovement, args=(0,rate, local_pos_pubs[0], local_vel_pubs[0], stateMts[0], ofb_ctls[0], pos[0], vel[0], setPublishSetpoint, setPublishSetpointTwo)), 
                                Thread(target=processDroneMovement, args=(1,rate, local_pos_pubs[1], local_vel_pubs[1], stateMts[1], ofb_ctls[1], pos[1], vel[0], setPublishSetpoint, setPublishSetpointTwo))]

    for i in range(2):
        droneThreads[i].start()

    droneThreads[0].join()
    droneThreads[1].join()
  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
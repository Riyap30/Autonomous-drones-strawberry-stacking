#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
# import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Gripper, GripperResponse, GripperRequest

setpoints = []

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
        if gripStatus == 'pickUp':
            rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
            attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
            attach_srv.wait_for_service()
            rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

            # Link them
            rospy.loginfo("Attaching iris and strawberry_box")
            req = AttachRequest()
            req.model_name_1 = "iris"
            req.link_name_1 = "base_link"
            req.model_name_2 = "strawberry_box"
            req.link_name_2 = "link"

            attach_srv.call(req)

        elif gripStatus == 'dropOff':
            rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
            attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
            attach_srv.wait_for_service()
            rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

            # Link them
            rospy.loginfo("Detaching iris and strawberry_box")
            req = AttachRequest()
            req.model_name_1 = "iris"
            req.link_name_1 = "base_link"
            req.model_name_2 = "strawberry_box"
            req.link_name_2 = "link"

            attach_srv.call(req)

        rospy.wait_for_service('/activate_gripper')
        try:
            pickupService = rospy.ServiceProxy('/activate_gripper', Gripper) 
            if gripStatus == 'pickUp':
                gripper_resp = pickupService(True)
            elif gripStatus == 'dropOff':
                gripper_resp = pickupService(False)
            print ("Gripper Response: %s"%(gripper_resp.result))
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

        self.droneOverBox = False

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


    def quaternion2rotmat(self, Q):
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
    

    def getArucoLocation(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 400
            
        # getting the corners, ids and rejected marker values
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if ids:
            print(f"Ids detected: {ids}, Location: {corners[0][0]}")
            ids = ids.flatten()
            id = ids[0]
            location = corners[0]
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(location, 0.06, self.matrix_coefficients, self.distortion_coefficients)
            
            location = location[0]

            rot_mat = np.matrix(cv2.Rodrigues(rvec)[0])
            transform1, transform2, center = np.zeros((4, 4)), np.zeros((4, 4)), [(location[0][0]+location[2][0])//2, (location[0][1]+location[2][1])//2, 0, 1] #[(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1]
            
            transform1[:-1, :-1] = self.quaternion2rotmat([self.local_orientation.x, self.local_orientation.y, self.local_orientation.z, self.local_orientation.w])
            transform1[:, 3] = [self.local_pos.x, self.local_pos.y, self.local_pos.z, 1]
            
            transform2[:-1, :-1] = rot_mat
            transform2[:, 3] = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 1.0]

            # print(f"Transformation 1: \nRotation: {self.quaternion2rotmat([self.local_orientation.x, self.local_orientation.y, self.local_orientation.z, self.local_orientation.w])} \nTranslation: {[self.local_pos.x, self.local_pos.y, self.local_pos.z]}")
            # print(f"Transformation 2: \nRotation: {rot_mat} \nTranslation: {[tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 1.0]}")
            # print(f"Center point to be transformed: { [(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1] }")

            # print(f"Transformation 1: {transform1}")
            # print(f"Transformation 2: {transform2}")
            # print(f"Point to be transformed: {[(markerPoints[0][0][0]+markerPoints[1][0][0])//2, (markerPoints[0][0][1]+markerPoints[1][0][1])//2, (markerPoints[0][0][2]+markerPoints[1][0][2])//2, 1]}")

            result = np.dot(np.dot(transform1, transform2), np.transpose(center))

            print(f"Result: {result}")

            if not self.droneOverBox:
                self.droneOverBox = True

                global setpoints

                setpoints.insert(2, [6.5, 0, 3, 5, 0, 0])
                setpoints.insert(3, [6.5, 0, 0, 0, 0, 5])

                print(f"Added setpoints box coordinates.... \n{setpoints}")
                

            # print(f"Current position: {self.local_pos}")
            # print(f"Translation matrix: {tvec} \nRotation matrix: {rvec} \nMarker points: {markerPoints}")
            # cv2.aruco.drawDetectedMarkers(img, location)
            # cv2.aruco.drawAxis(img, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)

        cv2.imshow("Drone Live Stream", img)
        cv2.waitKey(1)
        # print(f"Ids: {ids}, Detected markers: {Detected_ArUco_markers}")
        # return []


    def droneCamCallback(self, img):
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
    global setpoints
    setpoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
        [0.0, 0.0, 3.0, 0.0, 0.0, 5.0], # taking off
        [9.0, 0.0, 3.0, 5.0, 0.0, 0.0], # travelling to drop point
        [9.0, 0.0, 0.0, 0.0, 0.0, 5.0], # dropping point
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
 
        if current_setPoint_index < len(setpoints):
        
            if abs(stateMt.local_pos.x - setpoints[current_setPoint_index][0]) < 0.013 and abs(stateMt.local_pos.y - setpoints[current_setPoint_index][1]) < 0.085 and abs(stateMt.local_pos.z - setpoints[current_setPoint_index][2]) < 0.12:
                
                if len(setpoints) > 4:
                    resp = ""
                    if current_setPoint_index == 3:
                        print("Trying to pick up the box from setpoint 3 ......")
                        resp = ofb_ctl.boxHandler(gripStatus='pickUp')
                    elif current_setPoint_index == 5:
                        print("Trying to dop off the box at setpoint 5 ......")
                        resp = ofb_ctl.boxHandler(gripStatus='dropOff')
                    if resp and resp != "":
                        print("Picked up %s"%current_setPoint_index)
                    elif not resp and resp != "":
                        print("Dropped off %s"%current_setPoint_index)
                
                current_setPoint_index += 1

            print(f"Current setpoint: {current_setPoint_index}")
           
                          
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
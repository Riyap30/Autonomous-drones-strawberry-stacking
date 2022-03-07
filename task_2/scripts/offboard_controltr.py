#!/usr/bin/env python3

'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file
This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:
	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


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
          [0.0, 0.0, 3.0, 0.0, 0.0, 5.0],
        [3.0, 0.0, 3.0, 5.0, 0.0, 0.0],
        [3.0, 0.0, 0.0, 0.0, 0.0, 5.0],
        [3.0, 3.0, 3.0, 0.0, 5.0, 5.0],
        [3.0, 3.0, 0.0, 0.0, 0.0, 5.0],
        [3.0, 3.0, 3.0, 0.0, 0.0, 5.0],
        [0.0, 0.0, 3.0, 5.0, 5.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 5.0]
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
        
            if abs(stateMt.local_pos.x - setpoints[current_setPoint_index][0]) < 1 and abs(stateMt.local_pos.y - setpoints[current_setPoint_index][1]) < 1 and abs(stateMt.local_pos.z - setpoints[current_setPoint_index][2]) < 1 :
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

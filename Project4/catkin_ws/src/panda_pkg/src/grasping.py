#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState


class Gripper:
    def __init__(self):
        self.finger1_pub = rospy.Publisher('/panda_finger_joint1/command', Float64, queue_size=10)
        self.finger2_pub = rospy.Publisher('/panda_finger_joint2/command', Float64, queue_size=10)
        rospy.sleep(1)
        
    def grasp(self, finger1_y, finger2_y):
        finger1_data = Float64()
        finger1_data.data = finger1_y
        finger2_data = Float64()
        finger2_data.data = finger2_y
        
        self.finger1_pub.publish(finger1_data)
        self.finger2_pub.publish(finger2_data)
        
Q3 =0
def callback3(data):
    global Q3
    Q3=data.process_value
    print(Q3)
    return

rospy.init_node("grap2e_node")        
gripper = Gripper()
rospy.Subscriber("/panda_finger_joint2/state", JointControllerState, callback3)

rate = rospy.Rate(20) 
while True:
    gripper.grasp(0.1,0.09)
    # input("Enter")
    # print(Q3)
    # rospy.spin()
    # rospy.sleep(2)
    rate.sleep()

#!/usr/bin/env python

#Guining Pertin
#DRDO SASE's UAV Fleet Challenge

#Create a unified probability distribution map from current position data and
#detection accuracy of 3 UAVs

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import Altitude
from threading import Thread

#local_position callback function
def local_pose_callback(data):
    local_pose = data

#altitude callback function
def altitude_callback(data):
    altitude = data

if __name__ == "__main__":
    #Set up prob_dist node
    rospy.init_node("Prob_Dist_node", anonymous = True)
    rospy.loginfo("Set up Probability Distribution Node")
    #UAV0
    #Subscribe to current position
    local_pose = PoseStamped()
    local_pose_sub = rospy.Subscriber("/uav0/mavros/local_position/pose",
                                         PoseStamped, local_pose_callback)
    #Subscribe to current altitude
    altitude = Altitude()
    altitude_sub = rospy.Subscriber("/uav0/mavros/altitude",
                                    Altitude, altitude_callback)
    #Subscribe to detection accuracies
    #We consider that the detection accuracy will be provided by us directly

    #Gaussian heuristic function based on height and detection accuracy
    


    #Spin
    rospy.spin()

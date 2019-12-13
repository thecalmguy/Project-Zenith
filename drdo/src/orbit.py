#!/usr/bin/env python

#Guining Pertin
#DRDO SASE's UAV Fleet Challenge

#Import required libraries
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
from mavros_msgs.msg import Altitude
from threading import Thread
from tf.transformations import euler_from_quaternion

#Create a UAV class to control each UAV
class UAV:
    #Initialize UAV
    def __init__(self, index):
        #Set UAV index
        self.uav_index = index

        #Subscribe to its local position topic
        self.local_pose = PoseStamped()
        self.local_pose_sub_name = "/uav{0}/mavros/local_position/pose".format(index)
        self.local_pose_sub = rospy.Subscriber(self.local_pose_sub_name,
                                             PoseStamped, self.local_pose_callback)
        rospy.loginfo("UAV{0} position subscribed".format(index))

        #Subscribe to its altitude topic
        self.altitude = Altitude()
        self.altitude_sub_name = "/uav{0}/mavros/altitude".format(index)
        self.altitude_sub = rospy.Subscriber(self.altitude_sub_name,
                                             Altitude, self.altitude_callback)
        rospy.loginfo("UAV{0} altitude subscribed".format(index))

        #Publish the required velocity_control topic
        self.vel_cont = Twist()
        self.vel_pub_name = "/uav{0}/vel_control_topic".format(index)
        self.vel_pub = rospy.Publisher(self.vel_pub_name, Twist, queue_size=10)
        rospy.loginfo("UAV{0} Vel_Control publisher set up".format(index))

        #Set up initial Twist message
        self.vel_cont.linear.x = 0.0
        self.vel_cont.linear.y = 0.0
        self.vel_cont.linear.z = 0.0
        self.vel_cont.angular.x = 0.0
        self.vel_cont.angular.y = 0.0
        self.vel_cont.angular.z = 0.0

        #Publish in a separate thread to better prevent failsafe
        self.pub_thread = Thread(target=self.publish_topic, args=())
        self.pub_thread.daemon = True
        self.pub_thread.start()

        #Set current position
        self.curr_pose = self.local_pose.pose
        #Set current altitude
        self.curr_alt = self.altitude.local

    #Function to publish vel_control topic(Running in a different thread)
    def publish_topic(self):
        rate = rospy.Rate(100)
        #Geometry_msgs/Twist type
        while not rospy.is_shutdown():
            #If running - publish the required velocity control topic
            self.vel_pub.publish(self.vel_cont)
            #Prevent garbage in console output when thread is killed
            try: rate.sleep()
            except rospy.ROSInterruptException: pass

    #Function to hold altitude
    def hold_altitude(self, req_alt, timeout):
        #Global variables
        global altitude_reached
        global start_reached
        rospy.loginfo("UAV{0} Reaching Altitude".format(self.uav_index))
        #Local variables
        offset = 0.05
        alpha = 5
        req_vel = 60
        #Set rate
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        while True:
            #Update current altitude
            self.curr_alt = self.altitude.local
            #If search completed - break
            if zam_search_completed: break
            #Current error
            error = req_alt - self.curr_alt
            #If error within offset
            if abs(error) < offset:
                #Inform it reached
                if not altitude_reached[self.uav_index]:
                    rospy.loginfo("UAV{0} Altitude Reached".format(self.uav_index))
                altitude_reached[self.uav_index] = True
            #Proportional controller to hold altitude
            #Proportional gain with an offset
            Kp = req_vel*(1-np.exp(-alpha*(error**2)))/abs(error) + 30
            #Set velocity
            self.vel_cont.linear.z = Kp*error
            #Since it runs a separate thread, use it to update global map
            if np.all(start_reached):
                grid_map[int(3*self.curr_pose.position.x+20), int(3*self.curr_pose.position.y+50)] = 1
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to align all UAV's during initiation
    def initial_align(self, req_x, req_y, timeout):
        #Global variables
        global start_reached
        rospy.loginfo("UAV{0} Aligning".format(self.uav_index))
        #Local variables
        alpha = 5
        req_vel = 100
        Kp_yaw = 20
        offset = 0.1
        #Set rate
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        for i in range(timeout*loop_freq):
            #Update current pose
            self.curr_pose = self.local_pose.pose
            #Update error vector
            error = [req_x - self.curr_pose.position.x, req_y - self.curr_pose.position.y]
            #Proportional controller to align yaw angles
            curr_yaw = self.get_yaw_angle()
            self.vel_cont.angular.z = Kp_yaw*(0-curr_yaw)
            #If error and yaw_velocityes are less than offsets, inform
            if (np.linalg.norm(error) < offset) and (abs(curr_yaw - 0) < 0.1):
                #Inform it reached
                if not start_reached[self.uav_index]:
                    rospy.loginfo("UAV{0} Aligned".format(self.uav_index))
                start_reached[self.uav_index] = True
            #Hold on until all UAVs are aligned
            if np.all(start_reached):
                self.vel_cont.angular.z = 0
                self.vel_cont.linear.x = 0
                self.vel_cont.linear.y = 0
                break
            #Proportional gain
            Kp = req_vel*(1-np.exp(-alpha*(np.linalg.norm(error)**2)))/np.linalg.norm(error)
            #Set velocites
            self.vel_cont.linear.x = Kp*error[0]
            self.vel_cont.linear.y = Kp*error[1]
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to get current yaw_angle
    def get_yaw_angle(self):
        curr_orientation = self.local_pose.pose.orientation
        #Get current yaw angle
        euler_angles = euler_from_quaternion([curr_orientation.x,
                                             curr_orientation.y,
                                             curr_orientation.z,
                                             curr_orientation.w])
        return euler_angles[2]

    #Function to reach required point with constant x velocity
    def reach_point(self, req_x, req_y, fin_vel, angle_hold, timeout):
        rospy.loginfo("UAV{0} Moving to Point ({1},{2})"
                      .format(self.uav_index, req_x, req_y))
        #Local variables
        offset_x = 0.05
        offset_y = 0.01
        alpha_y = 20
        req_vel_y = 60
        req_vel_x = 80
        Kp_yaw = 20
        #Set rate
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        for i in range(timeout*loop_freq):
            #Update current pose
            self.curr_pose = self.local_pose.pose
            #Update error vector
            error = [req_x - self.curr_pose.position.x, req_y - self.curr_pose.position.y]
            #X-velocity - Constant velocity controller
            #If within required x point
            if abs(error[0]) < offset_x:
                self.vel_cont.linear.y = 0
                self.vel_cont.linear.x = fin_vel
                self.vel_cont.angular.z = 0
                rospy.loginfo("UAV{0} Reached Point".format(self.uav_index))
                break
            #Set constant X velocity
            else:
                if error[0] >= 0:
                    self.vel_cont.linear.x = req_vel_x
                else:
                    self.vel_cont.linear.x = -req_vel_x
            #Y-velocity - Proportional controller
            #If not within offset use controller
            if abs(error[1]) > offset_y:
                #Proportional gain
                Kp_y = req_vel_y*(1-np.exp(-alpha_y*(error[1]**2)))/abs(error[1])
                #Set y velocity
                self.vel_cont.linear.y = Kp_y*(error[1])
            #Else shut y velocity - otherwise causes divide by 0 error
            else: self.vel_cont.linear.y = 0
            #Align yaw angles
            curr_yaw = self.get_yaw_angle()
            self.vel_cont.angular.z = Kp_yaw*(angle_hold-curr_yaw)
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to perform orbit
    def perform_orbit(self, dist, req_vel, req_angle, timeout):
        rospy.loginfo("UAV{0} Performing Orbit".format(self.uav_index))
        #Local variables
        init_y = self.local_pose.pose.position.y
        init_yaw = self.get_yaw_angle()
        turned = False
        #Set rate
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        #Required velocity
        for i in range(timeout*loop_freq):
            #Update current pose
            self.curr_pose = self.local_pose.pose
            #Get current yaw angle
            yaw_angle = self.get_yaw_angle()
            if abs(yaw_angle - req_angle) < 0.01 :
                self.vel_cont.linear.y = 0
                self.vel_cont.angular.z = 0
                rospy.loginfo("UAV{0} Performed Orbit".format(self.uav_index))
                break
            #Set x and y velocity
            if (np.sign(req_vel) < 0):
                self.vel_cont.linear.x = abs(req_vel)*np.cos(yaw_angle)
                self.vel_cont.linear.y = abs(req_vel)*np.sin(yaw_angle)
            if (np.sign(req_vel) > 0):
                self.vel_cont.linear.x = abs(req_vel)*np.cos(yaw_angle)
                self.vel_cont.linear.y = abs(req_vel)*np.sin(yaw_angle)
            #Set yaw velocity
            self.vel_cont.angular.z = -2*abs(req_vel)/dist
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    def zamboni_search(self):
        #Global variables
        global altitude_reached
        rospy.loginfo("UAV{0} Performing Zamboni Pattern search".format(self.uav_index))
        #Wait until all the UAVs are at 5m
        while True:
            if np.all(altitude_reached): break
        #Align to start positions
        self.initial_align(0, y_points[self.uav_index],50)
        time.sleep(1)

        #Repeat 2 times
        for i in range(2):
            #After aligning
            self.reach_point(10, self.curr_pose.position.y, -100, 0, 100)
            self.perform_orbit(6.65, -80, np.pi, 100)
            self.reach_point(0, self.curr_pose.position.y, 100, np.pi, 100)
            self.perform_orbit(4.43, 80, 0, 100)
        #Last turn
        self.reach_point(10, self.curr_pose.position.y, -100, 0, 100)
        self.perform_orbit(6.65, -80, np.pi, 100)
        self.reach_point(0, self.curr_pose.position.y, 100, np.pi, 100)
        rospy.loginfo("Zamboni Pattern Search Completed")
        zam_search_completed = True

    #Callback function for current pose(local)
    def local_pose_callback(self, data):
        self.local_pose = data

    #Callback function for current Altitude
    def altitude_callback(self, data):
        self.altitude = data

if __name__ == "__main__":
    #Create the Multi UAV Controller Node node
    node_name = "Multi_UAV_Controller_Node"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Controller Node Set!")
    #Set up UAV objects
    uav0 = UAV(0)
    uav1 = UAV(1)
    uav2 = UAV(2)
    #Search
    grid_map = np.zeros([80,150], dtype=int)
    zam_search_completed = False
    altitude_reached = [False, False, False]
    start_reached = [False, False, False]
    y_points = [0,13.33,26.67]
    #Threads for altitude set
    uav0_alt_thread = Thread(target=uav0.hold_altitude, args=(5,50))
    uav0_alt_thread.daemon = True
    uav1_alt_thread = Thread(target=uav1.hold_altitude, args=(5,50))
    uav1_alt_thread.daemon = True
    uav2_alt_thread = Thread(target=uav2.hold_altitude, args=(5,50))
    uav2_alt_thread.daemon = True
    uav0_alt_thread.start()
    uav1_alt_thread.start()
    uav2_alt_thread.start()
    #Threads for orbit
    uav0_orbit_thread = Thread(target=uav0.zamboni_search, args=())
    uav0_orbit_thread.daemon = True
    uav0_orbit_thread.start()
    uav1_orbit_thread = Thread(target=uav1.zamboni_search, args=())
    uav1_orbit_thread.daemon = True
    uav1_orbit_thread.start()
    uav2_orbit_thread = Thread(target=uav2.zamboni_search, args=())
    uav2_orbit_thread.daemon = True
    uav2_orbit_thread.start()
    plt.matshow(grid_map)
    plt.show(block = False)
    while True:
        plt.clf()
        plt.matshow(grid_map, fignum=0)
        plt.show(block = False)
        plt.pause(0.001)
        if zam_search_completed: break
    #Spin
    print("Spinning")
    rospy.spin()

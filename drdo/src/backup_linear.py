#!/usr/bin/env python

#Guining Pertin
#DRDO SASE's UAV Fleet Challenge

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
# from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Altitude
from threading import Thread

#Perform linear search on 3 independent UAVs on unexplored map


#Create a UAV class to control each UAV
class UAV:
    #Initialize UAV
    def __init__(self, index):
        self.uav_index = index
        # #Subscribe to its global position topic
        # self.global_position = NavSatFix()
        # self.global_sub_name = "/uav{0}/mavros/global_position/global".format(index)
        # self.global_sub = rospy.Subscriber(self.global_sub_name,
        #                                    NavSatFix, self.global_position_callback)
        # rospy.loginfo("UAV{0} global_position subscribed".format(index))

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
        self.vel_cont.linear.z = 50.0
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
        # #Set current global position
        # self.curr_pos = self.global_position
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
        global altitude_reached
        global start_reached
        rospy.loginfo("Reaching altitude {0}m from {1}m".format(req_alt, self.curr_alt))
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        while True:
            #Update current altitude
            self.curr_alt = self.altitude.local
            #If linear search completed - break
            if lin_search_completed: break
            #First check it just started
            if self.curr_alt < 0.5:
                z_velocity = 40
            #Otherwise
            else:
                if abs(req_alt-self.curr_alt) < 0.1:
                    altitude_reached[self.uav_index] = 1
                #If current alt within 0.1m of required alt
                if abs(req_alt-self.curr_alt) < 0.01:
                    #Velocity control is hold control
                    z_velocity = 0
                    # rospy.loginfo("Altitude hold")
                #Else - check if up or down
                else:
                    #Proportional controller
                    #If current altitude is below - positive z vel
                    if req_alt > self.curr_alt:
                        z_velocity = 100*abs(req_alt-self.curr_alt)
                    #If current altitude is above - negative z vel
                    elif req_alt < self.curr_alt:
                        z_velocity = -100*abs(req_alt-self.curr_alt)
                    #Upper and lower limits
                    if z_velocity > 60: z_velocity = 60
                    elif z_velocity < -60: z_velocity = -60
            self.vel_cont.linear.z = z_velocity
            if np.sum(start_reached) == 3:
                grid_map[int(3*self.curr_pose.position.x+20), int(3*self.curr_pose.position.y+50)] = 1
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to reach x point
    def reach_point_x(self, req_x, timeout):
        global uav_reached_limits
        rospy.loginfo("Moving along x")
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        for i in range(timeout*loop_freq):
            #Update current pose
            self.curr_pose = self.local_pose.pose
            #If within required x
            if abs(req_x-self.curr_pose.position.x) < 0.1:
                self.vel_cont.linear.x = 0
                rospy.loginfo("Limit reached")
                uav_reached_limits[self.uav_index] = True
                break
            #Else
            else:
                if req_x > self.curr_pose.position.x:
                    self.vel_cont.linear.x = 100
                elif req_x < self.curr_pose.position.x:
                    self.vel_cont.linear.x = -100
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to reach y point
    def reach_point_y(self, req_y, timeout):
        global start_reached
        rospy.loginfo("Moving along y")
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        for i in range(timeout*loop_freq):
            #Update current pose
            self.curr_pose = self.local_pose.pose
            #If within required y
            if abs(req_y-self.curr_pose.position.y) < 0.1:
                self.vel_cont.linear.y = 0
                rospy.loginfo("Point reached")
                start_reached[self.uav_index] = 1
                break
            #Else
            else:
                if req_y > self.curr_pose.position.y:
                    self.vel_cont.linear.y = 100
                elif req_y < self.curr_pose.position.y:
                    self.vel_cont.linear.y = -100
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to perform linear search
    def lin_search(self):
        rospy.loginfo("UAV{0} Performing linear search".format(self.uav_index))
        #Wait until all the UAVs are at 5m
        while True:
            if np.sum(altitude_reached) == 3: break
        #Set difference in distances
        self.reach_point_y(uav_y_dists[self.uav_index],50)
        #Wait until all the UAVs are at starting y positions
        while True:
            if np.sum(start_reached) == 3: break
        #Start at lower limit
        self.reach_point_x(uav_x_low_limit[self.uav_index], 100)
        #Repeat 2 times
        for i in range(3):
            #Go to upper limit
            self.reach_point_x(uav_x_high_limit[self.uav_index], 100)
            #If reached limit, move along y for 1m
            self.reach_point_y(self.curr_pose.position.y+1,50)
            #Then go to lower limit
            self.reach_point_x(uav_x_low_limit[self.uav_index], 100)
            #If reached limit, move along y for 1m
            self.reach_point_y(self.curr_pose.position.y+1,50)
            #Repeat
        rospy.loginfo("Linear Search Completed")

    #Function to perform orbit
    def perform_orbit_up(self, dist, timeout):
        rospy.loginfo("UAV{0} Performing Orbit".format(self.uav_index))
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        init_y = self.local_pose.pose.position.y
        #Until uav doesn't reach height
        # while not uav_reached_limits[self.uav_index]:
        #     rospy.loginfo("Set x velocity")
        #     self.vel_cont.linear.x = 80
        self.vel_cont.linear.x = 100
        for i in range(timeout*loop_freq):
            self.curr_pose = self.local_pose.pose
            if abs(self.curr_pose.position.y - (init_y - dist)) < 0.1:
                self.vel_cont.linear.y = 0
                self.vel_cont.angular.z = 0
                rospy.loginfo("Performed Orbit")
                break
            else:
                self.vel_cont.linear.y = -100
                y = init_y - self.curr_pose.position.y
                self.vel_cont.linear.x = -100*(y*2/dist - 1)
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    #Function to perform orbit
    def perform_orbit_down(self, dist, timeout):
        rospy.loginfo("UAV{0} Performing Orbit".format(self.uav_index))
        loop_freq = 100
        rate = rospy.Rate(loop_freq)
        init_y = self.local_pose.pose.position.y
        #Until uav doesn't reach height
        # while not uav_reached_limits[self.uav_index]:
        #     rospy.loginfo("Set x velocity")
        #     self.vel_cont.linear.x = 80
        self.vel_cont.linear.x = -100
        for i in range(timeout*loop_freq):
            self.curr_pose = self.local_pose.pose
            if abs(self.curr_pose.position.y - (init_y + dist)) < 0.1:
                self.vel_cont.linear.y = 0
                rospy.loginfo("Performed Orbit")
                break
            else:
                self.vel_cont.linear.y = 100
                y = init_y - self.curr_pose.position.y
                self.vel_cont.linear.x = -100*(y*2/dist + 1)
            try:
                rate.sleep()
            except rospy.ROSException as error:
                rospy.loginfo(error)

    def zamboni_search(self):
        rospy.loginfo("UAV{0} Performing Zamboni search".format(self.uav_index))
        #Wait until all the UAVs are at 5m
        while True:
            if np.sum(altitude_reached) == 3: break
        #Set difference in distances
        self.reach_point_y(uav_y_dists[self.uav_index],50)
        #Wait until all the UAVs are at starting y positions
        while True:
            if np.sum(start_reached) == 3: break
        #Start at lower limit
        self.reach_point_x(uav_x_low_limit[self.uav_index], 100)
        #Repeat 2 times
        for i in range(2):
            #Go to upper limit along x
            self.reach_point_x(uav_x_high_limit[self.uav_index], 100)
            #If reached limit, move along y for 1m
            self.perform_orbit_up(6.65,100)
            #Then go to lower limit
            self.reach_point_x(uav_x_low_limit[self.uav_index], 100)
            #If reached limit, move along y for 1m
            self.perform_orbit_down(4.43,100)
            #Repeat
        #Go to upper limit along x
        self.reach_point_x(uav_x_high_limit[self.uav_index], 100)
        #If reached limit, move along y for 1m
        self.perform_orbit_up(6.65,100)
        #Then go to lower limit
        self.reach_point_x(uav_x_low_limit[self.uav_index], 100)
        rospy.loginfo("Zamboni Search Completed")
        lin_search_completed = 1

    # #Callback function for current GPS coordinates
    # def global_position_callback(self, data):
    #     self.global_position = data

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
    #Linear search parameters
    grid_map = np.zeros([80,150], dtype=int)
    uav_y_dists = [0, 13.33, 26.67]
    uav_x_low_limit = [0,0,0]
    uav_x_high_limit = [10,10,10]
    uav_reached_limits = [False,False,False]
    lin_search_completed = False
    altitude_reached = [0,0,0]
    start_reached = [0,0,0]
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
    # Threads for linear search
    # uav0_thread = Thread(target=uav0.lin_search, args=())
    # uav0_thread.daemon = True
    # uav1_thread = Thread(target=uav1.lin_search, args=())
    # uav1_thread.daemon = True
    # uav2_thread = Thread(target=uav2.lin_search, args=())
    # uav2_thread.daemon = True
    # uav0_thread.start()
    # uav1_thread.start()
    # uav2_thread.start()
    #Thread for orbit
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
    #Spin
    print("Spinning")
    rospy.spin()

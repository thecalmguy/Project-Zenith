#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Float64, Float64MultiArray
# from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread
import matplotlib.pyplot as plt
import numpy as np

lat_offset = 26.19#47.39
lon_offset = 91.69#8.54
gps_coordinates=[]
box_coordinates=[]
final_gps_coordinates = Float64MultiArray()

obj_found_1=obj_found_2=obj_found_3 = 0

counter1 = 0
counter2 = 0
counter3 = 0
append_freq = 2

heading1 = 0
heading2 = 0
heading3 = 0
height1 = 0
height2 = 0
height3 = 0
loc1 = (320,240)
loc2 = (320,240)
loc3 = (320,240)

f = open('gps_box_swarm.txt','w+')

#Callback functions for no of boxes
def obj_number_cb1(data):
    global obj_found_1
    obj_found_1=data.data
def obj_number_cb2(data):
    global obj_found_2
    obj_found_2=data.data
def obj_number_cb3(data):
    global obj_found_3
    obj_found_3=data.data

#Callback functions for GPS data and box append
def gps_cb1(data):
    global gps_coordinates
    global counter1
    counter1+=1
    #Reduce frequency by 1/append_freq
    if counter1>=append_freq :
        #For each obj_found
        for i in range(obj_found_1):
            rospy.loginfo("UAV1 found box")
            trans_data = pixel_to_global(loc1[0], loc1[1], (data.latitude, data.longitude), heading1, height1)
            gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
            f.write(str(gps_coordinates[-1])+"\n")
            counter1=0
def gps_cb2(data):
    global gps_coordinates
    global counter2
    counter2+=1
    if counter2>=append_freq :
        #For each obj_found
        for i in range(obj_found_2):
            rospy.loginfo("UAV2 found box")
            trans_data = pixel_to_global(loc2[0], loc2[1], (data.latitude, data.longitude), heading2, height2)
            gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
            f.write(str(gps_coordinates[-1])+"\n")
            counter2=0
def gps_cb3(data):
    global gps_coordinates
    global counter3
    counter3+=1
    if counter3>=append_freq :
        #For each obj_found
        for i in range(obj_found_3):
            rospy.loginfo("UAV3 found box")
            trans_data = pixel_to_global(loc3[0], loc3[1], (data.latitude, data.longitude), heading3, height3)
            gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
            f.write(str(gps_coordinates[-1])+"\n")
            counter3=0

#Callback functions for compass heading
def compass_cb1(data):
    global heading1
    heading1 = data.data
def compass_cb2(data):
    global heading2
    heading2 = data.data
def compass_cb3(data):
    global heading3
    heading3 = data.data

#Callback functions for altitude data
def alt_cb1(data):
    global height1
    height1 = data.data
def alt_cb2(data):
    global height2
    height2 = data.data
def alt_cb3(data):
    global height3
    height3 = data.data

#Function to convert from image to global frame
def pixel_to_global(x, y, gps_center, heading, height):
    x0 = 320
    y0 = 240
    pix_to_m = 1*height/640
    theta_pixel = np.arctan2(x0-x, 480-y-y0)
    dist_pixel  = np.sqrt((x-x0)**2 + (480-y-y0)**2)
    theta       = heading*np.pi/180 + theta_pixel
    dist_met    = dist_pixel * pix_to_m
    x_met       = dist_met * np.cos(theta + np.pi/2)
    y_met       = dist_met * np.sin(theta + np.pi/2)
    lat         = gps_center[0] + y_met/102470
    lon         = gps_center[1] + x_met/102470
    return (lat, lon)

#Function to cluster data
def cluster_data(bandwidth):
    global gps_coordinates
    global box_coordinates
    #Member coloring in plot works better with np array
    data = np.array(gps_coordinates)
    #Run only when gps_coordinates variable is not empty
    if len(gps_coordinates):
        #Perform mean shift clustering
        cluster_obj = MeanShift(bandwidth).fit(data)
        #Cluster labels
        cluster_labels = cluster_obj.labels_
        #Update box coordinates as the cluster centers
        box_coordinates = cluster_obj.cluster_centers_
        print(box_coordinates)
        #Number of clusters
        num_clusters = len(box_coordinates)
        #Plot
        colors = 'rgbcmyk'
        plt.clf()
        for k in range(num_clusters):
            #Get all the members in the cluster
            cluster_members = cluster_labels == k
            #Plot each member with different colors
            plt.plot(data[cluster_members, 0], data[cluster_members, 1],  '.', color = colors[k])
            plt.plot(box_coordinates[k][0], box_coordinates[k][1], 'o',
                     markerfacecolor=colors[k], markeredgecolor='k', markersize=14)
        plt.title("No. of clusters detected: {0}".format(num_clusters))
        plt.show(block = False)
        plt.pause(0.001)

#ROS Node
def cluster_node_func():
    global box_coordinates
    global final_gps_coordinates
    #Initialize
    rospy.init_node('gps_clustering_node', anonymous=True)
    #Subscribers
    #Get GPS Coordinates
    rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, gps_cb1)
    rospy.Subscriber("/uav2/mavros/global_position/global", NavSatFix, gps_cb2)
    rospy.Subscriber("/uav3/mavros/global_position/global", NavSatFix, gps_cb3)
    #Get number of objects
    rospy.Subscriber("/uav1/obj_found", Int8, obj_number_cb1)
    rospy.Subscriber("/uav2/obj_found", Int8, obj_number_cb2)
    rospy.Subscriber("/uav3/obj_found", Int8, obj_number_cb3)
    #Get compass heading
    rospy.Subscriber("/uav1/mavros/global_position/compass_hdg", Float64, compass_cb1)
    rospy.Subscriber("/uav2/mavros/global_position/compass_hdg", Float64, compass_cb2)
    rospy.Subscriber("/uav3/mavros/global_position/compass_hdg", Float64, compass_cb3)
    #Get relative altitude
    rospy.Subscriber("/uav1/mavros/global_position/rel_alt", Float64, alt_cb1)
    rospy.Subscriber("/uav1/mavros/global_position/rel_alt", Float64, alt_cb2)
    rospy.Subscriber("/uav1/mavros/global_position/rel_alt", Float64, alt_cb3)
    #Publisher
    pub = rospy.Publisher('/box_gps', Float64MultiArray, queue_size=10)
    #Set rate
    rate = rospy.Rate(1)
    plt.show(block = False)
    #Run clustering
    while not rospy.is_shutdown():
        cluster_data(200)
        final_gps_coordinates.data.clear()
        for box in box_coordinates:
            final_gps_coordinates.data.append((box[0]/10000000)+lat_offset)
            final_gps_coordinates.data.append((box[1]/10000000)+lon_offset)
        pub.publish(final_gps_coordinates)
        rate.sleep()
    f.close()

if __name__ == '__main__':
    try:
        cluster_node_func()
    except rospy.ROSInterruptException:
        pass

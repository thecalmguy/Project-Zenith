#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread
import matplotlib.pyplot as plt
import numpy as np

lat_offset = 47.39#26.192
lon_offset = 8.54#91.69
gps_coordinates=[]
box_coordinates=[]
box_gps_msg = NavSatFix()

obj_found_1=obj_found_2=obj_found_3 = 0

def obj_number_cb1(data):
    global obj_found_1
    obj_found_1=data.data

def obj_number_cb2(data):
    global obj_found_2
    obj_found_2=data.data

def obj_number_cb3(data):
    global obj_found_3
    obj_found_3=data.data

def gps_cb1(data):
    global gps_coordinates
    if obj_found_1 > 0:
        rospy.loginfo("UAV0 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb2(data):
    global gps_coordinates
    if obj_found_2 > 0:
        rospy.loginfo("UAV1 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb3(data):
    global gps_coordinates
    if obj_found_3 > 0:
        rospy.loginfo("UAV2 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def cluster_data(bandwidth):
    global gps_coordinates
    global box_coordinates
    #Member coloring in plot works better with np array
    data = np.array(gps_coordinates)
    # print(gps_coordinates)
    #Define bandwidth for the clustering
    #Run only when gps_coordinates variable is not empty
    if len(gps_coordinates):
        #Perform mean shift clustering
        cluster_obj = MeanShift(bandwidth).fit(data)
        #Cluster labels
        cluster_labels = cluster_obj.labels_
        #Update box coordinates as the cluster centers
        box_coordinates = cluster_obj.cluster_centers_
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

def cluster_node_func():
    global box_coordinates
    global box_gps_msg
    #Initialize
    rospy.init_node('gps_clustering_node', anonymous=True)
    #Subscribers
    rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, gps_cb1)
    rospy.Subscriber("/uav2/mavros/global_position/global", NavSatFix, gps_cb2)
    rospy.Subscriber("/uav3/mavros/global_position/global", NavSatFix, gps_cb3)
    rospy.Subscriber("/uav1/obj_found", Int8, obj_number_cb1)
    rospy.Subscriber("/uav2/obj_found", Int8, obj_number_cb2)
    rospy.Subscriber("/uav3/obj_found", Int8, obj_number_cb3)
    #Publisher
    pub = rospy.Publisher('/box_gps', NavSatFix, queue_size=10)
    #Set rate
    rate = rospy.Rate(1)
    plt.show(block = False)
    #Run clustering
    while not rospy.is_shutdown():
        cluster_data(500)
        for box in box_coordinates:
            box_gps_msg.latitude = (box[0]/10000000)+lat_offset
            box_gps_msg.longitude = (box[1]/1000000)+lon_offset
            pub.publish(box_gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        cluster_node_func()
    except rospy.ROSInterruptException:
        pass

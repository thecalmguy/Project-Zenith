#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread
import matplotlib.pyplot as plt

lat_offset = 26.192
lon_offset=91.69
gps_coordinates=[]
box_coordinates=[]

obj_found_1=obj_found_2=obj_found_3 = Int8()

def obj_number_cb1(data):
    global obj_found_1
    obj_found_1=data

def obj_number_cb2(data):
    global obj_found_2
    obj_found_2=data

def obj_number_cb3(data):
    global obj_found_3
    obj_found_3=data

def gps_cb1(data):
    if obj_found_1.data==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb2(data):
    global gps_coordinates
    if obj_found_2.data==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb3(data):
    global gps_coordinates
    if obj_found_3.data==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def cluster_data(bandwidth):
    global gps_coordinates
    global box_coordinates
    #Define bandwidth for the clustering
    #Run only when gps_coordinates variable is not empty
    if len(gps_coordinates):
        #Perform mean shift clustering
        cluster_obj = MeanShift(bandwidth).fit(gps_coordinates)
        #Cluster labels
        cluster_labels = cluster_obj.labels_
        #Update box coordinates as the cluster centers
        box_coordinates = cluster_obj.cluster_centers_
        #Number of clusters
        num_clusters = len(box_coordinates)
        # print("No. of clusters detected: {0}".format(num_clusters))
        #Plot
        colors = 'bgrcmyk'
        for k in range(num_clusters):
            #Get all the members in the cluster
            cluster_members = cluster_labels == k
            plt.clf()
            plt.plot(gps_coordinates[k][0], gps_coordinates[k][1], colors[k])
            # plt.plot(box_coordinates[k][0], box_coordinates[k][1], 'o',
            #          markerfacecolor=colors[k], markeredgecolor='k', markersize=14)
        plt.title('Number of clusters: {0}'.format(num_clusters))
        plt.show(block = False)
        plt.pause(0.001)

#rospy.spin()

def talker():
    rospy.init_node('gps_clustering_node', anonymous=True)
    rospy.Subscriber("/uav0/mavros/global_position/global", NavSatFix, gps_cb1)
    rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, gps_cb2)
    rospy.Subscriber("/uav2/mavros/global_position/global", NavSatFix, gps_cb3)
    rospy.Subscriber("/uav0/obj_found", Int8, obj_number_cb1)
    rospy.Subscriber("/uav1/obj_found", Int8, obj_number_cb2)
    rospy.Subscriber("/uav2/obj_found", Int8, obj_number_cb3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        plt.show(block = False)
        cluster_data(500)
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

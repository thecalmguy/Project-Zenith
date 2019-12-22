#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread
import matplotlib.pyplot as plt
import numpy as np

lat_offset = 26.19#47.39
lon_offset = 91.69#8.54
gps_coordinates=[]
box_coordinates=[]
box_gps_msg = NavSatFix()
final_gps_coordinates = Float64MultiArray()

obj_found_1=obj_found_2=obj_found_3 = 0

counter1 = 0
counter2 = 0
counter3 = 0
append_freq = 2

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
    global counter1
    global f
    counter1+=1
    if obj_found_1 > 0 and counter1>=append_freq :
        rospy.loginfo("UAV1 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*10000000)))
        counter1=0

def gps_cb2(data):
    global gps_coordinates
    global counter2
    global f
    counter2+=1
    if obj_found_2 > 0 and counter2>=append_freq:
        rospy.loginfo("UAV2 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*10000000)))
        counter2=0

def gps_cb3(data):
    global gps_coordinates
    global counter3
    global f
    counter3+=1
    if obj_found_3 > 0 and counter3>=append_freq:
        rospy.loginfo("UAV3 found box")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*10000000)))
        counter3=0


#Function to cluster data- Mean Shift
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

def cluster_node_func():
    global box_coordinates
    global final_gps_coordinates
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
    pub = rospy.Publisher('/box_gps', Float64MultiArray, queue_size=10)
    #Set rate
    rate = rospy.Rate(5)
    plt.show(block = False)
    #Run clustering
    while not rospy.is_shutdown():
        cluster_data(100)
        final_gps_coordinates.data.clear()
        for box in box_coordinates:
            final_gps_coordinates.data.append((box[0]/10000000)+lat_offset)
            final_gps_coordinates.data.append((box[1]/10000000)+lon_offset)
        pub.publish(final_gps_coordinates)
        rate.sleep()

if __name__ == '__main__':
    try:
        cluster_node_func()
    except rospy.ROSInterruptException:
        pass

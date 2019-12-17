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
box_gps_msg = NavSatFix()
center_gps_coordinates = Float64MultiArray()

obj_found_1=obj_found_2=obj_found_3 = 0

counter1 = 0
counter2 = 0
counter3 = 0
append_freq = 5

heading1 = 0
heading2 = 0
heading3 = 0
height1 = 0
height2 = 0
height3 = 0
loc1 = 0
loc2 = 0
loc3 = 0

f = open('gps_box_swarm.txt','w+')

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
        trans_data = pixel_global(loc1[1], loc1[0], (data.latitude, data.longitude), heading1, height1)
        gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
        f.write(str(gps_coordinates[-1])+"\n")
        counter1=0

def gps_cb2(data):
    global gps_coordinates
    global counter2
    global f
    counter2+=1
    if obj_found_2 > 0 and counter2>=append_freq:
        rospy.loginfo("UAV2 found box")
        trans_data = pixel_global(loc2[1], loc2[0], (data.latitude, data.longitude), heading2, height2)
        gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
        f.write(str(gps_coordinates[-1])+"\n")
        counter2=0

def gps_cb3(data):
    global gps_coordinates
    global counter3
    global f
    global heading3
    global height3
    global loc3
    counter3+=1
    if obj_found_3 > 0 and counter3>=append_freq:
        rospy.loginfo("UAV3 found box")
        trans_data = pixel_global(loc3[1], loc3[0], (data.latitude, data.longitude), heading3, height3)
        gps_coordinates.append((int((trans_data[0]-lat_offset)*10000000),int((trans_data[1]-lon_offset)*1000000)))
        f.write(str(gps_coordinates[-1])+"\n")
        counter3=0

def compass_cb1(data):
    global heading1
    heading1 = data.data

def compass_cb2(data):
    global heading2
    heading2 = data.data

def compass_cb3(data):
    global heading3
    heading3 = data.data

def alt_cb1(data):
    global height1
    height = data.data

def alt_cb2(data):
    global height2
    height = data.data

def alt_cb3(data):
    global height3
    height = data.data

def pixel_to_global(x, y, gps_center, heading, height):
    x0 = 640/2
    y0 = 480/2
    pix_to_m = 1*height/640
    theta_pixel = np.arctan2(480-y,x) - np.pi/2
    dist_pixel  = sqrt((x-x0)**2 + (480-y-y0)**2)
    theta       = heading*np.pi/180 + theta_pixel
    dist_met    = dist_pixel * pix_to_m
    x_met       = -dist_met * sin(theta + 90)
    y_met       = dist_met * cos(theta + 90)
    lat         = gps_center[0] + y_met/102470#times something
    lon         = gps_center[1] + x_met/102470#times something
    return (lat, lon)

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
    global center_gps_coordinates
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
    rospy.Subscriber("/uav1/mavros/global_position/compass_hdg", Float64, compass_cb1)
    rospy.Subscriber("/uav2/mavros/global_position/compass_hdg", Float64, compass_cb2)
    rospy.Subscriber("/uav3/mavros/global_position/compass_hdg", Float64, compass_cb3)
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
        cluster_data(500)
        center_gps_coordinates.data.clear()
        for box in box_coordinates:
            #box_gps_msg.latitude = (box[0]/10000000)+lat_offset
            #box_gps_msg.longitude = (box[1]/1000000)+lon_offset
            #pub.publish(box_gps_msg)
            center_gps_coordinates.data.append((box[0]/10000000)+lat_offset)
            center_gps_coordinates.data.append((box[1]/10000000)+lon_offset)

        pub.publish(center_gps_coordinates)
        rate.sleep()
    f.close()

if __name__ == '__main__':
    try:
        cluster_node_func()
    except rospy.ROSInterruptException:
        pass

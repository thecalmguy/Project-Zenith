#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread

lat_offset = 26.19, lon_offset=91.69
gps_coordinates=[]
box_coordinates=[]
box_gps_msg = NavSatFix()

obj_found_1=obj_found_2=obj_found_3 = Int8()

def obj_number_cb1(data):
    obj_found_1=data

def gps_cb1(data):
    if obj_found_1==1:
        print("*****************object found ******************")
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def cluster_data(bandwidth):
    #Define bandwidth for the clustering
    print("cluster code called")
    global gps_coordinates
    #Run only when gps_coordinates variable is not empty
    if len(gps_coordinates):
        #Perform mean shift clustering
        print("performing mean shift clustering")
        cluster_obj = MeanShift(bandwidth).fit(gps_coordinates)
        print("performed mean shift clustering &&&&&&&&&&&&&& ")
        #Print number of clusters for given bandwidth
        print("No. of clusters detected: {0}".format(len(cluster_obj.cluster_centers_)))
        #Update box coordinates
        box_coordinates = cluster_obj.cluster_centers_
        for box in box_coordinates:
            box_gps_msg.latitude = (box[0]/10000000)+lat_offset
            box_gps_msg.longitude = (box[1]/1000000)+lon_offset
            pub.publish(box_gps_msg)

#rospy.spin()

def cluster_node_func():
    rospy.init_node('gps_clustering_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb1)
    rospy.Subscriber("/obj_found", Int8, obj_number_cb1)
    pub = rospy.Publisher('/box_gps', NavSatFix, queue_size=10)
    while not rospy.is_shutdown():
        cluster_data(50)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        cluster_node_func()
    except rospy.ROSInterruptException:
        pass

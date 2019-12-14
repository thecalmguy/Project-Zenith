#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from sklearn.cluster import MeanShift
#from threading import Thread

lat_offset = 26.192, lon_offset=91.69
gps_coordinates=[]

obj_found_1=obj_found_2=obj_found_3 = Int8()

def obj_number_cb1(data):
    obj_found_1=data

def obj_number_cb2(data):
    obj_found_2=data

def obj_number_cb3(data):
    obj_found_3=data

def gps_cb1(data):
    if obj_found_1==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb2(data):
    if obj_found_2==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));

def gps_cb3(data):
    if obj_found_3==1:
        gps_coordinates.append((int((data.latitude-lat_offset)*10000000),int((data.longitude-lon_offset)*1000000)));


#rospy.spin()

def talker():
    # pub = rospy.Publisher('gps_clustering_node', String, queue_size=10)
    rospy.init_node('gps_clustering_node', anonymous=True)
    rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, gps_cb1)
    rospy.Subscriber("/uav2/mavros/global_position/global", NavSatFix, gps_cb2)
    rospy.Subscriber("/uav3/mavros/global_position/global", NavSatFix, gps_cb3)
    rospy.Subscriber("/uav1/obj_found", Int8, obj_number_cb1)
    rospy.Subscriber("/uav2/obj_found", Int8, obj_number_cb2)
    rospy.Subscriber("/uav3/obj_found", Int8, obj_number_cb3)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


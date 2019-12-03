import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


# callback method for state sub

lat = 0
lon = 0
alt = 0


def gps_cb(data):
    global lat
    global lon
    global alt
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude
    
    
while(True):    
    latitude = lat
    longitude = lon
    altitude = alt
    rospy.Subscriber("mavros/global_position/global", NavSatFix, gps_cb)
    rospy.loginfo(latitude)
    rospy.loginfo(longitude)
    rospy.loginfo(altitude)
    print(latitude)
    print(longitude)
    print(altitude)
    

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass

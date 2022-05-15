#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import pymap3d as pm

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point


origin = rospy.get_param('origin')
boat = [0, 0]

def gps_fix_callback(msg):
    boat[0], boat[1] = enu_convert([msg.latitude, msg.longitude, msg.altitude])

def enu_convert(gnss):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return e, n
    # TODO hopping말고 다른 곳에 쓰이는지 볼 것

def main():
    rospy.init_node('gnss_converter', anonymous=True)

    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher('enu_position', Point, queue_size=10)
    
    rate = rospy.Rate(10) #10Hz

    enu_position = Point()

    while not rospy.is_shutdown():
        enu_position.x = boat[0]
        enu_position.y = boat[1]
        # enu_position.z = boat[2]
        pub.publish(enu_position)

        rate.sleep()

    rospy.spin() # 넣어야 하나??

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

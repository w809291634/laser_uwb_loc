#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import tf
import os

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix

class Car_loc:
    def __init__(self):
        self.FRAMEID = "map"
        self.__locTime = 0
        
        self.tf_listener = tf.TransformListener()
        self.waitTf=True

        self.__gps_pos = None
        self.__subUWB = rospy.Subscriber("/gps/fix", NavSatFix, self.__onUWBCb) # 小车UWB消息定位函数   
    
    # 监听UWB实时位置
    def __onUWBCb(self, msg):
        if msg.position_covariance[0]!= msg.position_covariance[0]:
            self.__gps_pos =  None
            return
        if msg.position_covariance[0] < 20:
            y = msg.latitude
            x = msg.longitude
            self.__gps_pos = (time.time(), x, y)
        else:
            self.__gps_pos =  None

    # 基于UWB获取当前小车位置（x,y）
    def getGPSPos(self):
        return self.__gps_pos

    # 基于雷达获取当前小车位置（x,y,a）
    def getCarRadarPos(self):
        try:
            if self.waitTf:
                self.tf_listener.waitForTransform(self.FRAMEID, '/base_link', rospy.Time(), rospy.Duration(0.1))
                self.waitTf = False
        except:
            rospy.loginfo("tf error "+self.FRAMEID+"->base_link")
            return None
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.FRAMEID, '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            self.waitTf = True
            return None
        euler = tf.transformations.euler_from_quaternion(rot)
        return [trans[0],trans[1],euler[2]] 

if __name__ == '__main__':
    import rospy                                                            
    import signal
    
    def quit(signum, frame):
        print ''
        print 'stop app'
        sys.exit()

    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)

    rospy.init_node("car_laser_loc", log_level=rospy.INFO)
    car_loc=Car_loc()
    while True:
        RadarPos=car_loc.getCarRadarPos()
        if RadarPos:
            print(RadarPos)
        else:
            rospy.logerr("Getting RadarPos fail!")
        
        GPSPos=car_loc.getGPSPos()
        if GPSPos:
            print(GPSPos)
        else:
            rospy.logerr("Getting GPSPos fail!")
        time.sleep(1)


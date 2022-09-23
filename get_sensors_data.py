#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import tf
import os
import sys,signal
from std_msgs.msg import Int32MultiArray

this = sys.modules[__name__]
c_dir = os.path.split(os.path.realpath(__file__))[0]

class Sensors:
    def __init__(self):
        self.data=None
        self.vbat=None
        self.temp=None
        self.humi=None
        self.pressure=None
        self.light=None
        self.mp503=None
        self.mp2=None
        self.sonar1=None
        self.sonar2=None
        self.sonar3=None
        self.sonar4=None
        self.__subsensors = rospy.Subscriber("/xcar/sensors", Int32MultiArray, self.__sensors_handler) # 小车UWB消息定位函数   

    def __sensors_handler(self,msg):
        self.data=msg.data
        self.vbat=int(self.data[0])/10.0
        self.temp=int(self.data[1])/10.0
        self.humi=int(self.data[2])

        self.pressure=int(self.data[4])/1000.0
        self.light=int(self.data[5])
        self.mp503=int(self.data[6])
        self.mp2=int(self.data[7])

        self.sonar1=int(self.data[8])/100.0
        self.sonar2=int(self.data[9])/100.0
        self.sonar3=int(self.data[10])/100.0
        self.sonar4=int(self.data[11])/100.0

    def get(self):
        return self.data

if __name__ == '__main__':
    def quit(signum, frame):
        print('')
        print('stop app')
        sys.exit()
    signal.signal(signal.SIGINT, quit)  
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("get_sensors_data_demo", log_level=rospy.INFO)
    sensors=Sensors()
    while True:
        data=sensors.get()
        if data:
            print("++++++++++++++++++++++++++++++++++++++++++")
            print("vbat is %.1fv"%sensors.vbat)
            print("temp is %.1f℃"%sensors.temp)
            print("humi is %d%%"%sensors.humi)

            print("pressure is %.1fpa"%sensors.pressure)
            print("light is %dlux"%sensors.light)
            print("mp503 is %dg/mol"%sensors.mp503)
            print("mp2 is %dppm"%sensors.mp2)

            print("sonar1 is %.2f"%sensors.sonar1)
            print("sonar2 is %.2f"%sensors.sonar2)
            print("sonar3 is %.2f"%sensors.sonar3)
            print("sonar4 is %.2f"%sensors.sonar4)
            print("++++++++++++++++++++++++++++++++++++++++++")
        time.sleep(1)



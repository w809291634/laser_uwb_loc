#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from copy import deepcopy
import math
import threading
import numpy as np
import sys
import time
import tf
import os
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
CAR_W = 0.3 #车体宽
CAR_H = 0.4 #车体长
CHK_W = 0.02
CHK_H = 0.02
hanfAngle = math.asin(CAR_W/CAR_H)*180/math.pi # 36.8°
def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle 
    
class PluseRadar:
    
    def __init__(self):
        self.laserProj = LaserProjection()
        self.__scan = rospy.Subscriber('/scan_f', LaserScan, self.__scanCallback)
        self.baseLinkTrans = None
        #self.__miniDis = (float('inf'),0)
        self.__distance = None
        self.lastLimitTime = 0
        self.__pointcloud = None
    
    def waitRadar(self, timeout=10):
        if timeout == 0:
            while self.__pointcloud == None:
                time.sleep(1)
        else:
            while self.__pointcloud == None and timeout != 0:
                time.sleep(1)
                timeout -=1
        return self.__pointcloud != None
        
    def getMiniDistance(self, a1, a2):
        '''base_link 指定方向 -hanfAngle°到 + hanfAngle°的距离检测 '''
        a1 = normalize_angle(a1);
        a2 = normalize_angle(a2);
        minDis = (float('inf'), 0)
        a1 = int(a1 * ( 180 / math.pi)+0.5)
        a2 = int(a2 * ( 180 / math.pi)+0.5)
        if a1 > a2:
            a1,a2 = a2, a1
        #ang = (-hanfAngle+a, hanfAngle+a)
        for aa in range(a1, a2+1):
            angle = aa * math.pi / 180  
            rr = self.__getDistance( self.__distance, angle)
            if rr > CAR_H/2 and rr < minDis[0]:
                minDis = (rr, aa)
   
        return minDis
        
    def getDistance(self, ang):
        rr = self.__getDistance( self.__distance, ang)
        return rr
        
    def clacMiniDistance(self, dir=0):
        pc = pc2.read_points(self.__pointcloud, skip_nans=True, field_names=("x", "y", "z"))
        v = float('inf')
        if dir == 0 or dir == 1:
            if dir == 0: #正前方
                mind = -CAR_W/2 - self.baseLinkTrans[0][1] + CHK_W #y 轴最小  
                maxd = CAR_W/2  - self.baseLinkTrans[0][1] - CHK_W #y 轴最大
                cidx = 1         #检测坐标y
                vidx = 0         #检测值 x
                vmin = CAR_H/2 - self.baseLinkTrans[0][0] #最小值过滤
            elif dir == 1: #左侧
                mind = -CAR_H/2 - self.baseLinkTrans[0][0] + CHK_H
                maxd = CAR_H/2 - self.baseLinkTrans[0][0] - CHK_H
                cidx = 0         #检测坐标x
                vidx = 1         #检测值 y
                vmin = CAR_W/2 - self.baseLinkTrans[0][1] #最小值过滤

            for p in pc:
                if p[cidx] >= mind and p[cidx] <= maxd:
                    if p[vidx] < vmin-0.05:
                        continue
                    if p[vidx] < v:
                        v = p[vidx]
        elif dir==2 or dir == 3:
            v = float('-inf')
            if dir == 2: #正后方
                mind = -CAR_W/2 - self.baseLinkTrans[0][1] + CHK_W#y 轴最小  
                maxd = CAR_W/2  - self.baseLinkTrans[0][1] - CHK_W #y 轴最大
                cidx = 1         #检测坐标y
                vidx = 0         #检测值 x
                vmax = -CAR_H/2 - self.baseLinkTrans[0][0] #最小值过滤
            elif dir == 3: #右侧
                mind = -CAR_H/2 - self.baseLinkTrans[0][0] + CHK_H
                maxd = CAR_H/2 - self.baseLinkTrans[0][0] - CHK_H
                cidx = 0         #检测坐标x
                vidx = 1         #检测值 y
                vmax = -CAR_W/2 - self.baseLinkTrans[0][1] #最小值过滤
            for p in pc:
                if p[cidx] >= mind and p[cidx] <= maxd:
                    if p[vidx] > vmax + 0.05:
                        continue
                    if p[vidx] > v:
                        v = p[vidx]
        return v
 
    def getObstacleDistance(self, dir=0):
        dis = self.clacMiniDistance(dir)
        sub = (CAR_H/2, CAR_W/2, CAR_H/2,CAR_W/2)
        dis = abs(dis)  -  sub[dir]
        if dis < 0:
            dis = 0
        return dis
 
    def __getDistance(self, msg, ang):
        while ang >= math.pi*2:
            ang -= math.pi*2
        while ang < 0:
            ang += math.pi*2
        ii = int(ang / msg.angle_increment+0.5)%len(msg.ranges)
        st = ii - 5
        et = ii + 6
        dat = []
        if st >= 0 and st < et and et < len(msg.ranges):
            dat = [x for x in msg.ranges[st:et]]
        elif st < 0:
            dat += [x for x in msg.ranges[len(msg.ranges)+st:]]
            dat += [x for x in msg.ranges[0:et]]
        elif et >=  len(msg.ranges):
            dat += [x for x in msg.ranges[st:]]
            dat += [x for x in msg.ranges[0:len(msg.ranges)]]
        else:
            print 'eeeeeeeeeeeeeeeeeeeeeeee dat'
        
        dat.sort()
        return dat[len(dat)/2]

    def __scanCallback(self, msg):
        #cloud_out = self.laserProj.projectLaser(msg)
        #print cloud_out
        if self.baseLinkTrans == None:
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform('/base_link', msg.header.frame_id, rospy.Time(), rospy.Duration(1))

            (trans, rot) = tf_listener.lookupTransform('/base_link', msg.header.frame_id, rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            self.baseLinkTrans = (trans, euler)  # 雷达坐标到baselink转换
            #print self.baseLinkTrans
        #旋转雷达坐标到base link
        idx = int(self.baseLinkTrans[1][2]/msg.angle_increment)
        msg.ranges =  msg.ranges[-idx:]+msg.ranges[0:-idx]
        self.__distance = msg
        self.__pointcloud = self.laserProj.projectLaser(msg)
   
   
    def __del__(self):
        self.__scan.unregister()
if __name__ == '__main__':
    import rospy
    rospy.init_node("radar-demo", log_level=rospy.INFO)
    r = PluseRadar()
    
    r.waitRadar(30)
    print r.baseLinkTrans
    while  not rospy.is_shutdown():
        print '==================='
        print "%.2f" % r.getObstacleDistance(0)
        print "%.2f" % r.getObstacleDistance(1)
        print "%.2f" % r.getObstacleDistance(2)
        print "%.2f" % r.getObstacleDistance(3)
        print '-------'
        d,a = r.getMiniDistance(-math.pi/4, math.pi/4)
        d0 = r.getDistance((a-1)*math.pi/180)
        d1 = r.getDistance((a-0.5)*math.pi/180)
        d2 = r.getDistance((a)*math.pi/180)
        d3 = r.getDistance((a+0.5)*math.pi/180)
        d4 = r.getDistance((a+1)*math.pi/180)
        print a, d0,d1,d2,d3,d4
        time.sleep(0.2)
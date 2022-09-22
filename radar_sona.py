#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import tf
import os
import sys,signal
this = sys.modules[__name__]
c_dir = os.path.split(os.path.realpath(__file__))[0]

if __name__ == '__main__':
    sys.path.append(c_dir+'/car')
    import radar
    import sona
    def quit(signum, frame):
        print('')
        print('stop app')
        sys.exit()
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("radar_sona", log_level=rospy.INFO)

    # 应用程序
    this.pluse_radar = None
    this.pluse_sona = None

    # 检查雷达，并初始化
    this.pluse_radar = radar.PluseRadar()
    if not this.pluse_radar.waitRadar(30):
        print(u"雷达系统异常。")
        sys.exit(0)
    # 检查超声波，并初始化
    this.pluse_sona = sona.PluseSona() 
    while True:
        if this.pluse_radar != None:
            #　获取小车base_link方向(车头)-40度到40度范围的最小距离
            dis_radar, angle = this.pluse_radar.getMiniDistance(-0.7,  0.7)  
            print(dis_radar,angle)      #　最小距离和对应的角度
        if this.pluse_sona != None:
            dis_sona = this.pluse_sona.getDistance()
            print(dis_sona)
        time.sleep(1)

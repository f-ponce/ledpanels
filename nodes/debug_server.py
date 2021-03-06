#!/usr/bin/env python

from __future__ import division
import roslib;roslib.load_manifest('ledpanels')

import rospy
import serial
import numpy as N
import time

if __name__ == '__main__':
    from ledpanels.msg import MsgPanelsCommand,MsgDebug
    from ledpanels.srv import *


class PanelDebugServer():

    def __init__(self):

        rospy.init_node('panel_debug')
        
        self.controller_pub = rospy.Publisher('/ledpanels/command', 
                                    MsgPanelsCommand,
                                    queue_size = 10)
        self.debug_pub = rospy.Publisher('/ledpanels/debug', 
                                    MsgDebug,
                                    queue_size = 10)

        rospy.sleep(3)
        self.rate = rospy.Rate(10)
        """poll the panel_node with service requests for feedback from the led_panels and 
        publish that info"""
        self.controller_pub.publish(command = 'quiet_mode_off',arg1 = 0,arg2 = 0,arg3 = 0,arg4 = 0,arg5 = 0,arg6 = 0)
        self.readline = rospy.ServiceProxy('readline', SrvReadline)



if __name__ == '__main__':
    server = PanelDebugServer()
    while not rospy.is_shutdown():
        debug_line = server.readline().data
        if len(debug_line) > 0:
            server.debug_pub.publish(data = debug_line)
        server.rate.sleep()
    server.controller_pub.publish(command = 'quite_mode_on')

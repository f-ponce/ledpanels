#!/usr/bin/env python
import time
import random
import threading
import numpy as np
import os
import sys
import datetime
import roslib
import rospy
import roslib;roslib.load_manifest('ledpanels')
import functools
from ledpanels import display_ctrl
from ledpanels.msg import MsgPanelsCommand
from ledpanels.srv import *
from std_msgs.msg import Float64
from magnotether.msg import MsgAngleData

timestr = time.strftime("ledpanels_%Y%m%d_%H%M%S", time.localtime())
directory = '/home/francescavponce/catkin/src/ledpanels/data/'
filename = os.path.join(directory,'ledpanels_data_%s.csv'%timestr)
filename2 = os.path.join(directory,'ledpanels_data2_%s.csv'%timestr)
led_ids = open(filename,'w')
led_ids2 = open(filename2,'w')

class DisplayEvent(object):

    def __init__(self, start_time, duration, display_func):
        self.start_time = start_time
        self.duration = duration
        display_func()

    def is_done(self, t):
        return (t - self.start_time) >= self.duration


class RollingCircularMean(object):

    def __init__(self, size=800):
        self.size = size
        self.data = []

    def insert_data(self,item):
        self.data.append(np.deg2rad(item))
        if len(self.data) > self.size:
            self.data.pop(0)

    def value(self):
        if self.data:
            return np.rad2deg(circmean(self.data))
        else:
            return 0.0 


class ExpScript(object):

    def __init__(self):

        rospy.init_node('exp_script')

        pat_gains = [2,4,8,32]
        shuffle_pg = pat_gains
        np.random.shuffle(shuffle_pg)
        spg = shuffle_pg
        led_ids2.write('{0}\n'.format(spg))
        
        
        ### Experiment Protocol ###

        t1 = 59 #1 min dark
        pattern_id1 = functools.partial(self.panels_off)
        t2 = 1 #1 min panels ON
        #pattern_id2 = functools.partial(self.show_pattern, pattern_id=3, gain_x=0, bias_x = 0)
        pattern_id2 = functools.partial(self.panels_off)
        t3 = 180 #3 min panels ON and SUN
        #pattern_id3 = functools.partial(self.show_pattern, pattern_id=3, gain_x=0, bias_x = 0)
        pattern_id3 = functools.partial(self.panels_off)
        t4 = 180 #3 min panels moving and SUN
        pattern_id4 = functools.partial(self.mean_angle_to_pattern, gain_x=spg[0])
        t5 = 180 #3 min panels moving and SUN
        #pattern_id5 = functools.partial(self.show_pattern, pattern_id=3, gain_x=0, bias_x = 0)
        pattern_id5 = functools.partial(self.panels_off)
        t6 = 180 #3 min panels moving and SUN
        pattern_id6 = functools.partial(self.mean_angle_to_pattern, gain_x=spg[1])
        t7 = 180 #3 min panels moving and SUN
        #pattern_id7 = functools.partial(self.show_pattern, pattern_id=3, gain_x=0, bias_x = 0)
        pattern_id7 = functools.partial(self.panels_off)
        t8 = 180 #3 min panels moving and SUN
        pattern_id8 = functools.partial(self.mean_angle_to_pattern, gain_x=spg[2])
        t9 = 180 #3 min panels moving and SUN
        #pattern_id9 = functools.partial(self.show_pattern, pattern_id=3, gain_x=0, bias_x = 0)
        pattern_id9 = functools.partial(self.panels_off)
        t10 = 180 #3 min panels moving and SUN
        pattern_id10 = functools.partial(self.mean_angle_to_pattern, gain_x=spg[3])


        self.display_action_list = [
                {'type': 'fixed', 'duration': t1, 'func': pattern_id1},
                {'type': 'fixed', 'duration': t2, 'func': pattern_id2},
                {'type': 'fixed', 'duration': t3, 'func': pattern_id3},
                {'type': 'mean',  'duration': t4, 'func': pattern_id4},
                {'type': 'fixed', 'duration': t5, 'func': pattern_id5},
                {'type': 'mean',  'duration': t6, 'func': pattern_id6},
                {'type': 'fixed', 'duration': t7, 'func': pattern_id7},
                {'type': 'mean',  'duration': t8, 'func': pattern_id8},
                {'type': 'fixed', 'duration': t9, 'func': pattern_id9},
                {'type': 'mean',  'duration': t10,'func': pattern_id10},
                ]

        self.config_id = 1
        self.ctrl = display_ctrl.LedControler()
        self.ctrl.set_config_id(self.config_id)

        self.start_t = rospy.get_time()
        self.lock = threading.Lock()
        self.rolling_circ_mean = RollingCircularMean(size=1800)
        self.angles = rospy.Subscriber('/angle_data', MsgAngleData,self.on_angle_data_callback) 
        self.display_event = DisplayEvent(0.0, 0.0, self.do_nothing)

    def mean_angle_to_pattern(self, mean_angle, gain_x):


        a1 = 0;
        a2 = 22.5;
        a3 = 45;
        a4 = 67.5;
        a5 = 90;
        a6 = 112.5;
        a7 = 135;
        a8 = 157.5;
        a9 = 180;
        a10 = -22.5;
        a11 = -45;
        a12 = -67.5;
        a13 = -90;
        a14 = -112.5;
        a15 = -135;
        a16 = -157.5;
        a17 = -179.9999;

        ii = 4
        if a1 < mean_angle <= a2:
            plist = [4,7,5,9,3]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n") 
        elif a2 < mean_angle <= a3:
            plist = [10,4,7,5,9]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a3 < mean_angle <= a4:
            plist = [10,4,7,5,9]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a4 < mean_angle <= a5:
            plist = [6,10,4,7,5]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a5 < mean_angle <= a6:
            plist = [6,10,4,7,5]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a6 < mean_angle <= a7:
            plist = [8,3,10,4,7]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a7 < mean_angle <= a8:
            plist = [8,6,10,4,7]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a8 < mean_angle <= a9:
            plist = [3,8,6,10,4]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a10 < mean_angle <= a1:
            plist = [4,7,5,9,3]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a11 < mean_angle <= a10:
            plist = [7,5,9,3,8]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a12 < mean_angle <= a11:
            plist = [7,5,9,3,8]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a13 < mean_angle <= a12:
            plist = [5,9,3,8,6]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a14 < mean_angle <= a13:
            plist = [5,9,3,8,6]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a15 < mean_angle <= a14:
            plist = [9,3,8,6,10]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a16 < mean_angle <= a15:
            plist = [9,3,8,6,10]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")
        elif a17 < mean_angle <= a16:
            plist = [3,8,6,10,4]
            patlist = [x for x in plist]
            display_func = functools.partial(self.show_pattern, pattern_id=patlist[ii], gain_x=gain_x, bias_x = 0)
            led_ids.write('{0}\n'.format(patlist[ii]) + "\n")

        return display_func


    def on_angle_data_callback(self, data): 
        with self.lock:
            self.rolling_circ_mean.insert_data(data.angle)

    def elapsed_time(self):
        return rospy.get_time() - self.start_t

    def show_pattern(self, pattern_id, gain_x, bias_x):
        self.ctrl.stop()
        self.ctrl.set_pattern_id(pattern_id)      
        self.ctrl.send_gain_bias(gain_x = gain_x, bias_x = bias_x,gain_y = gain_x,bias_y =bias_x)
        self.ctrl.start()


    def panels_off(self):
        self.ctrl.stop()
        self.ctrl.all_off()

    def do_nothing(self):
        pass

    def run(self):
        done = False
        while (not rospy.is_shutdown()) and (not done):

            # Get elapsed time and rolling mean of angle data
            dt = self.elapsed_time()
            with self.lock:
                mean_angle = self.rolling_circ_mean.value()
                #rospy.logwarn(mean_angle)
            # Check if current display event is done if so selet next display event
            if self.display_event.is_done(dt):
                if self.display_action_list:
                    current_action = self.display_action_list.pop(0)
                    print(current_action['type'])

                    if current_action['type'] == 'fixed':
                        duration = current_action['duration']
                        display_func = current_action['func']

                    elif current_action['type'] == 'mean':
                        duration = current_action['duration']
                        display_func = current_action['func'](mean_angle)

                    else:
                        raise RuntimeError('unknow display action type {}'.format(current_action))
                    self.display_event = DisplayEvent(dt, duration, display_func)
                else:
                    done = True

        self.clean_up()


    def clean_up(self):
        self.ctrl.stop() 
        self.ctrl.all_off() 


# Utility functions
# -------------------------------------------------------------------------------------------------
def circmean(alpha,axis=None):
    mean_angle = np.arctan2(np.mean(np.sin(alpha),axis),np.mean(np.cos(alpha),axis))
    return mean_angle

def circvar(alpha,axis=None):
    R = np.sqrt(np.sum(np.sin(alpha),axis)**2 + np.sum(np.cos(alpha),axis)**2)/len(alpha)
    V = 1-R
    return R

def get_smallestSignedAngleBetween(ax, y):
    yy=np.deg2rad(np.ones(len(ax))*y)
    axx = np.deg2rad(ax)



# --------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    exp_scrpt = ExpScript()
    exp_scrpt.run()



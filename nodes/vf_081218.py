#!/usr/bin/env python
import time
import random
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



class Scheduler(object):

    def __init__(self,event_table=[]):
        self.event_table = event_table 
        self.pos = 0

    def update(self,t):
        if self.pos < len(self.event_table):
            next_event = self.event_table[self.pos]
            if (t >= next_event.t):
                next_event.func()
                self.pos += 1

    def done(self):
        if self.pos == len(self.event_table):
            return True
        else:
            return False

    def add(self, event):
        self.event_table.append(event)

    def reset(self):
        self.pos = 0

    def clear(self):
        self.event_table = []


class Event(object):

    def __init__(self,t,func):
        self.t = t
        self.func = func
        


class ExpScript(object):


    def __init__(self):
        print 'starting'
        # Initialize exp_script node
        rospy.init_node('exp_script')
        #rospy.on_shutdown(self.clean_up)
        self.pattern_indices = [1]
        self.closed_loop_gain = -1.0
        self.closed_loop_bias = 0.0
        self.open_loop_gain = 5.0
        self.open_loop_bias = 0.0
        self.open_loop_duration = 60.0
        self.config_id = 1
        self.open_or_closed_loop = 0
        #self.exp_duration = 1763 # run the experiment for exp_duration seconds


        # Load led controller
        self.ctrl = display_ctrl.LedControler()
        self.ctrl.set_config_id(self.config_id)

        patterns_ids = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
        #1 - rotating stripe lowest intensity
        #2 - rotating stripe
        #3-10 - highest intensity dithered r1-r8
        #11-18 - lowest intensity "dithered" r1-r8
       
#here I shuffle the patterns that I will use, except the stripe

        pids1 = [11,12,13,14,15,16,17,18] #low intensity patterns

        spids1 = pids1
        np.random.shuffle(spids1)
        rospy.logwarn(spids1)

        pids2 = [3,4,5,6,7,8,9,10] #high intensity patterns

        spids2 = pids2
        np.random.shuffle(spids2)
        rospy.logwarn(spids2)

        speeds = [8,64]

        ssp = speeds
        np.random.shuffle(ssp)
        rospy.logwarn(ssp)


        tsun = 300 #time sun only
        ts1 = 30 #time of stripe 
        tp = 60 #time of pattern of ventral flow at certain speed

        event_table = [

                Event(1.0,                  functools.partial(self.show_pattern, pattern_id=2, gain_x=30, bias_x = 0)),
                Event(1.0+ts1,              self.do_nothing),

                Event(2.0+ts1,              functools.partial(self.show_pattern, pattern_id=17, gain_x=0, bias_x = 0)),
                Event(2.0+ts1+tsun,         self.do_nothing),

                Event(3.0+ts1+tsun,         functools.partial(self.show_pattern, pattern_id=spids1[0], gain_x=ssp[0], bias_x = 0)),
                Event(3.0+ts1+tsun+tp,      self.do_nothing),

                Event(4.0+ts1+tsun+tp,      functools.partial(self.show_pattern, pattern_id=spids1[1], gain_x=ssp[1], bias_x = 0)),
                Event(4.0+ts1+tsun+tp*2,    self.do_nothing),

                Event(5.0+ts1+tsun+tp*2,    functools.partial(self.show_pattern, pattern_id=17, gain_x=0, bias_x = 0)),
                Event(5.0+ts1+tsun*2+tp*2,  self.do_nothing),

                Event(6.0+ts1+tsun*2+tp*2,  functools.partial(self.show_pattern, pattern_id=spids2[0], gain_x=ssp[1], bias_x = 0)),
                Event(6.0+ts1+tsun*2+tp*3,  self.do_nothing),

                Event(7.0+ts1+tsun*2+tp*3,  functools.partial(self.show_pattern, pattern_id=spids2[1], gain_x=ssp[0], bias_x = 0)),
                Event(7.0+ts1+tsun*2+tp*4,  self.do_nothing),

                ]
        
        self.scheduler = Scheduler(event_table)
        self.start_t = rospy.get_time()

    def clean_up(self):
        self.ctrl.stop() 
        self.ctrl.all_off() 


    def elapsed_time(self):
        return rospy.get_time() - self.start_t


    def show_pattern(self, pattern_id, gain_x, bias_x):
        self.ctrl.stop()
        self.ctrl.set_pattern_id(pattern_id)      
        self.ctrl.send_gain_bias(gain_x = gain_x, bias_x = bias_x,gain_y = gain_x,bias_y =bias_x)
        self.ctrl.start()
        rospy.logwarn(gain_x)
        rospy.logwarn(pattern_id)
        print(pattern_id)

    def do_nothing(self):
        pass

    def run(self):

        while not rospy.is_shutdown():
            dt = self.elapsed_time()
            self.scheduler.update(dt)
            if self.scheduler.done():
                break;

        self.clean_up()



if __name__ == '__main__':
    exp_scrpt = ExpScript()
    exp_scrpt.run()

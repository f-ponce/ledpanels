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
       
        patterns_idsi1 = [11,12,13,14,15,16,17,18]
        patterns_idsi2 = [3,4,5,6,7,8,9,10]

        spids1 = patterns_idsi1
        np.random.shuffle(spids1)
        rospy.logwarn(spids1)

        spids2 = patterns_idsi2
        np.random.shuffle(spids2)
        rospy.logwarn(spids2)

        speeds = [0,1,4,8,16,32,64,128]

        ssp = speeds
        np.random.shuffle(ssp)
        rospy.logwarn(ssp)

        #shuffle the gains for the stripe shown between patterns

        stripe_gains = [30,-30, 45, -45, 20, -20]
        shuffle_sg = stripe_gains
        np.random.shuffle(shuffle_sg)
        ssg = shuffle_sg
        rospy.logwarn(ssg)

        ts1 = 30
        ts = 10 #time of stripe between trials
        tp = 60 #time of pattern of ventral flow at certain speed

        event_table = [
                #For intensity 1
                
                # stripe 30 s
                Event(1.0,                    functools.partial(self.show_pattern, pattern_id=2, gain_x=30, bias_x = 0)),
                Event(1.0+ts1,                self.ctrl.stop),
                # exp pattern 1/dark here
                Event(2.0+ts1,                self.ctrl.all_off),
                Event(2.0+ts1+tp,             self.do_nothing),
                # stripe 1
                Event(3.0+ts1+tp,             functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[0], bias_x = 0)),
                Event(3.0+ts1+tp+ts,          self.do_nothing),
                # exp pattern 2
                Event(4.0+ts1+tp+ts,          functools.partial(self.show_pattern, pattern_id=spids1[0], gain_x=ssp[0], bias_x = 0)),
                Event(4.0+ts1+tp*2+ts,        self.do_nothing),
                # stripe 2
                Event(5.0+ts1+tp*2+ts,        functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[1], bias_x = 0)),
                Event(5.0+ts1+tp*2+ts*2,      self.do_nothing),
                # exp pattern 3
                Event(6.0+ts1+tp*2+ts*2,     functools.partial(self.show_pattern, pattern_id=spids1[1], gain_x=ssp[1], bias_x = 0)),
                Event(6.0+ts1+tp*3+ts*2,     self.do_nothing),
                # stripe 3
                Event(7.0+ts1+tp*3+ts*2,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[2], bias_x = 0)),
                Event(7.0+ts1+tp*3+ts*3,     self.do_nothing),
                # exp pattern 4
                Event(8.0+ts1+tp*3+ts*3,     functools.partial(self.show_pattern, pattern_id=spids1[2], gain_x=ssp[2], bias_x = 0)),
                Event(8.0+ts1+tp*4+ts*3,     self.do_nothing),
                # stripe 4
                Event(9.0+ts1+tp*4+ts*3,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[3], bias_x = 0)),
                Event(9.0+ts1+tp*4+ts*4,     self.do_nothing),
                # exp pattern 5
                Event(10.0+ts1+tp*4+ts*4,     functools.partial(self.show_pattern, pattern_id=spids1[3], gain_x=ssp[3], bias_x = 0)),
                Event(10.0+ts1+tp*5+ts*4,     self.do_nothing),
                # stripe 5
                Event(11.0+ts1+tp*5+ts*4,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[4], bias_x = 0)),
                Event(11.0+ts1+tp*5+ts*5,     self.do_nothing),
                # exp pattern 6
                Event(12.0+ts1+tp*5+ts*5,     functools.partial(self.show_pattern, pattern_id=spids1[4], gain_x=ssp[4], bias_x = 0)),
                Event(12.0+ts1+tp*6+ts*5,     self.do_nothing),
                # stripe 6
                Event(13.0+ts1+tp*6+ts*5,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[5], bias_x = 0)),
                Event(13.0+ts1+tp*6+ts*6,     self.do_nothing),
                # exp pattern 7
                Event(14.0+ts1+tp*6+ts*6,     functools.partial(self.show_pattern, pattern_id=spids1[5], gain_x=ssp[5], bias_x = 0)),
                Event(14.0+ts1+tp*7+ts*6,     self.do_nothing),
                # stripe 7
                Event(15.0+ts1+tp*7+ts*6,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[0], bias_x = 0)),
                Event(15.0+ts1+tp*7+ts*7,     self.do_nothing),
                # exp pattern 8
                Event(16.0+ts1+tp*7+ts*7,     functools.partial(self.show_pattern, pattern_id=spids1[6], gain_x=ssp[6], bias_x = 0)),
                Event(16.0+ts1+tp*8+ts*7,     self.do_nothing),
                # stripe 8
                Event(17.0+ts1+tp*8+ts*7,     functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[1], bias_x = 0)),
                Event(17.0+ts1+tp*8+ts*8,     self.do_nothing),
                # exp pattern 9
                Event(18.0+ts1+tp*8+ts*8,     functools.partial(self.show_pattern, pattern_id=spids1[7], gain_x=ssp[7], bias_x = 0)),
                Event(18.0+ts1+tp*9+ts*8,     self.do_nothing),

#                # stripe 9
                Event(19.0+ts1+tp*9+ts*8,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[0], bias_x = 0)),
                Event(19.0+ts1+tp*9+ts*9,     self.do_nothing),
                
                #intensity 2

                # exp pattern 10
                Event(20.0+ts1+tp*9+ts*9,     functools.partial(self.show_pattern, pattern_id=spids2[0], gain_x=ssp[0], bias_x = 0)),
                Event(20.0+ts1+tp*10+ts*9,    self.do_nothing),
                # stripe 10
                Event(21.0+ts1+tp*10+ts*9,    functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[1], bias_x = 0)),
                Event(21.0+ts1+tp*10+ts*10,   self.do_nothing),
                # exp pattern 11
                Event(22.0+ts1+tp*10+ts*10,   functools.partial(self.show_pattern, pattern_id=spids2[1], gain_x=ssp[1], bias_x = 0)),
                Event(22.0+ts1+tp*11+ts*10,   self.do_nothing),
                # stripe 11
                Event(23.0+ts1+tp*11+ts*10,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[2], bias_x = 0)),
                Event(23.0+ts1+tp*11+ts*11,   self.do_nothing),
                # exp pattern 12
                Event(24.0+ts1+tp*11+ts*11,   functools.partial(self.show_pattern, pattern_id=spids2[2], gain_x=ssp[2], bias_x = 0)),
                Event(24.0+ts1+tp*12+ts*11,   self.do_nothing),
                # stripe 12
                Event(25.0+ts1+tp*12+ts*11,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[3], bias_x = 0)),
                Event(25.0+ts1+tp*12+ts*12,   self.do_nothing),
                # exp pattern 13
                Event(26.0+ts1+tp*12+ts*12,   functools.partial(self.show_pattern, pattern_id=spids2[3], gain_x=ssp[3], bias_x = 0)),
                Event(26.0+ts1+tp*13+ts*12,   self.do_nothing),
                # stripe 13
                Event(27.0+ts1+tp*13+ts*12,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[4], bias_x = 0)),
                Event(27.0+ts1+tp*13+ts*13,   self.do_nothing),
                # exp pattern 14
                Event(28.0+ts1+tp*13+ts*13,   functools.partial(self.show_pattern, pattern_id=spids2[4], gain_x=ssp[4], bias_x = 0)),
                Event(28.0+ts1+tp*14+ts*13,   self.do_nothing),
                # stripe 14
                Event(29.0+ts1+tp*14+ts*13,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[5], bias_x = 0)),
                Event(29.0+ts1+tp*14+ts*14,   self.do_nothing),
                # exp pattern 15
                Event(30.0+ts1+tp*14+ts*14,   functools.partial(self.show_pattern, pattern_id=spids2[5], gain_x=ssp[5], bias_x = 0)),
                Event(30.0+ts1+tp*15+ts*14,   self.do_nothing),
                # stripe 15
                Event(31.0+ts1+tp*15+ts*14,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[0], bias_x = 0)),
                Event(31.0+ts1+tp*15+ts*15,   self.do_nothing),
                # exp pattern 16
                Event(32.0+ts1+tp*15+ts*15,   functools.partial(self.show_pattern, pattern_id=spids2[6], gain_x=ssp[6], bias_x = 0)),
                Event(32.0+ts1+tp*16+ts*15,   self.do_nothing),
                # stripe 16
                Event(33.0+ts1+tp*16+ts*15,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[1], bias_x = 0)),
                Event(33.0+ts1+tp*16+ts*16,   self.do_nothing),
                # exp pattern 17
                Event(34.0+ts1+tp*16+ts*16,   functools.partial(self.show_pattern, pattern_id=spids2[7], gain_x=ssp[7], bias_x = 0)),
                Event(34.0+ts1+tp*17+ts*16,   self.do_nothing),
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

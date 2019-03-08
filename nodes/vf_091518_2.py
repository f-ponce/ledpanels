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
       
        patterns_idsi1 = [11,12,13,14]

        spids1 = patterns_idsi1
        np.random.shuffle(spids1)
        rospy.logwarn(spids1)


        timestr = time.strftime("ledpanels_%Y%m%d_%H%M%S", time.localtime())
        directory = '/home/francescavponce/catkin/src/ledpanels/data/'
        filename = os.path.join(directory,'ledpanels_data_%s.csv'%timestr)
        led_ids = open(filename,'w')

        led_ids.write('{0}\n'.format(spids1))


        ts1 = 30
        #ts = 30 #time of stripe between trials
        tp = 60 #time of pattern of ventral flow at certain speed
        tsun = 300 #sun time


        event_table = [
                #For intensity 1
                
                # stripe 30 s
                Event(1.0,                     functools.partial(self.show_pattern, pattern_id=2, gain_x=30, bias_x = 0)),
                Event(1.0+ts1,                 self.ctrl.stop),
                # exp pattern 1
                Event(2.0+ts1,                 functools.partial(self.show_pattern, pattern_id=spids1[0], gain_x=32, bias_x = 0)),
                Event(2.0+ts1+tp,              self.do_nothing),
                # exp pattern 2
                Event(3.0+ts1+tp,              functools.partial(self.show_pattern, pattern_id=spids1[1], gain_x=32, bias_x = 0)),
                Event(3.0+ts1+tp*2,            self.do_nothing),
                # exp pattern 3
                Event(4.0+ts1+tp*2,            functools.partial(self.show_pattern, pattern_id=spids1[2], gain_x=32, bias_x = 0)),
                Event(4.0+ts1+tp*3,            self.do_nothing),
                # exp pattern 4
                Event(5.0+ts1+tp*3,            functools.partial(self.show_pattern, pattern_id=spids1[3], gain_x=32, bias_x = 0)),
                Event(5.0+ts1+tp*4,            self.do_nothing),
                # exp pattern 1
                Event(6.0+ts1+tp*4,            functools.partial(self.show_pattern, pattern_id=spids1[0], gain_x=32, bias_x = 0)),
                Event(6.0+ts1+tp*5,            self.do_nothing),
                # exp pattern 2
                Event(7.0+ts1+tp*5,            functools.partial(self.show_pattern, pattern_id=spids1[1], gain_x=32, bias_x = 0)),
                Event(7.0+ts1+tp*6,            self.do_nothing),
                # exp pattern 3
                Event(8.0+ts1+tp*6,            functools.partial(self.show_pattern, pattern_id=spids1[2], gain_x=32, bias_x = 0)),
                Event(8.0+ts1+tp*7,            self.do_nothing),
                # exp pattern 4
                Event(9.0+ts1+tp*7,            functools.partial(self.show_pattern, pattern_id=spids1[3], gain_x=32, bias_x = 0)),
                Event(9.0+ts1+tp*8,            self.do_nothing),
                # static image
                Event(10.0+ts1+tp*8,           functools.partial(self.show_pattern, pattern_id=spids1[3], gain_x=0, bias_x = 0)),
                Event(10.0+ts1+tp*8+tsun,      self.do_nothing),
                # exp pattern 1
                Event(11.0+ts1+tp*8+tsun,      functools.partial(self.show_pattern, pattern_id=spids1[0], gain_x=32, bias_x = 0)),
                Event(11.0+ts1+tp*9+tsun,      self.do_nothing),
                # exp pattern 2
                Event(12.0+ts1+tp*9+tsun,      functools.partial(self.show_pattern, pattern_id=spids1[1], gain_x=32, bias_x = 0)),
                Event(12.0+ts1+tp*10+tsun,     self.do_nothing),
                # exp pattern 3
                Event(13.0+ts1+tp*10+tsun,     functools.partial(self.show_pattern, pattern_id=spids1[2], gain_x=32, bias_x = 0)),
                Event(13.0+ts1+tp*11+tsun,     self.do_nothing),
                # exp pattern 4
                Event(14.0+ts1+tp*11+tsun,     functools.partial(self.show_pattern, pattern_id=spids1[3], gain_x=32, bias_x = 0)),
                Event(14.0+ts1+tp*12+tsun,     self.do_nothing),

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

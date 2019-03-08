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

        pids = [3,4,5,6,7,8,9,10]

        spids = pids
        np.random.shuffle(spids)
        rospy.logwarn(spids)

        speeds = [64]

#shuffle the gains for the stripe shown between patterns

        stripe_gains = [30,-30, 64, -64, 20, -20]
        shuffle_sg = stripe_gains
        np.random.shuffle(shuffle_sg)
        rospy.logwarn(shuffle_sg)
        ssg = shuffle_sg


        ts1 = 30
        ts = 10 #time of stripe between trials
        tp = 60 #time of pattern of ventral flow at certain speed

        event_table = [
                Event(1.0,    self.ctrl.all_off  ),
                Event(2.0,    self.ctrl.all_off ),
                Event(3.0,    self.ctrl.all_off  ),
                Event(4.0,    self.ctrl.all_off ),
                #For intensity 1
                
                # stripe 30 s
                Event(5.0,                    functools.partial(self.show_pattern, pattern_id=2, gain_x=30, bias_x = 0)),
                Event(5.0+ts1,                self.do_nothing),
                # exp pattern 1
                Event(6.0+ts1,                functools.partial(self.show_pattern, pattern_id=spids[0], gain_x=speeds[0], bias_x = 0)),
                Event(6.0+ts1+tp,             self.do_nothing),
                # stripe 1
                Event(7.0+ts1+tp,             functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[0], bias_x = 0)),
                Event(7.0+ts1+tp+ts,          self.do_nothing),
                # exp pattern 2
                Event(8.0+ts1+tp+ts,          functools.partial(self.show_pattern, pattern_id=spids[1], gain_x=speeds[0], bias_x = 0)),
                Event(8.0+ts1+tp*2+ts,        self.do_nothing),
                # stripe 2
                Event(9.0+ts1+tp*2+ts,        functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[1], bias_x = 0)),
                Event(9.0+ts1+tp*2+ts*2,      self.do_nothing),
                # exp pattern 3
                Event(10.0+ts1+tp*2+ts*2,     functools.partial(self.show_pattern, pattern_id=spids[2], gain_x=speeds[0], bias_x = 0)),
                Event(10.0+ts1+tp*3+ts*2,     self.do_nothing),
                # stripe 3
                Event(11.0+ts1+tp*3+ts*2,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[2], bias_x = 0)),
                Event(11.0+ts1+tp*3+ts*3,     self.do_nothing),
                # exp pattern 4
                Event(12.0+ts1+tp*3+ts*3,     functools.partial(self.show_pattern, pattern_id=spids[3], gain_x=speeds[0], bias_x = 0)),
                Event(12.0+ts1+tp*4+ts*3,     self.do_nothing),
                # stripe 4
                Event(13.0+ts1+tp*4+ts*3,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[3], bias_x = 0)),
                Event(13.0+ts1+tp*4+ts*4,     self.do_nothing),
                # exp pattern 5
                Event(14.0+ts1+tp*4+ts*4,     functools.partial(self.show_pattern, pattern_id=spids[4], gain_x=speeds[0], bias_x = 0)),
                Event(14.0+ts1+tp*5+ts*4,     self.do_nothing),
                # stripe 5
                Event(15.0+ts1+tp*5+ts*4,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[4], bias_x = 0)),
                Event(15.0+ts1+tp*5+ts*5,     self.do_nothing),
                # exp pattern 6
                Event(16.0+ts1+tp*5+ts*5,     functools.partial(self.show_pattern, pattern_id=spids[5], gain_x=speeds[0], bias_x = 0)),
                Event(16.0+ts1+tp*6+ts*5,     self.do_nothing),
                # stripe 6
                Event(17.0+ts1+tp*6+ts*5,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[5], bias_x = 0)),
                Event(17.0+ts1+tp*6+ts*6,     self.do_nothing),
                # exp pattern 7
                Event(18.0+ts1+tp*6+ts*6,     functools.partial(self.show_pattern, pattern_id=spids[6], gain_x=speeds[0], bias_x = 0)),
                Event(18.0+ts1+tp*7+ts*6,     self.do_nothing),
                # stripe 7
                Event(19.0+ts1+tp*7+ts*6,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[0], bias_x = 0)),
                Event(19.0+ts1+tp*7+ts*7,     self.do_nothing),
                # exp pattern 8
                Event(20.0+ts1+tp*7+ts*7,     functools.partial(self.show_pattern, pattern_id=spids[7], gain_x=speeds[0], bias_x = 0)),
                Event(20.0+ts1+tp*8+ts*7,     self.do_nothing),
#                # stripe 8
#                Event(21.0+ts1+tp*8+ts*7,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[1], bias_x = 0)),
#                Event(21.0+ts1+tp*8+ts*8,     self.do_nothing),
#                # exp pattern 9
#                Event(22.0+ts1+tp*8+ts*8,     functools.partial(self.show_pattern, pattern_id=sps1[8][0], gain_x=sps1[8][1], bias_x = 0)),
#                Event(22.0+ts1+tp*9+ts*8,     self.do_nothing),
##                # stripe 9
#                Event(23.0+ts1+tp*9+ts*8,     functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[2], bias_x = 0)),
#                Event(23.0+ts1+tp*9+ts*9,     self.do_nothing),
#                # exp pattern 10
#                Event(24.0+ts1+tp*9+ts*9,     functools.partial(self.show_pattern, pattern_id=sps1[9][0], gain_x=sps1[9][1], bias_x = 0)),
#                Event(24.0+ts1+tp*10+ts*9,    self.do_nothing),
#                # stripe 10
#                Event(25.0+ts1+tp*10+ts*9,    functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[3], bias_x = 0)),
#                Event(25.0+ts1+tp*10+ts*10,   self.do_nothing),
#                # exp pattern 11
#                Event(26.0+ts1+tp*10+ts*10,   functools.partial(self.show_pattern, pattern_id=sps1[10][0], gain_x=sps1[10][1], bias_x = 0)),
#                Event(26.0+ts1+tp*11+ts*10,   self.do_nothing),
#                # stripe 11
#                Event(27.0+ts1+tp*11+ts*10,   functools.partial(self.show_pattern, pattern_id=2, gain_x=ssg[4], bias_x = 0)),
#                Event(27.0+ts1+tp*11+ts*11,   self.do_nothing),
#                # exp pattern 12
#                Event(28.0+ts1+tp*11+ts*11,   functools.partial(self.show_pattern, pattern_id=sps1[11][0], gain_x=sps1[11][1], bias_x = 0)),
#                Event(28.0+ts1+tp*12+ts*11,   self.do_nothing),

#                # stripe 12
#                Event(29.0+ts1+tp*12+ts*11,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[0], bias_x = 0)),
#                Event(29.0+ts1+tp*12+ts*12,   self.do_nothing),

#                # exp pattern 13
#                Event(30.0+ts1+tp*12+ts*12,   functools.partial(self.show_pattern, pattern_id=sps2[0][0], gain_x=sps2[0][1], bias_x = 0)),
#                Event(30.0+ts1+tp*13+ts*12,   self.do_nothing),
#                # stripe 13
#                Event(31.0+ts1+tp*13+ts*12,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[1], bias_x = 0)),
#                Event(31.0+ts1+tp*13+ts*13,   self.do_nothing),
#                # exp pattern 14
#                Event(32.0+ts1+tp*13+ts*13,   functools.partial(self.show_pattern, pattern_id=sps2[1][0], gain_x=sps2[1][1], bias_x = 0)),
#                Event(32.0+ts1+tp*14+ts*13,   self.do_nothing),
#                # stripe 14
#                Event(33.0+ts1+tp*14+ts*13,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[2], bias_x = 0)),
#                Event(33.0+ts1+tp*14+ts*14,   self.do_nothing),
#                # exp pattern 15
#                Event(34.0+ts1+tp*14+ts*14,   functools.partial(self.show_pattern, pattern_id=sps2[2][0], gain_x=sps2[2][1], bias_x = 0)),
#                Event(34.0+ts1+tp*15+ts*14,   self.do_nothing),
#                # stripe 15
#                Event(35.0+ts1+tp*15+ts*14,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[3], bias_x = 0)),
#                Event(35.0+ts1+tp*15+ts*15,   self.do_nothing),
#                # exp pattern 16
#                Event(36.0+ts1+tp*15+ts*15,   functools.partial(self.show_pattern, pattern_id=sps2[3][0], gain_x=sps2[3][1], bias_x = 0)),
#                Event(36.0+ts1+tp*16+ts*15,   self.do_nothing),
#                # stripe 16
#                Event(37.0+ts1+tp*16+ts*15,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[4], bias_x = 0)),
#                Event(37.0+ts1+tp*16+ts*16,   self.do_nothing),
#                # exp pattern 17
#                Event(38.0+ts1+tp*16+ts*16,   functools.partial(self.show_pattern, pattern_id=sps2[4][0], gain_x=sps2[4][1], bias_x = 0)),
#                Event(38.0+ts1+tp*17+ts*16,   self.do_nothing),
#                # stripe 17
#                Event(39.0+ts1+tp*17+ts*16,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[5], bias_x = 0)),
#                Event(39.0+ts1+tp*17+ts*17,   self.do_nothing),
#                # exp pattern 18
#                Event(40.0+ts1+tp*17+ts*17,   functools.partial(self.show_pattern, pattern_id=sps2[5][0], gain_x=sps2[5][1], bias_x = 0)),
#                Event(40.0+ts1+tp*18+ts*17,   self.do_nothing),
#                # stripe 18
#                Event(41.0+ts1+tp*18+ts*17,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[0], bias_x = 0)),
#                Event(41.0+ts1+tp*18+ts*18,   self.do_nothing),
#                # exp pattern 19
#                Event(42.0+ts1+tp*18+ts*18,   functools.partial(self.show_pattern, pattern_id=sps2[6][0], gain_x=sps2[6][1], bias_x = 0)),
#                Event(42.0+ts1+tp*19+ts*18,   self.do_nothing),
#                # stripe 19
#                Event(43.0+ts1+tp*19+ts*18,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[1], bias_x = 0)),
#                Event(43.0+ts1+tp*19+ts*19,   self.do_nothing),
#                # exp pattern 20
#                Event(44.0+ts1+tp*19+ts*19,   functools.partial(self.show_pattern, pattern_id=sps2[7][0], gain_x=sps2[7][1], bias_x = 0)),
#                Event(44.0+ts1+tp*20+ts*19,   self.do_nothing),
#                # stripe 20
#                Event(45.0+ts1+tp*20+ts*19,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[2], bias_x = 0)),
#                Event(45.0+ts1+tp*20+ts*20,   self.do_nothing),
#                # exp pattern 21
#                Event(46.0+ts1+tp*20+ts*20,   functools.partial(self.show_pattern, pattern_id=sps2[8][0], gain_x=sps2[8][1], bias_x = 0)),
#                Event(46.0+ts1+tp*21+ts*20,   self.do_nothing),
#                # stripe 21
#                Event(47.0+ts1+tp*21+ts*20,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[3], bias_x = 0)),
#                Event(47.0+ts1+tp*21+ts*21,   self.do_nothing),
#                # exp pattern 22
#                Event(48.0+ts1+tp*21+ts*21,   functools.partial(self.show_pattern, pattern_id=sps2[9][0], gain_x=sps2[9][1], bias_x = 0)),
#                Event(48.0+ts1+tp*22+ts*21,   self.do_nothing),
#                # stripe 22
#                Event(49.0+ts1+tp*22+ts*21,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[4], bias_x = 0)),
#                Event(49.0+ts1+tp*22+ts*22,   self.do_nothing),
#                # exp pattern 23
#                Event(50.0+ts1+tp*22+ts*22,   functools.partial(self.show_pattern, pattern_id=sps2[10][0], gain_x=sps2[10][1], bias_x = 0)),
#                Event(50.0+ts1+tp*23+ts*22,   self.do_nothing),
#                # stripe 23
#                Event(51.0+ts1+tp*23+ts*22,   functools.partial(self.show_pattern, pattern_id=1, gain_x=ssg[5], bias_x = 0)),
#                Event(51.0+ts1+tp*23+ts*23,   self.do_nothing),
#                # exp pattern 24
#                Event(52.0+ts1+tp*23+ts*23,   functools.partial(self.show_pattern, pattern_id=sps2[11][0], gain_x=sps2[11][1], bias_x = 0)),
#                Event(52.0+ts1+tp*24+ts*23,   self.do_nothing),
##                # stripe 24
#                Event(53.0+ts1+tp*24+ts*23,   functools.partial(self.show_pattern, pattern_id=1, gain_x=30, bias_x = 0)),
#                Event(53.0+ts1+tp*24+ts*24,   self.do_nothing),
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

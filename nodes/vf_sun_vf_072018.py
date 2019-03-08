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
        self.exp_duration = 1274 # run the experiment for exp_duration seconds


        # Load led controller
        self.ctrl = display_ctrl.LedControler()
        self.ctrl.set_config_id(self.config_id)

        patterns_ids = [1,2,3,4,5,6,7,8]
        #shuffle_pids = patterns_ids
        #np.random.shuffle(shuffle_pids)

        event_table = [
                Event(1.0,    self.ctrl.all_on  ),
                Event(2.0,    self.ctrl.all_off ),
                Event(3.0,    self.ctrl.all_on  ),
                Event(4.0,    self.ctrl.all_off ),
                Event(5.0,    functools.partial(self.show_pattern, pattern_id=4, gain_x=20)),
                Event(125.0,  self.do_nothing),
                Event(126.0,  functools.partial(self.show_pattern, pattern_id=5, gain_x=20)),
                Event(246.0,  self.do_nothing),
                Event(247.0,  functools.partial(self.show_pattern, pattern_id=3, gain_x=20)),
                Event(367.0,  self.do_nothing),
                Event(368.0,  functools.partial(self.show_pattern, pattern_id=6, gain_x=20)),
                Event(488.0,  self.do_nothing),
                Event(489.0,  self.ctrl.stop ),
                Event(490.0,  self.ctrl.all_off ),
                Event(791.0,  functools.partial(self.show_pattern, pattern_id=4, gain_x=20)),
                Event(911.0,  self.do_nothing),
                Event(912.0,  functools.partial(self.show_pattern, pattern_id=5, gain_x=20)),
                Event(1032.0, self.do_nothing),
                Event(1033.0, functools.partial(self.show_pattern, pattern_id=3, gain_x=20)),
                Event(1153.0, self.do_nothing),
                Event(1154.0, functools.partial(self.show_pattern, pattern_id=6, gain_x=20)),
                Event(1274.0, self.do_nothing),
                ]

#on 07/10/18 changed gain from 30

        self.scheduler = Scheduler(event_table)
        self.start_t = rospy.get_time()

    def clean_up(self):
        self.ctrl.stop() 
        self.ctrl.all_off() 


    def elapsed_time(self):
        return rospy.get_time() - self.start_t


    def show_pattern(self, pattern_id, gain_x):
        self.ctrl.set_pattern_id(pattern_id)
        self.ctrl.stop()
        self.ctrl.send_gain_bias(gain_x = gain_x, bias_x = 0,gain_y = 20,bias_y =0)
#on 07/10/18 changed this line self.ctrl.send_gain_bias(gain_x = gain_x, bias_x = 0,gain_y = 1,bias_y =0)
        self.ctrl.start()
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



        #self.open_loop_duration = 60.0
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.stop()
        #self.ctrl.send_gain_bias(gain_x = 30, bias_x = 0,gain_y = 1,bias_y =0)
        #self.ctrl.start()
        #rospy.sleep(self.open_loop_duration)

        #self.pattern_id = 2
        #self.open_loop_duration = 30.0
        #self.ctrl.stop()
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.send_gain_bias(gain_x = 30, bias_x = 0,gain_y = 1,bias_y =0)
        #self.ctrl.start()
        #rospy.sleep(self.open_loop_duration)

        #self.pattern_id = 3
        #self.open_loop_duration = 30.0
        #self.ctrl.stop()
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.send_gain_bias(gain_x = 30, bias_x = 0,gain_y = 1,bias_y =0)
        #self.ctrl.start()
        #rospy.sleep(self.open_loop_duration)

        #self.pattern_id = 4
        #self.open_loop_duration = 30.0
        #self.ctrl.stop()
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.send_gain_bias(gain_x = 30, bias_x = 0,gain_y = 1,bias_y =0)
        #self.ctrl.start()
        #rospy.sleep(self.open_loop_duration)

        #self.pattern_id = 5
        #self.open_loop_duration = 30.0
        #self.ctrl.stop()
        #self.ctrl.set_pattern_id(self.pattern_id)
        #self.ctrl.send_gain_bias(gain_x = 30, bias_x = 0,gain_y = 1,bias_y =0)
        #self.ctrl.start()
        #rospy.sleep(self.open_loop_duration)
        #self.ctrl.stop()

        #self.ctrl.all_off()

        #print('done')



if __name__ == '__main__':

    exp_scrpt = ExpScript()
    exp_scrpt.run()





## Old stuff
# ----------------------------------------------------------------------------------------------------------------------
#    def open_loop_mode(self):
#        self.ctrl.set_position(0,0) #randomize x position, use 1 position y channel
#        time.sleep(0.05)
#        self.ctrl.set_mode('xrate=funcx','yrate=funcy')
#        time.sleep(0.05)
#        self.ctrl.send_gain_bias(gain_x = self.open_loop_gain, bias_x = self.open_loop_bias,gain_y = 0,bias_y = 0)
#        time.sleep(0.05)
#        self.ctrl.start()
#        time.sleep(self.open_loop_duration)
#        self.ctrl.stop()
#        time.sleep(0.05)

#    def experimental_loop(self):


#        # Publish current pattern and the panel controller mode
#        self.pattern_pub.publish(self.pattern_id)
#        #self.panel_mode_pub.publish(self.open_or_closed_loop)

#        # Load the selected pattern
#        self.set_pattern()

#        # Set to closed-loop mode
#        print 'Open loop mode'
#        #open_loop_mode(open_loop_duration,open_loop_gain,open_loop_bias)
#        self.open_loop_mode(100,1,0)

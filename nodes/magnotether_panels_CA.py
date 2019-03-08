import time
import os
import sys
import datetime
import roslib
import rospy
from ledpanels import display_ctrl
from ledpanels.msg import MsgPanelsCommand
from ledpanels.srv import *

class exp_script():
    def __init__(self):

        print 'starting'
        # Initialize exp_script node
        rospy.init_node('exp_script')
        self.pattern_indices = [1]
        self.closed_loop_gain = -1.0
        self.closed_loop_bias = 0.0
        self.open_loop_gain = 5.0
        self.open_loop_bias = 0.0
        self.open_loop_duration = 60.0
        self.pattern_id = 1
        self.open_or_closed_loop = 0

        self.exp_duration = 1800 # run the experiment for exp_duration seconds

        print 'doing stuff with the panels'
        # Load led controller
        self.ctrl = display_ctrl.LedControler()
        time.sleep(1.0)
        self.ctrl.all_on()
        time.sleep(1.0)
        self.ctrl.all_off()
        time.sleep(1.0)

        # Pattern publisher
        #self.pattern_pub = rospy.Publisher('ledpanels/command, MsgPanelsCommand, queue_size = 10)

        # Panel controller mode publisher
        #self.panel_mode_pub = rospy.Publisher('ledpanels/command', queue_size = 10)
        print('patterns')

        self.ctrl.set_pattern_id(self.pattern_id)
        self.ctrl.send_gain_bias(gain_x = 3000, bias_x = 0,gain_y = 0,bias_y =0)
        self.ctrl.start()
        time.sleep(self.open_loop_duration)
        self.ctrl.stop()
        print('stop')
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


if __name__ == '__main__':
    expscrpt = exp_script()
    rospy.spin()

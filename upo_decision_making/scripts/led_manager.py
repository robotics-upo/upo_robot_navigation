#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import rospkg
from teresa_driver.srv import Teresa_leds
from hri_common import SpencerLocalizationInterface
from upo_decision_making.msg import IDArray, HRIControlMode, HRIFeedbackStateInfo
#from hri_feedback_msgs.msg import HRIControlMode, HRIFeedbackStateInfo
from functools import partial
import pdb
import threading

SELECTED_IT = 1
OTHER_IT = 2
OBSTACLE = 3
MODE = 4 # mode is either manual or automatic
STATE = 5 # state represents a state machine state

LEN_LEDS = 60

AUTO = 0
MANUAL = 1



class LedManager(object):
    @staticmethod
    def to_led_index(theta):
        ix = int(theta*30/np.pi) + 15
        
        return ix

    def __init__(self):
        self._priorities = [MODE, STATE, SELECTED_IT,
                            OTHER_IT,
                            OBSTACLE]
        rate = rospy.get_param("~rate", 20)
        self.rate = rospy.Rate(rate)
        rpkg = rospkg.RosPack()
        base_path = rpkg.get_path("decision_making_uva")
        yaml_path = base_path + "/cfg/leds.yaml"
        with open(yaml_path, "r") as yamlf:
            self.led_params = yaml.load(yamlf)
        print self.led_params
        max_kernel_width = rospy.get_param("~max_kernel_width", 9)
        if (max_kernel_width % 2) != 1:
            rospy.logwarn("Warning: max kernel width for led manager is not an odd number")
            rospy.logwarn("Adjusting max kernel width to %d" % max_kernel_width + 1)
            max_kernel_width = int(max_kernel_width) + 1
        self._min_d = rospy.get_param("~min_scaling_dist", 0.5)
        self._max_d = rospy.get_param("~max_scaling_dist", 2.5)
        self._w = (max_kernel_width - 1)/2
        self.base_link_frame = rospy.get_param("hri_common/base_link_frame", "base_link")
        self.spencer_iface = SpencerLocalizationInterface()
        self.it_id_sub = rospy.Subscriber('hri_action_server_test/test_interaction_target_ids',
                                           IDArray, self.it_id_cb, queue_size=1)
        self.state = None
        self.state_sub = rospy.Subscriber('/hri_state_info', HRIFeedbackStateInfo, self.state_cb)
        self._pub_control_mode = rospy.Subscriber('/hri_control_mode', HRIControlMode, self.mode_cb)
        self.it_ids = set([])

        self._lock = threading.Lock()
        color = self.led_params["wait_color"]
        self.draw_functions = [self.draw_persons,partial(self.draw_state,color=color)]
        self.mode = MANUAL
        self.state = "Wait"
        self.fade_animation_counter = 29
        self.ledclient = rospy.ServiceProxy("/teresa_leds", Teresa_leds, persistent=True)
        connected_to_lasers = False
        while not connected_to_lasers and not rospy.is_shutdown():
            try:
                self.ledclient.wait_for_service(timeout=1.0)
                connected_to_lasers = True
            except rospy.ROSException as e:
                rospy.logwarn(e)
        if not connected_to_lasers:
            return
        rospy.loginfo("Connected to /teresa_leds service")
        self.timer = rospy.Timer(rospy.Duration(1./rate), self.timer_cb)
        self.stopped = False
        rospy.on_shutdown(self.turn_off)

    def it_id_cb(self, msg):
        self.it_ids = set(msg.ids)

    def state_cb(self,msg):
        self.state = msg.state_name
        if self.mode==AUTO:
            self.get_draw_functions()

    def get_draw_functions(self):
        if self.state == "normal_interaction_policy":
            color= self.led_params["interaction_mode_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state,color=color)]
        elif self.state == "close_interaction_policy":
            color= self.led_params["interaction_mode_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state,color=color)]
        elif self.state == "look_for_interaction_targets":
            color = self.led_params["seek_interaction_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state,color=color)]
        elif self.state == "Yield":
            color = self.led_params["yield_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]
        elif self.state == "WalkSideBySide":
            color = self.led_params["wsbs_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]
        elif self.state == "NavigateWaypoint":
            color = self.led_params["nav_plan"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]
        elif self.state == "NavigateInteractionTarget":
            color = self.led_params["navigate_it_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]
        elif self.state == "Wait":
            color = self.led_params["wait_color"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]
        elif self.state == "NavigateHome":
            color= self.led_params["nav_home"]
            self.draw_functions = [self.draw_persons, partial(self.draw_state, color=color)]

    def mode_cb(self,msg):
        if msg.mode == HRIControlMode.MANUAL_MODE:
            self.mode = MANUAL
            color = self.led_params["manual"]
            self.draw_functions = [partial(self.draw_circle,color=color)]
            self.fade_animation_counter = 0
        elif msg.mode == HRIControlMode.AUTONOMOUS_MODE:
            self.fade_animation_counter = np.floor(LEN_LEDS/2-1)
            self.mode = AUTO
            self.get_draw_functions()

    def draw_persons(self):
        it_leds = np.zeros((LEN_LEDS, 3))
        other_leds = np.zeros((LEN_LEDS, 3))
        it_color = np.array(self.led_params['selected_it'])
        other_color = np.array(self.led_params['other_it'])
        locs = self.spencer_iface.get_all_locs(self.base_link_frame)
        for it_id, l in locs.items():
            color = it_color if it_id in self.it_ids else other_color
            leds = it_leds if it_id in self.it_ids else other_leds
            x = l.footprint.transform.translation.x
            y = l.footprint.transform.translation.y
            theta = np.arctan2(y, x)
            ix = self.to_led_index(theta)
            d = np.hypot(x, y)
            if d > self._min_d:
                if d > self._max_d:
                    w = 0
                else:
                    w = int(np.round(((self._max_d-d)/self._max_d)*self._w))
            else:
                w = self._w

            leds = self.draw_line(w, color, ix=ix, decay=7, leds=leds)

        return {SELECTED_IT: it_leds, OTHER_IT: other_leds}
        
    def draw_state(self, color):
        width = self.led_params["state_width"]
        index = self.led_params["state_index"]
        out = self.draw_line(width, color, ix=index)
        return {STATE: out}

    def draw_circle(self,color=[255, 255, 255]):
        out = np.ones((LEN_LEDS, 3)) * np.array(color)
        return {MODE:out}

    def draw_line(self,w,c,ix=15,decay=0,leds = None):
        # This can also be used to draw a dot!
        leds = np.zeros((LEN_LEDS, 3)) if leds is None else leds
        # w: the width on either side of the center index
        # c: the color of the line
        # ix: the center index
        # decay: the color decay for every led away from the center
        for ixj in np.arange(ix - w, ix + w + 1):
            ixj = ixj % LEN_LEDS
            s = np.abs(ixj - ix)
            value = c if s == 0 or decay==0 else c /(s*decay)
            leds[ixj] = value if np.sum(value) > np.sum(leds[ixj, :]) else leds[ixj]
        return leds


    def timer_cb(self, ev):
        if not self.stopped:
            # For now, just for conversation mode
            dicts = []
            for i in self.draw_functions:
                dicts.append(i())
            if self.fade_animation_counter>1:
                color = np.array(self.led_params['manual'])
                dicts.append({MODE:self.draw_line(self.fade_animation_counter,color,ix=45)})
                self.fade_animation_counter-=1
            merged_dict = merge_dicts(dicts)
            out_array = self.array_from_dict(merged_dict)
            self.publish(out_array)

    def publish(self, led_values):
        #ledvalues = np.ones(180, dtype='uint8')
        self.ledclient.call(rgb_values=led_values.tolist())

    def array_from_dict(self, dict):
        first = True
        out = np.zeros((LEN_LEDS,3))
        for i in self._priorities:
            if dict.has_key(i) and first:
                out = dict[i]
                first = False
            elif dict.has_key(i):
                non_zero_out = np.array([np.sum(out, axis=1) == 0]).T * dict[i]
                out += non_zero_out
        return out.flatten()
    
    def turn_off(self):
        self.stopped = True
        led = np.zeros(3*LEN_LEDS).tolist()
        self.ledclient.call(rgb_values=led)

def merge_dicts(dicts):
    out = {}
    for dic in dicts:
        for key in dic.keys():
            out[key] = dic[key]
    return out
if __name__ == '__main__':
    rospy.init_node('led_manager')
    lm = LedManager()

    rospy.loginfo('boilerplate node running')

    rospy.spin()

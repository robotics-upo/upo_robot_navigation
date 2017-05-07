#!/usr/bin/env python
import rospy as rp
import numpy as np
import yaml
import rospkg
import tf
import threading

#from hri_common import (SpencerLocalizationInterface,
#                        TFLocalizationInterface,
#                        PersonLocalizationException)
from upo_decision_making.msg import ControlEvent, IDArray, HRIFeedbackFromInterface
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal
#from hri_feedback_msgs.msg import HRIFeedbackFromInterface
from std_msgs.msg import UInt16

class ControlEventBase(object):
    def __init__(self, event_type):
        self._pub = rp.Publisher("behavior_manager/control_events", ControlEvent, queue_size=1)
        self._etype = event_type

    def trigger(self):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = self._etype
        self._pub.publish(ce)


class LowBatteryEvent(ControlEventBase):
    """
    Triggered whenever the battery level goes below a certain voltage
    """
    def __init__(self):
        super(LowBatteryEvent, self).__init__(ControlEvent.LOW_BATT)
        self._sub = rp.Subscriber("ups_battery_voltage", Float32, self.callback, queue_size=1)
        self._thr = rp.get_param("event_params/low_battery_event/threshold", 12.0)

    def callback(self, msg):
        if msg.data < self._thr:
            self.trigger()


#class ManualControlEvent(ControlEventBase):
#    """
#    Triggered whenever the user sends a velocity command to the robot
#    """
#    def __init__(self):
#        super(ManualControlEvent, self).__init__(ControlEvent.MANUAL_CONTROLLING)
#        self._sub = rp.Subscriber("cmd_vel_itf", Twist, self.callback, queue_size=1)
#
#    def callback(self, msg):
#        self.trigger()



#class PersonLocalizationEventManager():
#    def __init__(self):
#        # TODO: For now we only use one interaction target, regardless of 
#        # how many are selected
#        self._lock = threading.Lock()
#        self._lock.acquire()
#        self._base_link_frame = rp.get_param("hri_common/base_link_frame", "base_link")
#        self._cev_pub = rp.Publisher('behavior_manager/control_events', ControlEvent, queue_size=1)
#        self._it_ids_sub = rp.Subscriber('behavior_manager/interaction_target_ids',
#                                         IDArray, self._it_ids_cb, queue_size=1)
#        self._last_it_pos = None
#        self._it_ids = []
#        self._last_it_ids = []
#        self._has_it_track = False
#        unsafe_bound = rp.get_param(
#            'event_params/person_localization_events/proxemic_ranges/unsafe_upper_bound', 0.55) #0.43
#        personal_bound = rp.get_param(
#            'event_params/person_localization_events/proxemic_ranges/personal_upper_bound', 0.935)
#        social_bound = rp.get_param(
#            'event_params/person_localization_events/proxemic_ranges/social_upper_bound', 1.42)
#        public_bound = rp.get_param(
#            'event_params/person_localization_events/proxemic_ranges/public_upper_bound', 3.0)
#
#        self._prox_ranges = np.array([unsafe_bound, personal_bound, social_bound, public_bound])
#        self._ev_types = [ControlEvent.INTERACTION_TARGET_AT_UNSAFE_RANGE,
#                          ControlEvent.INTERACTION_TARGET_AT_PERSONAL_RANGE,
#                          ControlEvent.INTERACTION_TARGET_AT_SOCIAL_RANGE,
#                          ControlEvent.INTERACTION_TARGET_AT_PUBLIC_RANGE,
#                          ControlEvent.INTERACTION_TARGET_OUT_OF_RANGE]
#
#        self._use_spencer = rp.get_param('hri_common/use_spencer', True)
#        if self._use_spencer:
#            self._ploc_iface = SpencerLocalizationInterface()
#        else:
#            self._ploc_iface = TFLocalizationInterface()
#
#        r = rp.get_param('event_params/person_localization_events/poll_rate', 10.0)
#        self._rate = rp.Rate(int(r))
#        self._exec_thread = threading.Thread(target=self.execute)
#        self._lock.release()
#        self._exec_thread.start()
#
#    def _it_ids_cb(self, msg):
#        self._lock.acquire()
#        self._it_ids = msg.ids
#        self._lock.release()
#
#    def execute(self):
#        while not rp.is_shutdown():
#            self._lock.acquire()
#            if len(self._it_ids) > 0:
#                try:
#                    it_id = self._it_ids[0]
#                    plocs = self._ploc_iface.get_all_locs(self._base_link_frame)
#                    it_loc = plocs[it_id]
#                    np_com_pos = self._ploc_iface.com_as_np([it_loc])
#                    self.evaluate_events([it_id], np_com_pos)
#                    self._last_it_pos = np_com_pos
#                    self._last_it_ids = [it_id]
#                    self._has_it_track = True
#                except Exception:
#                    if self._last_it_pos is not None:
#                        cev = ControlEvent()
#                        cev.stamp = rp.Time.now()
#                        cev.type = ControlEvent.LOST_TRACK_OF_INTERACTION_TARGET
#                        self._cev_pub.publish(cev)
#                    self._last_it_pos = None
#                    self._has_it_track = False
#            self._lock.release()
#            self._rate.sleep()
#
#    def evaluate_events(self, it_ids, com_loc):
#        cev = ControlEvent()
#        cev.stamp = rp.Time.now()
#
#        if it_ids != self._last_it_ids:
#            cev.type = ControlEvent.NEW_INTERACTION_TARGET
#            self._cev_pub.publish(cev)
#            return
#
#        if not self._has_it_track:
#            cev.type = ControlEvent.REGAINED_TRACK_OF_INTERACTION_TARGET
#            self._cev_pub.publish(cev)
#            return
#
#        idx = np.searchsorted(self._prox_ranges, np.linalg.norm(com_loc))
#        last_idx = np.searchsorted(self._prox_ranges, np.linalg.norm(self._last_it_pos))
#        if idx != last_idx:
#            cev.type = self._ev_types[idx]
#            self._cev_pub.publish(cev)
#            return


if __name__ == '__main__':
    rp.init_node('event_manager')

    p = rospkg.RosPack().get_path('upo_decision_making')

    direct_events = [LowBatteryEvent()]

    #personloc_evmanager = PersonLocalizationEventManager()

    rp.loginfo('Event manager initialized.')
    rp.spin()

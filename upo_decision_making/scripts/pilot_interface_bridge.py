#!/usr/bin/env python
import rospy as rp
import rospkg
import yaml
import math

from upo_decision_making.msg import ControlEvent, IDArray, HRIFeedbackFromInterface
from geometry_msgs.msg import PoseStamped
#from hri_feedback_msgs.msg import HRIFeedbackFromInterface
from functools import partial


class PilotInterfaceBridge(object):
    def __init__(self):
        waypoint_defs_path = rp.get_param('/waypoint_defs')
        rospack = rospkg.RosPack()
        base_path = rospack.get_path("teresa_common")
        if base_path[-1] != "/":
            base_path += "/"
        with open(base_path + waypoint_defs_path, 'r') as f:
            self.wayp_dict = yaml.load(f)

        self._sub = rp.Subscriber("teresa_pilot_feedback",
                                  HRIFeedbackFromInterface,
                                  self.callback,
                                  queue_size=1)
        self._ce_pub = rp.Publisher("behavior_manager/control_events", ControlEvent, queue_size=1)
        self._goal_pub = rp.Publisher("behavior_manager/nav_goal",
                                      PoseStamped,
                                      queue_size=1,
                                      latch=True)
        self._it_id_pub = rp.Publisher("behavior_manager/interaction_target_ids",
                                       IDArray,
                                       queue_size=1,
                                       latch=True)
        self._cb_map = {HRIFeedbackFromInterface.NAV_TO_PERSON_REQUESTED:
                            partial(self.id_ev_req, ControlEvent.NAV_TO_PERSON_REQUESTED),
                        HRIFeedbackFromInterface.WALK_WITH_PERSON_REQUESTED:
                            partial(self.id_ev_req, ControlEvent.WALK_WITH_PERSON_REQUESTED),
                        HRIFeedbackFromInterface.NAV_TO_WAYPOINT_REQUESTED: self.nav_to_waypoint_req,
                        HRIFeedbackFromInterface.CALL_OVER:
                            partial(self.generic_ev_req, ControlEvent.CALL_OVER),
                        HRIFeedbackFromInterface.HEARING_PROBLEMS_INDICATED:
                            partial(self.generic_ev_req, ControlEvent.HEARING_PROBLEMS_INDICATED),
                        HRIFeedbackFromInterface.MANUAL_CONTROL_REQUESTED:
                            partial(self.generic_ev_req, ControlEvent.MANUAL_CONTROLLING)}

    def pub_id(self, msg):
        it_id = int(msg.data)
        it_id_list = IDArray([it_id])
        self._it_id_pub.publish(it_id_list)

    def pub_event(self, ev_type):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ev_type
        self._ce_pub.publish(ce)

    def id_ev_req(self, ev_type, msg):
        self.pub_id(msg)
        self.pub_event(ev_type)

    def nav_to_waypoint_req(self, msg):
        try:
            w = self.wayp_dict[msg.data]
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.header.stamp = rp.Time.now()
            g.pose.position.x = w['x']
            g.pose.position.y = w['y']
            g.pose.orientation.z = math.sin(w['theta']/2.0)
            g.pose.orientation.w = math.cos(w['theta']/2.0)
            self._goal_pub.publish(g)

            ce = ControlEvent()
            ce.stamp = rp.Time.now()
            ce.type = ControlEvent.NAV_TO_WAYPOINT_REQUESTED
            self._ce_pub.publish(ce)
        except KeyError as e:
            rp.logwarn(e)

    def generic_ev_req(self, ev_type, msg):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ev_type
        self._ce_pub.publish(ce)

    def callback(self, msg):
        if msg.type in self._cb_map:
            self._cb_map[msg.type](msg)

if __name__ == '__main__':
    rp.init_node('pilot_interface_bridge')
    rp.loginfo('Pilot Interface Bridge initialized.')
    
    bridge = PilotInterfaceBridge()
    rp.spin()
    

#!/usr/bin/python2
import rospy as rp
import rospkg
import yaml
import math

from upo_msgs.msg import PersonPoseUPO
from upo_msgs.msg import PersonPoseArrayUPO
#from hri_feedback_msgs.msg import HRIFeedbackFromInterface
from upo_decision_making.msg import ControlEvent, IDArray, HRIFeedbackFromInterface

class EventPublisher(object):
    def __init__(self):
        waypoint_defs_path = rp.get_param('/waypoint_defs')
        rospack = rospkg.RosPack()
        base_path = rospack.get_path("upo_launchers") #"teresa_common"
        if base_path[-1] != "/":
            base_path += "/"
        with open(base_path + waypoint_defs_path, 'r') as f:
            self.wayp_dict = yaml.load(f)

        self.gif_pub = rp.Publisher("teresa_pilot_feedback", HRIFeedbackFromInterface, queue_size=1)
        self.cev_pub = rp.Publisher("behavior_manager/control_events", ControlEvent, queue_size=1)
        self.people_sub = rp.Subscriber("people/navigation",
                                      PersonPoseArrayUPO,
                                      self.people_cb,
                                      queue_size=1)
        self._people = []

    def people_cb(self, msg):
        self._people = msg.personPoses


    def publish_interactiontarget_req(self):
        per_selected = False

        while not per_selected:
            print 'Select person Id:'
            req_id = int(raw_input('> '))
	
            req_idx = -1

            for a in range(len(self._people)):
                if self._people[a].id == req_id:
                    req_idx = a
                    per_selected = True

            if per_selected == False:
                print 'There is not such a person Id.'

        gif = HRIFeedbackFromInterface()
        gif.header.stamp = rp.Time.now()
        gif.type = HRIFeedbackFromInterface.NAV_TO_PERSON_REQUESTED

        print "Sending request to IT id '"+str(req_id) + "'"
        gif.data = str(req_id)
        self.gif_pub.publish(gif)


    def publish_walkside_req(self):
        per_selected = False
        while not per_selected:
            print 'Select person Id:'
            req_id = int(raw_input('> '))
	
            req_idx = -1

            for a in range(len(self._people)):
                if self._people[a].id == req_id:
                    req_idx = a
                    per_selected = True

            if per_selected == False:
                print 'There is not such a person Id.'
	
        gif = HRIFeedbackFromInterface()
        gif.header.stamp = rp.Time.now()
        gif.type = HRIFeedbackFromInterface.WALK_WITH_PERSON_REQUESTED

	print "Sending request to walk with IT id '%s'" % req_id

	gif.data = str(req_id)
        self.gif_pub.publish(gif)


    def publish_waypoint_req(self):
        wayp_selected = False
        while not wayp_selected:
            try:
                print 'Select waypoint index (starts at 0):'
                a = int(raw_input('> '))
                if a in range(len(self.wayp_dict)):
                    wayp_selected = True
            except ValueError:
                rp.logwarn("Not a valid input.")
            finally:
                if not wayp_selected:
                    print 'Incorrect waypoint index.'
                    print 'There are only %d registered waypoints.' % len(self.wayp_dict)
                    print 'Check cfg/waypoint_defs.yaml.'

        k = self.wayp_dict.keys()[a]
        print "Sending request to waypoint '%s'" % k
        gif = HRIFeedbackFromInterface()
        gif.header.stamp = rp.Time.now()
        gif.type = HRIFeedbackFromInterface.NAV_TO_WAYPOINT_REQUESTED

	gif.data = k
        self.gif_pub.publish(gif)

    def publish_low_batt(self):
        cev = ControlEvent()
        cev.stamp = rp.Time.now()
        cev.type = ControlEvent.LOW_BATT
        self.cev_pub.publish(cev)

    def publish_call_over(self):
        gif = HRIFeedbackFromInterface()
        gif.header.stamp = rp.Time.now()
        gif.type = HRIFeedbackFromInterface.CALL_OVER
        self.gif_pub.publish(gif)

    def publish_hearing_probs(self):
        gif = HRIFeedbackFromInterface()
        gif.header.stamp = rp.Time.now()
        gif.type = HRIFeedbackFromInterface.HEARING_PROBLEMS_INDICATED
        self.gif_pub.publish(gif)


if __name__ == '__main__':
    rp.init_node('upo_behavior_tester')

    gp = EventPublisher()

    rp.loginfo('test node running')
    option_dict = {'1': gp.publish_waypoint_req,
                   '2': gp.publish_interactiontarget_req,
                   '3': gp.publish_walkside_req,
                   '4': gp.publish_low_batt,
                   '5': gp.publish_call_over,
                   '6': gp.publish_hearing_probs}

    while not rp.is_shutdown():
        print 'Select an event type:'
        print '1: Navigate to Waypoint Request'
        print '2: Navigate to Interaction Target Request'
        print '3: Walk side-by-side Request'
        print '4: Low Battery'
        print '5: Call Over'
        print '6: Hearing Problems Indicated'

        a = raw_input('> ')
        try:
            option_dict[a]()
        except KeyError:
            print 'Option not recognized. Please select an option from the list'

    rp.spin()

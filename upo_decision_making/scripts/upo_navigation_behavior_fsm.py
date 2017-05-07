#!/usr/bin/env python
import rospy as rp
import rospkg
import smach
import smach_ros
import threading
import actionlib
import abc

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from upo_decision_making.msg import ControlEvent, IDArray, HRIFeedbackStateInfo
from smach_common import EventMonitorState
from upo_navigation_macro_actions.msg import *
from hri_common import WaypointParser
#from hri_feedback_msgs.msg import HRIFeedbackStateInfo
#from teresa_driver.msg import StalkRef


MAX_HEAD_HEIGHT = 1.41


class SocialNavigationState(smach.State):
    __metaclass__ = abc.ABCMeta

    def __init__(self, macro_name, macro_type):
        smach.State.__init__(self, outcomes=['aborted',
                                             'blocked',
                                             'yield',
                                             'low battery',
                                             'call over',
                                             'manual control',
                                             'nav to waypoint req',
                                             'nav to target req',
                                             'walk with it req',
                                             'succeeded',
                                             'preempted',
                                             'unknown'])
        self._control_ret = None
        self._ac = actionlib.SimpleActionClient(macro_name, macro_type)
        self._macro_type = macro_type
        self._macro_name = macro_name
        self._actionlib_status_dict = {GoalStatus.ABORTED: 'aborted',
                                       GoalStatus.PREEMPTED: 'preempted',
                                       GoalStatus.SUCCEEDED: 'succeeded'}
        self._control_ev_dict = {ControlEvent.CALL_OVER: 'call over',
                                 ControlEvent.LOW_BATT: 'low battery',
                                 ControlEvent.MANUAL_CONTROLLING: 'manual control',
                                 ControlEvent.NAV_TO_PERSON_REQUESTED: 'nav to target req',
                                 ControlEvent.NAV_TO_WAYPOINT_REQUESTED: 'nav to waypoint req',
                                 ControlEvent.WALK_WITH_PERSON_REQUESTED: 'walk with it req',}
        while not self._ac.wait_for_server(rp.Duration(1.0)):
            rp.logwarn("Action state '%s' waiting for navigation actionlib server to start..."
                       % macro_name)
        #self._stalkpub = rp.Publisher("/stalk_ref", StalkRef, queue_size=1)
        rp.loginfo("Action state '%s' ready to execute." % macro_name)
        self._stateinfo_pub = rp.Publisher("hri_state_info", HRIFeedbackStateInfo, queue_size=1)
        self._events_sub = rp.Subscriber("behavior_manager/control_events",
                                         ControlEvent, self.event_cb)
    @abc.abstractmethod
    def get_goal(self):
        raise NotImplementedError("You can't call this pure virtual method!")

    def event_cb(self, msg):
        if msg.type in self._control_ev_dict:
            self._control_ret = self._control_ev_dict[msg.type]
            self._ac.cancel_all_goals()  # ac already waiting for result

    def execute(self, userdata):
        # Sending a reset signal to the stalk
        #self.reset_stalk()
        self._control_ret = None
        self.get_goal()
        rp.loginfo("Macro-action '%s': Sending goal." % self._macro_name)
        stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='started')
        self._stateinfo_pub.publish(stateinfo)
        GoalType = globals()[self._macro_name + "Goal"]
        if self._control_ret is None:
            self._ac.send_goal(GoalType(self._goal))
            self._ac.wait_for_result()
            a = str(self._ac.get_goal_status_text())
            if a.find('blocked') > -1:
                stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:blocked')
                self._stateinfo_pub.publish(stateinfo)
                return 'blocked'  # special check for the "blocked" situation
            if a.find('social path') > -1:
                stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:yield')
                self._stateinfo_pub.publish(stateinfo)
                return 'yield'      # special check for the "yield" situation    

            if self._control_ret is not None:
                stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:interrupted')
                self._stateinfo_pub.publish(stateinfo)
                return self._control_ret

            goal_state = self._ac.get_state()
            try:
                retstr = self._actionlib_status_dict[goal_state]
                stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:%s' % retstr)
                self._stateinfo_pub.publish(stateinfo)
                return retstr
            except KeyError:
                stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:unknown')
                self._stateinfo_pub.publish(stateinfo)
                return 'unknown'
        else:
            stateinfo = HRIFeedbackStateInfo(state_name=self._macro_name, info='ended:interrupted')
            self._stateinfo_pub.publish(stateinfo)
            return self._control_ret

    #def reset_stalk(self):
    #    sr = StalkRef()
    #    sr.head_height = MAX_HEAD_HEIGHT
    #    self._stalkpub.publish(sr)


class NavGoalActionState(SocialNavigationState):
    def __init__(self, macro_name, macro_type):
        self._thr_ev = threading.Event()
        super(NavGoalActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        rp.Subscriber("behavior_manager/nav_goal",
                      PoseStamped,
                      self.goal_cb)
        self._thr_ev.wait()
        self._thr_ev.clear()

    def goal_cb(self, msg):
        self._goal = msg
        self._thr_ev.set()


class ITIDActionState(SocialNavigationState):
    def __init__(self, macro_name, macro_type):
        self._thr_ev = threading.Event()
        super(ITIDActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        rp.Subscriber("behavior_manager/interaction_target_ids",
                      IDArray,
                      self.goal_cb)
        self._thr_ev.wait()
        self._thr_ev.clear()

    def goal_cb(self, msg):
        self._goal = msg.ids[0]
        self._thr_ev.set()


class FixedGoalActionState(SocialNavigationState):
    def __init__(self, macro_name, macro_type, fixed_goal):
        self._goal = fixed_goal
        super(FixedGoalActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        return self._goal  # already in self._goal


class NavigationBehaviorTest(smach.StateMachine):
    def __init__(self, conversation_active=False):
        smach.StateMachine.__init__(self,
                                    outcomes=['nav error',
                                              'nav preempted',
                                              'nav succeeded',
                                              'nav exit'],
                                    input_keys=['transition_label'])
        self._initial_state_dict = {'init': 'Wait for Goal',
                                    'nav to waypoint req': 'Navigate to Waypoint',
                                    'nav to target req': 'Navigate to Interaction Target',
                                    'walk with it req': 'Walk with Target',
                                    'manual control': 'Assisted Steering',
                                    'call over': 'Navigate Home',
                                    'low batt': 'Navigate Home'}
        wp_def_path = rp.get_param('/waypoint_defs')
        rospack = rospkg.RosPack()
        base_path = rospack.get_path("teresa_common")
        if base_path[-1] != "/":
            base_path += "/"
        wp_parser = WaypointParser(base_path + wp_def_path)
        homepos = wp_parser.get_waypoint('home')

	if conversation_active:
		tran_if_goal_reached = 'nav succeeded'
	else:
		tran_if_goal_reached = 'Wait for Goal' 	

        with self:
            self.add('Wait for Goal',
                     EventMonitorState({ControlEvent.NAV_TO_WAYPOINT_REQUESTED: 'nav to waypoint req',
                                        ControlEvent.NAV_TO_PERSON_REQUESTED: 'nav to person req',
                                        ControlEvent.WALK_WITH_PERSON_REQUESTED: 'walk with it req',
                                        ControlEvent.MANUAL_CONTROLLING: 'manual control',
                                        ControlEvent.CALL_OVER: 'call over',
                                        ControlEvent.LOW_BATT: 'low batt'}),
                     transitions={'nav to waypoint req': 'Navigate to Waypoint',
                                  'nav to person req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low batt': 'Navigate Home',
                                  'aborted': 'nav exit'})  # Warning: Not preemptable - needs concurrency for that
            self.add('Navigate to Waypoint',
                     NavGoalActionState("NavigateWaypoint", NavigateWaypointAction),
                     transitions={'succeeded': tran_if_goal_reached, 
                                  'aborted': 'Navigate to Waypoint',
                                  'nav to waypoint req': 'Navigate to Waypoint',
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to Waypoint',
                                  'yield': 'Yield in Navigation to Waypoint',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Navigate to Interaction Target',
                     ITIDActionState("NavigateInteractionTarget", NavigateInteractionTargetAction),
                     transitions={'succeeded': tran_if_goal_reached, 
                                  'aborted': 'Wait for Goal', 
                                  'nav to waypoint req': 'Navigate to Waypoint',
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to Target',
                                  'yield': 'Yield in Navigation to IT',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Wait for Path to Waypoint',
                     NavGoalActionState("Wait", WaitAction),
                     transitions={'succeeded': 'Navigate to Waypoint',
                                  'aborted': 'Wait for Path to Waypoint',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'nav error',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Wait for Path to Target',
                     NavGoalActionState("Wait", WaitAction),
                     transitions={'succeeded': 'Navigate to Interaction Target',
                                  'aborted': 'Wait for Path to Target',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'nav error',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Wait for Path Home',
                     NavGoalActionState("Wait", WaitAction),
                     transitions={'succeeded': 'Navigate Home',
                                  'aborted': 'Wait for Path Home',
                                  'nav to waypoint req': 'Navigate Home', #'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate Home',  #'Navigate to Interaction Target',
                                  'walk with it req': 'Navigate Home',   #'Walk with Target',
                                  'blocked': 'nav error',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Navigate Home',
                     FixedGoalActionState("NavigateHome", NavigateHomeAction, fixed_goal = homepos),
                     transitions={'succeeded': 'Wait for Goal',
                                  'aborted': 'Navigate Home',
                                  'nav to waypoint req': 'Navigate Home', #'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate Home', #'Navigate to Interaction Target',
                                  'walk with it req': 'Navigate Home',
                                  'blocked': 'Wait for Path Home',
                                  'yield': 'Yield in Navigation Home',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',  # shouldn't happen, but needs to be specified for the sake of the abstraction
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Yield in Navigation to Waypoint',
                    NavGoalActionState("Yield", YieldAction),
                    transitions={'succeeded': 'Navigate to Waypoint',
                                  'aborted': 'Yield in Navigation to Waypoint',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to Waypoint',
                                  'yield': 'nav error', 
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Yield in Navigation to IT',
                     NavGoalActionState("Yield", YieldAction),
                     transitions={'succeeded': 'Navigate to Interaction Target',
                                  'aborted': 'Yield in Navigation to IT',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to Target',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Yield in Navigation Home',
                     NavGoalActionState("Yield", YieldAction),
                     transitions={'succeeded': 'Navigate Home',
                                  'aborted': 'Yield in Navigation Home',
                                  'nav to waypoint req': 'Navigate Home',  # new goal received
                                  'nav to target req': 'Navigate Home',
                                  'walk with it req': 'Navigate Home',
                                  'blocked': 'Wait for Path Home',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Walk with Target',
                     ITIDActionState("WalkSideBySide", WalkSideBySideAction),
                     transitions={'succeeded': tran_if_goal_reached, 
                                  'aborted': 'Wait for Goal',
                                  'nav to waypoint req': 'Navigate to Waypoint',
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to walk with IT',
                                  'yield': 'Yield in walking with IT',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Wait for Path to walk with IT',
                     NavGoalActionState("Wait", WaitAction),
                     transitions={'succeeded': 'Walk with Target',
                                  'aborted': 'Wait for Path to walk with IT',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'nav error',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Yield in walking with IT',
                     NavGoalActionState("Yield", YieldAction),
                     transitions={'succeeded': 'Walk with Target',
                                  'aborted': 'Yield in walking with IT',
                                  'nav to waypoint req': 'Navigate to Waypoint',  # new goal received
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'Wait for Path to walk with IT',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Assisted Steering',
                     FixedGoalActionState("AssistedSteering", AssistedSteeringAction, fixed_goal=Twist()),
                     transitions={'succeeded': 'Wait for Goal', 
                                  'aborted': 'Assisted Steering',
                                  'nav to waypoint req': 'Navigate to Waypoint',
                                  'nav to target req': 'Navigate to Interaction Target',
                                  'walk with it req': 'Walk with Target',
                                  'blocked': 'nav error',
                                  'yield': 'nav error',
                                  'manual control': 'Assisted Steering',
                                  'call over': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})

    def execute(self, parent_ud=smach.UserData()):
        try:
            l = parent_ud.transition_label
        except KeyError:
            l = 'init'
        s = self._initial_state_dict[l]
        self.set_initial_state([s])
        out = smach.StateMachine.execute(self, parent_ud)
        return out

if __name__ == '__main__':
    rp.init_node("navigation_behavior")

    sm = NavigationBehaviorTest()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('navigation_behavior', sm, '/SM_ROOT')
    sis.start()

    # Execute state machine
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    rp.spin()
    sm.request_preempt()
    smach_thread.join()
    sis.stop()

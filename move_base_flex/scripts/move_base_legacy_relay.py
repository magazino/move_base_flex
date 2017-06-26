#!/usr/bin/env python
import actionlib
import copy
import move_base_flex_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped
from move_base.cfg import MoveBaseConfig

__author__ = 'Jorge Santos'

"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay dynamic reconfiguration calls. NOTE: some parameters have changed names; see wiki for details
"""


def simple_goal_cb(msg):
    mbf_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
    rospy.logdebug("Relaying move_base_simple/goal pose to mbf")
    mbf_ac.wait_for_result()


def mb_execute_cb(msg):
    mbf_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose), feedback_cb=mbf_feedback_cb)
    rospy.logdebug("Relaying legacy move_base goal to mbf")
    mbf_ac.wait_for_result()

    status = mbf_ac.get_state()
    result = mbf_ac.get_result()

    rospy.logdebug("MBF execution completed with status [%d]: %s", result.status, result.error_msg)
    if result.status == mbf_msgs.MoveBaseResult.SUCCESS:
        mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
    elif result.status == mbf_msgs.MoveBaseResult.CANCELED:
        mb_as.set_preempted()
    elif result.status == mbf_msgs.MoveBaseResult.CTRL_FAILURE:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(),
                          "Failed to find a valid control. Even after executing recovery behaviors.")
    elif result.status == mbf_msgs.MoveBaseResult.PLAN_FAILURE:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(),
                          "Failed to find a valid plan. Even after executing recovery behaviors.")
    elif result.status == mbf_msgs.MoveBaseResult.OSCILLATION:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(),
                          "Robot is oscillating. Even after executing recovery behaviors.")
    else:  # mbf can also fail with FAILURE, COLLISION, GOAL_BLOCKED or START_BLOCKED
        mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.error_msg)


def mbf_feedback_cb(feedback):
    mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))


def mb_reconf_cb(config, level):
    rospy.logdebug("Relaying legacy move_base reconfigure request to mbf")

    if not hasattr(mb_reconf_cb, "default_config"):
        mb_reconf_cb.default_config = copy.deepcopy(config)

    if config.get('restore_defaults'):
        config = mb_reconf_cb.default_config

    mbf_config = copy.deepcopy(config)

    # Map move_base legacy parameters to new mbf ones
    if 'base_local_planner' in mbf_config:
        mbf_config['local_planner'] = mbf_config.pop('base_local_planner')
    if 'controller_frequency' in mbf_config:
        mbf_config['local_planner_frequency'] = mbf_config.pop('controller_frequency')
    if 'controller_patience' in mbf_config:
        mbf_config['local_planner_patience'] = mbf_config.pop('controller_patience')
    if 'max_controller_retries' in mbf_config:
        mbf_config['local_planner_max_retries'] = mbf_config.pop('max_controller_retries')
    if 'base_global_planner' in mbf_config:
        mbf_config['global_planner'] = mbf_config.pop('base_global_planner')
    if 'planner_frequency' in mbf_config:
        mbf_config['global_planner_frequency'] = mbf_config.pop('planner_frequency')
    if 'planner_patience' in mbf_config:
        mbf_config['global_planner_patience'] = mbf_config.pop('planner_patience')
    if 'max_planning_retries' in mbf_config:
        mbf_config['global_planner_max_retries'] = mbf_config.pop('max_planning_retries')
    if 'conservative_reset_dist' in mbf_config:
        mbf_config.pop('conservative_reset_dist')  # no mbf equivalent for this!

    mbf_drc.update_configuration(mbf_config)
    return config


if __name__ == '__main__':
    rospy.init_node("move_base")

    # TODO what happens with malformed target goal???  FAILURE  or INVALID_POSE
    # txt must be:  "Aborting on goal because it was sent with an invalid quaternion"   

    # move_base_flex action and dynamic reconfigure clients 
    mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_ac.wait_for_server(rospy.Duration(20))
    mbf_drc = Client("move_base_flex", timeout=10)

    # move_base simple topic and action server
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, simple_goal_cb)
    mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    # move_base dynamic reconfigure server
    mb_drs = Server(MoveBaseConfig, mb_reconf_cb)

    rospy.spin()

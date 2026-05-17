#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MoveIt-based pick-and-place on a REAL Dobot CR5 (V4 firmware).

Unlike pick_place_real_v4.py (which speaks the dashboard text protocol on
TCP 29999 directly), this script goes through the ROS stack:

  - dobot_bringup brings up the controller bridge. It owns the TCP
    connection, publishes /joint_states from the real robot, and
    exposes ROS services (EnableRobot, ClearError, SpeedFactor, ...)
    plus a FollowJointTrajectory action server.
  - cr5_moveit's move_group plans trajectories for the "cr5_arm" group
    and ships them to that action server.
  - This script is just the orchestrator: it enables the robot, then
    asks MoveIt to move to each waypoint.

Same Cartesian waypoints as pick_place_real_v4.py, so the motion should
match the dashboard-protocol version.

Run order (three terminals; all need the workspace sourced and
DOBOT_TYPE=cr5 exported):
  1. roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1
  2. roslaunch cr5_moveit cr5_moveit.launch
  3. rosrun dobot_bringup pick_place_moveit_real_v4.py _cycles:=1 _speed:=15
"""

import math
import sys

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

from dobot_v4_bringup.srv import (
    EnableRobot, DisableRobot, ClearError, SpeedFactor,
)


HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

GRASP_DWELL_S = 1.0


def pose_from_real(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
    """Build a Pose from the same (mm, deg) coordinates used by the dashboard script."""
    p = geometry_msgs.msg.Pose()
    p.position.x = x_mm / 1000.0
    p.position.y = y_mm / 1000.0
    p.position.z = z_mm / 1000.0
    q = quaternion_from_euler(
        math.radians(rx_deg),
        math.radians(ry_deg),
        math.radians(rz_deg),
    )
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


PICK_APPROACH  = pose_from_real(-150.0, -400.0, 350.0, 180.0, 0.0, -90.0)
PICK           = pose_from_real(-150.0, -400.0, 220.0, 180.0, 0.0, -90.0)
PLACE_APPROACH = pose_from_real( 150.0, -400.0, 350.0, 180.0, 0.0, -90.0)
PLACE          = pose_from_real( 150.0, -400.0, 220.0, 180.0, 0.0, -90.0)


def call_service(name, srv_type, *args):
    rospy.wait_for_service(name, timeout=10.0)
    proxy = rospy.ServiceProxy(name, srv_type)
    resp = proxy(*args)
    rospy.loginfo("%s -> %s", name, resp)
    return resp


def move_to_joints(group, joints):
    group.set_joint_value_target(joints)
    ok = group.go(wait=True)
    group.stop()
    if not ok:
        raise RuntimeError("joint move failed: %s" % joints)


def move_to_pose(group, pose):
    """Equivalent of MovJ() -- joint-interpolated motion to a Cartesian pose."""
    group.set_pose_target(pose)
    ok = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if not ok:
        raise RuntimeError("pose move failed")


def move_cartesian(group, target_pose, eef_step=0.005, min_fraction=0.95):
    """Equivalent of MovL() -- straight-line Cartesian motion."""
    waypoints = [target_pose]
    plan, fraction = group.compute_cartesian_path(
        waypoints, eef_step, 0.0, avoid_collisions=False
    )
    if fraction < min_fraction:
        raise RuntimeError("cartesian path only %.1f%% computed" % (fraction * 100))
    group.execute(plan, wait=True)
    group.stop()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("cr5_pick_place_moveit_real", anonymous=False)

    cycles       = int(rospy.get_param("~cycles", 1))
    speed        = int(rospy.get_param("~speed", 15))           # 1..100
    skip_disable = bool(rospy.get_param("~skip_disable", False))
    vel_scale    = float(rospy.get_param("~vel_scale", 0.2))    # MoveIt scaling, separate from controller SpeedFactor
    acc_scale    = float(rospy.get_param("~acc_scale", 0.2))

    rospy.loginfo("[info] waiting for dobot_bringup services ...")
    call_service("/dobot_v4_bringup/srv/ClearError",  ClearError)
    call_service("/dobot_v4_bringup/srv/EnableRobot", EnableRobot)
    call_service("/dobot_v4_bringup/srv/SpeedFactor", SpeedFactor, speed)
    rospy.sleep(1.0)

    group = moveit_commander.MoveGroupCommander("cr5_arm")
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)

    rospy.loginfo("[info] planning frame : %s", group.get_planning_frame())
    rospy.loginfo("[info] end effector  : %s", group.get_end_effector_link())

    try:
        rospy.loginfo("[info] -> home")
        move_to_joints(group, HOME_JOINTS)

        for i in range(cycles):
            if rospy.is_shutdown():
                break
            rospy.loginfo("[info] === cycle %d / %d ===", i + 1, cycles)

            rospy.loginfo("[info]  -> pick approach")
            move_to_pose(group, PICK_APPROACH)

            rospy.loginfo("[info]  -> pick (linear)")
            move_cartesian(group, PICK)
            rospy.loginfo("[info]     [grasp dwell %.1fs]", GRASP_DWELL_S)
            rospy.sleep(GRASP_DWELL_S)

            rospy.loginfo("[info]  -> retreat (linear)")
            move_cartesian(group, PICK_APPROACH)

            rospy.loginfo("[info]  -> place approach")
            move_to_pose(group, PLACE_APPROACH)

            rospy.loginfo("[info]  -> place (linear)")
            move_cartesian(group, PLACE)
            rospy.loginfo("[info]     [release dwell %.1fs]", GRASP_DWELL_S)
            rospy.sleep(GRASP_DWELL_S)

            rospy.loginfo("[info]  -> retreat (linear)")
            move_cartesian(group, PLACE_APPROACH)

        rospy.loginfo("[info] -> home (final)")
        move_to_joints(group, HOME_JOINTS)

    except Exception as e:
        rospy.logerr("[error] %s", e)
    finally:
        if not skip_disable:
            try:
                call_service("/dobot_v4_bringup/srv/DisableRobot", DisableRobot)
            except Exception as e:
                rospy.logwarn("disable failed: %s", e)
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("[info] done.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

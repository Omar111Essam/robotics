#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Minimal MoveIt flow test on a REAL Dobot CR5.

Purpose: prove the end-to-end pipe works without committing to a real
pick-and-place motion. Exercises every piece the full demo relies on:
  - dobot_bringup services (ClearError / EnableRobot / SpeedFactor / DisableRobot)
  - /joint_states from the real controller
  - moveit_commander planning for group "cr5_arm"
  - FollowJointTrajectory execution on the real action server

Motion: reads the CURRENT joint configuration, nudges ONLY joint 6
(wrist roll) by +/- a small angle (default 5 deg), then returns to the
exact starting joints. Joint 6 has the smallest swept volume on the arm
and is away from the candle singularity, so this is the safest possible
non-trivial trajectory to validate the stack.

Run order (three terminals, all with the workspace sourced and
DOBOT_TYPE=cr5 exported):
  1. roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1
  2. roslaunch cr5_moveit cr5_moveit.launch
  3. rosrun dobot_bringup moveit_flow_test_real_v4.py \
         _delta_deg:=5 _speed:=10 _dry_run:=false

Useful params (private, set with _name:=value):
  _delta_deg   : how many degrees to wiggle joint 6 (default 5)
  _joint_index : 0..5, which joint to wiggle (default 5 = wrist roll)
  _speed       : controller SpeedFactor 1..100 (default 10)
  _vel_scale   : MoveIt velocity scaling 0..1 (default 0.1)
  _acc_scale   : MoveIt acceleration scaling 0..1 (default 0.1)
  _dwell_s     : pause at the nudged pose before returning (default 1.0)
  _dry_run     : if true, plan but do NOT execute (default false)
"""

import math
import sys

import rospy
import moveit_commander

from dobot_v4_bringup.srv import (
    EnableRobot, DisableRobot, ClearError, SpeedFactor,
)

# V4 bringup advertises everything under /dobot_v4_bringup/srv/...
SRV_NS = "/dobot_v4_bringup/srv"


def call_service(name, srv_type, *args):
    rospy.wait_for_service(name, timeout=10.0)
    proxy = rospy.ServiceProxy(name, srv_type)
    resp = proxy(*args)
    rospy.loginfo("%s -> %s", name, resp)
    return resp


def plan_and_execute(group, joints, dry_run):
    group.set_joint_value_target(joints)
    if dry_run:
        plan = group.plan()
        # plan return shape differs across MoveIt versions; just check
        # that it produced some trajectory points.
        traj = plan[1] if isinstance(plan, tuple) else plan
        n = len(getattr(traj.joint_trajectory, "points", []))
        rospy.loginfo("[dry-run] planned trajectory with %d points", n)
        if n == 0:
            raise RuntimeError("planner returned empty trajectory")
        return
    ok = group.go(wait=True)
    group.stop()
    if not ok:
        raise RuntimeError("execution failed for target %s" % joints)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("cr5_moveit_flow_test", anonymous=False)

    delta_deg    = float(rospy.get_param("~delta_deg", 5.0))
    joint_index  = int(rospy.get_param("~joint_index", 5))      # joint6 = wrist roll
    speed        = int(rospy.get_param("~speed", 10))
    vel_scale    = float(rospy.get_param("~vel_scale", 0.1))
    acc_scale    = float(rospy.get_param("~acc_scale", 0.1))
    dwell_s      = float(rospy.get_param("~dwell_s", 1.0))
    skip_disable = bool(rospy.get_param("~skip_disable", False))
    dry_run      = bool(rospy.get_param("~dry_run", False))

    if not (0 <= joint_index <= 5):
        rospy.logfatal("joint_index must be 0..5")
        return
    if abs(delta_deg) > 20.0:
        rospy.logfatal("delta_deg=%g looks unsafe; keep |delta| <= 20", delta_deg)
        return

    if not dry_run:
        rospy.loginfo("[info] waiting for dobot_v4_bringup services ...")
        call_service("/dobot_v4_bringup/srv/ClearError",  ClearError)
        call_service("/dobot_v4_bringup/srv/EnableRobot", EnableRobot)
        call_service("/dobot_v4_bringup/srv/SpeedFactor", SpeedFactor, speed)
        rospy.sleep(1.0)
    else:
        rospy.loginfo("[info] dry-run: skipping all bringup service calls")

    group = moveit_commander.MoveGroupCommander("cr5_arm")
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)

    rospy.loginfo("[info] planning frame : %s", group.get_planning_frame())
    rospy.loginfo("[info] end effector  : %s", group.get_end_effector_link())

    start = list(group.get_current_joint_values())
    if len(start) != 6:
        rospy.logfatal("expected 6 joints, got %d", len(start))
        return
    rospy.loginfo("[info] start joints (rad) : %s",
                  ["%.4f" % v for v in start])
    rospy.loginfo("[info] start joints (deg) : %s",
                  ["%.2f" % math.degrees(v) for v in start])

    nudged = list(start)
    nudged[joint_index] = start[joint_index] + math.radians(delta_deg)
    rospy.loginfo("[info] nudging joint%d by %+0.2f deg %s",
                  joint_index + 1, delta_deg,
                  "(DRY RUN, no execution)" if dry_run else "")

    try:
        rospy.loginfo("[info]  -> nudged target")
        plan_and_execute(group, nudged, dry_run)
        if dwell_s > 0 and not dry_run:
            rospy.loginfo("[info]     [dwell %.1fs]", dwell_s)
            rospy.sleep(dwell_s)

        rospy.loginfo("[info]  -> back to start")
        plan_and_execute(group, start, dry_run)

        rospy.loginfo("[info] flow test OK")

    except Exception as e:
        rospy.logerr("[error] %s", e)
    finally:
        if not skip_disable and not dry_run:
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

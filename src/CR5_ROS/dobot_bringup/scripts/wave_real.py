#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Service-based wave demo for the real Dobot CR5.

Talks to the dobot_bringup driver (which must already be running, e.g.
    roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1
). Sequence:

  1. ClearError + EnableRobot.
  2. SpeedFactor at a low ratio for safety.
  3. JointMovJ to a standing home pose.
  4. Loop: alternate RelMovJ on joint5 (wrist) to produce a wave.
  5. On Ctrl+C, return to home, then DisableRobot.

IMPORTANT
  * The dobot_bringup *services* take DEGREES, not radians.
    (The FollowJointTrajectory action takes radians and converts internally.)
  * `/joint_states` from dobot_bringup is in RADIANS, so we convert
    when comparing against the target.
  * Service calls are non-blocking on the driver side -- the controller
    queues the motion. We poll /joint_states to wait for arrival.

USAGE
    rosrun dobot_bringup wave_real.py            # default params
    rosrun dobot_bringup wave_real.py _speed:=15 _amplitude_deg:=30 _cycles:=5
"""

import math
import threading

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from dobot_bringup.srv import (
    ClearError,
    EnableRobot,
    DisableRobot,
    SpeedFactor,
    JointMovJ,
    RelMovJ,
)

# Standing "home" pose in DEGREES (matches the sim's HOME_POSE in radians).
HOME_DEG = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]

# Tolerance (degrees) for considering a joint "arrived".
ARRIVAL_TOL_DEG = 1.5

# Hard timeout (seconds) waiting for any single move to complete.
MOVE_TIMEOUT_S = 20.0


class JointStateTracker(object):
    """Subscribes to /joint_states (radians) and keeps the latest reading."""

    def __init__(self):
        self._lock = threading.Lock()
        self._positions_rad = None  # list of 6 floats, or None until first msg
        self._sub = rospy.Subscriber('/joint_states', JointState,
                                     self._cb, queue_size=1)

    def _cb(self, msg):
        if len(msg.position) < 6:
            return
        with self._lock:
            self._positions_rad = list(msg.position[:6])

    def positions_deg(self):
        with self._lock:
            if self._positions_rad is None:
                return None
            return [math.degrees(p) for p in self._positions_rad]

    def wait_for_first(self, timeout=5.0):
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.positions_deg() is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rate.sleep()
        return False


def call_service(name, srv_type, **kwargs):
    """Wait for service, then call it. Returns the response or None on failure."""
    rospy.loginfo("Waiting for service %s ...", name)
    rospy.wait_for_service(name)
    try:
        proxy = rospy.ServiceProxy(name, srv_type)
        return proxy(**kwargs)
    except rospy.ServiceException as e:
        rospy.logerr("Service %s failed: %s", name, e)
        return None


def wait_until_close_to(tracker, target_deg, tol_deg=ARRIVAL_TOL_DEG,
                        timeout_s=MOVE_TIMEOUT_S):
    """Block until every joint is within tol_deg of target_deg, or timeout."""
    t0 = rospy.Time.now()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        current = tracker.positions_deg()
        if current is not None:
            err = [abs(current[i] - target_deg[i]) for i in range(6)]
            if max(err) < tol_deg:
                return True
        if (rospy.Time.now() - t0).to_sec() > timeout_s:
            rospy.logwarn("wait_until_close_to: timed out. target=%s current=%s",
                          target_deg, current)
            return False
        rate.sleep()
    return False


def go_home(tracker):
    rospy.loginfo("Moving to home pose %s deg ...", HOME_DEG)
    call_service('/dobot_bringup/srv/JointMovJ', JointMovJ,
                 j1=HOME_DEG[0], j2=HOME_DEG[1], j3=HOME_DEG[2],
                 j4=HOME_DEG[3], j5=HOME_DEG[4], j6=HOME_DEG[5])
    wait_until_close_to(tracker, HOME_DEG)


def wave_once(tracker, amplitude_deg):
    """One full wave cycle on joint5: +amp, then -2*amp (to the other side),
    then +amp back to home. Uses RelMovJ so we don't accumulate drift."""
    rospy.loginfo("Wave: +%.1f deg on joint5", amplitude_deg)
    base = list(HOME_DEG)
    target_plus = list(base); target_plus[4]  += amplitude_deg
    target_minus = list(base); target_minus[4] -= amplitude_deg

    call_service('/dobot_bringup/srv/RelMovJ', RelMovJ,
                 offset1=0, offset2=0, offset3=0, offset4=0,
                 offset5=+amplitude_deg, offset6=0)
    wait_until_close_to(tracker, target_plus)

    rospy.loginfo("Wave: -%.1f deg on joint5", 2 * amplitude_deg)
    call_service('/dobot_bringup/srv/RelMovJ', RelMovJ,
                 offset1=0, offset2=0, offset3=0, offset4=0,
                 offset5=-2 * amplitude_deg, offset6=0)
    wait_until_close_to(tracker, target_minus)

    rospy.loginfo("Wave: +%.1f deg on joint5 (back to home)", amplitude_deg)
    call_service('/dobot_bringup/srv/RelMovJ', RelMovJ,
                 offset1=0, offset2=0, offset3=0, offset4=0,
                 offset5=+amplitude_deg, offset6=0)
    wait_until_close_to(tracker, base)


def main():
    rospy.init_node('cr5_wave_real', anonymous=False)

    speed_ratio  = int(rospy.get_param('~speed', 20))           # 1..100
    amplitude    = float(rospy.get_param('~amplitude_deg', 25)) # wrist sway
    cycles       = int(rospy.get_param('~cycles', 3))           # how many waves
    skip_disable = bool(rospy.get_param('~skip_disable', False))

    rospy.loginfo("CR5 wave_real starting: speed=%d%%, amplitude=%.1f deg, cycles=%d",
                  speed_ratio, amplitude, cycles)

    tracker = JointStateTracker()
    if not tracker.wait_for_first(timeout=5.0):
        rospy.logerr("No /joint_states received. Is dobot_bringup running and "
                     "connected to the robot?")
        return

    call_service('/dobot_bringup/srv/ClearError',  ClearError)
    call_service('/dobot_bringup/srv/EnableRobot', EnableRobot)
    call_service('/dobot_bringup/srv/SpeedFactor', SpeedFactor, ratio=speed_ratio)

    try:
        go_home(tracker)
        for i in range(cycles):
            if rospy.is_shutdown():
                break
            rospy.loginfo("--- wave cycle %d / %d ---", i + 1, cycles)
            wave_once(tracker, amplitude)
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted; returning home.")
    finally:
        if not rospy.is_shutdown():
            try:
                go_home(tracker)
            except Exception as e:
                rospy.logwarn("go_home on shutdown failed: %s", e)
        if not skip_disable:
            rospy.loginfo("Disabling robot.")
            call_service('/dobot_bringup/srv/DisableRobot', DisableRobot)
        rospy.loginfo("Done.")


if __name__ == '__main__':
    main()

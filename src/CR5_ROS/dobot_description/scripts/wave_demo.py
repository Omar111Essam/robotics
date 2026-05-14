#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Wave demo for the Dobot CR5.

Publishes /joint_states so robot_state_publisher + RViz show the robot:
  1. Smoothly moving from the all-zero (collapsed) pose to an upright
     "standing" home pose.
  2. Continuously waving the wrist back and forth, like a hand wave.

Replaces joint_state_publisher_gui in display.launch.
"""

import math
import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Upright "standing" home pose for the CR5 (radians).
# joint2 = -pi/2 lifts the shoulder so the upper arm points up,
# joint3 =  pi/2 brings the forearm forward/up,
# joint4 = -pi/2 keeps the wrist aligned, joint5 = pi/2 orients the gripper.
HOME_POSE = [0.0,
             -math.pi / 2.0,
              math.pi / 2.0,
             -math.pi / 2.0,
              math.pi / 2.0,
              0.0]

# How long (seconds) to interpolate from zero to the home pose.
MOVE_TO_HOME_TIME = 3.0

# Wave parameters: oscillate joint1 (base) + joint5 (wrist) like a hand wave.
WAVE_AMPLITUDE_BASE  = 0.6   # rad, ~34 deg, joint1 sway
WAVE_AMPLITUDE_WRIST = 0.8   # rad, ~46 deg, joint5 sway
WAVE_FREQ_HZ         = 0.5   # one full wave cycle every 2 s

PUBLISH_RATE_HZ = 50.0


def lerp(a, b, t):
    return a + (b - a) * t


def main():
    rospy.init_node('cr5_wave_demo', anonymous=False)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE_HZ)

    # Give robot_state_publisher / RViz a moment to come up.
    rospy.sleep(1.0)

    msg = JointState()
    msg.name = JOINT_NAMES

    start_time = rospy.Time.now()
    moved_home = False
    home_finished_time = None

    rospy.loginfo("CR5 wave demo: moving to standing pose, then waving.")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()

        if not moved_home:
            t = min(elapsed / MOVE_TO_HOME_TIME, 1.0)
            # ease-in-out for a smoother motion
            s = 0.5 - 0.5 * math.cos(math.pi * t)
            positions = [lerp(0.0, HOME_POSE[i], s) for i in range(6)]
            if t >= 1.0:
                moved_home = True
                home_finished_time = now
                rospy.loginfo("Standing pose reached. Starting wave.")
        else:
            wave_t = (now - home_finished_time).to_sec()
            omega = 2.0 * math.pi * WAVE_FREQ_HZ
            base_offset  = WAVE_AMPLITUDE_BASE  * math.sin(omega * wave_t)
            wrist_offset = WAVE_AMPLITUDE_WRIST * math.sin(omega * wave_t)

            positions = list(HOME_POSE)
            positions[0] += base_offset   # joint1: sway the whole arm
            positions[4] += wrist_offset  # joint5: wave the wrist

        msg.header.stamp = now
        msg.position = positions
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

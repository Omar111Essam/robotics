#!/usr/bin/env python
# -- coding: utf-8 --
"""
MoveIt-based pick-and-place simulation for the Dobot CR5.
Uses the same Cartesian waypoints as pick_place_real_v4.py.
"""

import math
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

# === Same Cartesian waypoints as the real script ===
# [x, y, z, rx, ry, rz] in METERS and RADIANS (MoveIt convention).
# The real script used mm and degrees -- we convert here.
def pose_from_real(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
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

HOME_JOINTS    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
PICK_APPROACH  = pose_from_real(-150.0, -400.0, 350.0, 180.0, 0.0, -90.0)
PICK           = pose_from_real(-150.0, -400.0, 220.0, 180.0, 0.0, -90.0)
PLACE_APPROACH = pose_from_real( 150.0, -400.0, 350.0, 180.0, 0.0, -90.0)
PLACE          = pose_from_real( 150.0, -400.0, 220.0, 180.0, 0.0, -90.0)

GRASP_DWELL_S = 1.0


def move_to_joints(group, joints):
    group.set_joint_value_target(joints)
    group.go(wait=True)
    group.stop()


def move_to_pose(group, pose):
    """Equivalent of MovJ() -- joint-interpolated motion to a Cartesian pose."""
    group.set_pose_target(pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return success


def move_cartesian(group, target_pose, eef_step=0.005):
    """Equivalent of MovL() -- straight-line Cartesian motion."""
    waypoints = [target_pose]
    plan, fraction = group.compute_cartesian_path(
        waypoints, eef_step, 0.0, avoid_collisions=False
    )
    if fraction < 0.95:
        rospy.logwarn("Cartesian path only %.1f%% computed", fraction * 100)
        return False
    group.execute(plan, wait=True)
    group.stop()
    return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cr5_pick_place_moveit', anonymous=False)

    group = moveit_commander.MoveGroupCommander("cr5_arm")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_planning_time(5.0)

    rospy.loginfo("Planning frame: %s", group.get_planning_frame())
    rospy.loginfo("End effector  : %s", group.get_end_effector_link())

    cycles = rospy.get_param('~cycles', 1)

    rospy.loginfo("-> home")
    move_to_joints(group, HOME_JOINTS)

    for i in range(cycles):
        rospy.loginfo("=== cycle %d / %d ===", i + 1, cycles)

        rospy.loginfo(" -> pick approach")
        move_to_pose(group, PICK_APPROACH)

        rospy.loginfo(" -> pick (linear)")
        move_cartesian(group, PICK)
        rospy.sleep(GRASP_DWELL_S)

        rospy.loginfo(" -> retreat (linear)")
        move_cartesian(group, PICK_APPROACH)

        rospy.loginfo(" -> place approach")
        move_to_pose(group, PLACE_APPROACH)

        rospy.loginfo(" -> place (linear)")
        move_cartesian(group, PLACE)
        rospy.sleep(GRASP_DWELL_S)

        rospy.loginfo(" -> retreat (linear)")
        move_cartesian(group, PLACE_APPROACH)

    rospy.loginfo("-> home (final)")
    move_to_joints(group, HOME_JOINTS)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone pick-and-place motion demo for the Dobot CR5 (V4 firmware).

Talks directly to the controller's dashboard port (TCP 29999) using the
Dobot text protocol. No gripper actuation -- the "grasp" and "release"
points are just dwells, so this is purely a motion choreography.

Sequence:
  1. Connect, ClearError, EnableRobot, SpeedFactor.
  2. JointMovJ to a known home pose.
  3. MovL to PICK_APPROACH (above the part).
  4. MovL down to PICK (simulated grasp dwell).
  5. MovL back up to PICK_APPROACH.
  6. MovJ across to PLACE_APPROACH (above the drop point).
  7. MovL down to PLACE (simulated release dwell).
  8. MovL back up to PLACE_APPROACH.
  9. JointMovJ home, DisableRobot.

Usage:
    python3 pick_place_real_v4.py
    python3 pick_place_real_v4.py --ip 192.168.5.1 --speed 15 --cycles 2

Coordinates are in millimetres / degrees. Tweak PICK_* / PLACE_* to match
your workspace before running on real hardware.
"""

import argparse
import re
import signal
import socket
import sys
import time

# Candle / vertical-stretch pose: all links aligned, pointing straight up.
# Wrist singularity -- enter/leave it via JointMovJ only (which we do).
HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Cartesian poses: [x, y, z, rx, ry, rz] in mm and degrees.
# Tool pointing straight down: rx=180, ry=0, rz=0.
# Z values are at the tool flange. The attached gripper extends ~90 mm
# below the flange, so add that to keep the tip clear of the table.
PICK_APPROACH  = [-150.0, -400.0, 350.0, 180.0, 0.0, -90.0]
PICK           = [-150.0, -400.0, 220.0, 180.0, 0.0, -90.0]
PLACE_APPROACH = [ 150.0, -400.0, 350.0, 180.0, 0.0, -90.0]
PLACE          = [ 150.0, -400.0, 220.0, 180.0, 0.0, -90.0]

# Dwell at pick/place to simulate gripper actuation time.
GRASP_DWELL_S = 1.0


class DobotDashboard(object):
    """Minimal client for the Dobot V4 text protocol on port 29999."""

    def __init__(self, ip, port=29999, timeout=5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(0.3)
        try:
            self.sock.recv(4096)
        except socket.timeout:
            pass
        self.sock.settimeout(self.timeout)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def send(self, cmd, read_timeout=2.0):
        if self.sock is None:
            raise RuntimeError("not connected")
        self.sock.sendall((cmd + "\n").encode("ascii"))

        self.sock.settimeout(read_timeout)
        chunks = []
        deadline = time.time() + read_timeout
        try:
            while True:
                remaining = max(0.05, deadline - time.time())
                self.sock.settimeout(remaining)
                data = self.sock.recv(4096)
                if not data:
                    break
                chunks.append(data)
                if b';' in data:
                    break
        except socket.timeout:
            pass
        finally:
            self.sock.settimeout(self.timeout)
        return b"".join(chunks).decode("ascii", errors="replace").strip()

    @staticmethod
    def _err_code(reply):
        if not reply:
            return None
        try:
            return int(reply.split(",", 1)[0])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _parse_vec(reply):
        m = re.search(r"\{([^}]*)\}", reply)
        if not m:
            return None
        try:
            return [float(x) for x in m.group(1).split(",")]
        except ValueError:
            return None

    def get_angles(self):
        vec = self._parse_vec(self.send("GetAngle()"))
        return vec[:6] if vec else None

    def get_pose(self):
        vec = self._parse_vec(self.send("GetPose()"))
        return vec[:6] if vec else None


def wait_joints(dash, target_deg, tol=1.5, timeout=25.0):
    t0 = time.time()
    cur = None
    while time.time() - t0 < timeout:
        cur = dash.get_angles()
        if cur and len(cur) == 6:
            if max(abs(cur[i] - target_deg[i]) for i in range(6)) < tol:
                return True
        time.sleep(0.1)
    print("[warn] joint arrival timeout. target=%s last=%s" % (target_deg, cur))
    return False


def wait_pose(dash, target_pose, pos_tol=2.0, ori_tol=2.0, timeout=25.0):
    t0 = time.time()
    cur = None
    while time.time() - t0 < timeout:
        cur = dash.get_pose()
        if cur and len(cur) == 6:
            pos_err = max(abs(cur[i] - target_pose[i]) for i in range(3))
            ori_err = max(abs(cur[i] - target_pose[i]) for i in range(3, 6))
            if pos_err < pos_tol and ori_err < ori_tol:
                return True
        time.sleep(0.1)
    print("[warn] pose arrival timeout. target=%s last=%s" % (target_pose, cur))
    return False


def joint_move(dash, joints_deg):
    cmd = "JointMovJ(%s)" % ",".join("%.4f" % v for v in joints_deg)
    reply = dash.send(cmd)
    if DobotDashboard._err_code(reply) != 0:
        cmd2 = "MovJ(joint={%s})" % ",".join("%.4f" % v for v in joints_deg)
        reply = dash.send(cmd2)
        if DobotDashboard._err_code(reply) != 0:
            raise RuntimeError("JointMovJ rejected: %s" % reply)
    return reply


def cart_move_l(dash, pose):
    """Linear Cartesian move (straight line in tool space)."""
    args = ",".join("%.4f" % v for v in pose)
    reply = dash.send("MovL(%s)" % args)
    if DobotDashboard._err_code(reply) != 0:
        reply = dash.send("MovL(pose={%s})" % args)
        if DobotDashboard._err_code(reply) != 0:
            raise RuntimeError("MovL rejected: %s" % reply)
    return reply


def cart_move_j(dash, pose):
    """Joint-interpolated Cartesian move (curved path, often faster)."""
    args = ",".join("%.4f" % v for v in pose)
    reply = dash.send("MovJ(%s)" % args)
    if DobotDashboard._err_code(reply) != 0:
        reply = dash.send("MovJ(pose={%s})" % args)
        if DobotDashboard._err_code(reply) != 0:
            raise RuntimeError("MovJ rejected: %s" % reply)
    return reply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=int, default=15, help="SpeedFactor 1..100")
    ap.add_argument("--cycles", type=int, default=1, help="pick-and-place repeats")
    ap.add_argument("--skip-disable", action="store_true")
    args = ap.parse_args()

    print("[info] connecting to %s:29999 ..." % args.ip)
    dash = DobotDashboard(args.ip)
    dash.connect()

    interrupted = {"flag": False}
    def _on_sigint(_sig, _frm):
        interrupted["flag"] = True
        print("\n[info] Ctrl+C received, will return home and disable.")
    signal.signal(signal.SIGINT, _on_sigint)

    try:
        print("[info] mode    :", dash.send("RobotMode()"))
        print("[info] clear   :", dash.send("ClearError()"))
        print("[info] enable  :", dash.send("EnableRobot()"))
        print("[info] speed   :", dash.send("SpeedFactor(%d)" % args.speed))
        time.sleep(1.0)

        print("[info] -> home %s" % HOME_DEG)
        joint_move(dash, HOME_DEG)
        wait_joints(dash, HOME_DEG)

        for i in range(args.cycles):
            if interrupted["flag"]:
                break
            print("[info] === cycle %d / %d ===" % (i + 1, args.cycles))

            print("[info]  -> pick approach %s" % PICK_APPROACH)
            cart_move_j(dash, PICK_APPROACH)
            wait_pose(dash, PICK_APPROACH)
            if interrupted["flag"]: break

            print("[info]  -> pick %s" % PICK)
            cart_move_l(dash, PICK)
            wait_pose(dash, PICK)
            print("[info]     [grasp dwell %.1fs]" % GRASP_DWELL_S)
            time.sleep(GRASP_DWELL_S)
            if interrupted["flag"]: break

            print("[info]  -> retreat to pick approach")
            cart_move_l(dash, PICK_APPROACH)
            wait_pose(dash, PICK_APPROACH)
            if interrupted["flag"]: break

            print("[info]  -> place approach %s" % PLACE_APPROACH)
            cart_move_j(dash, PLACE_APPROACH)
            wait_pose(dash, PLACE_APPROACH)
            if interrupted["flag"]: break

            print("[info]  -> place %s" % PLACE)
            cart_move_l(dash, PLACE)
            wait_pose(dash, PLACE)
            print("[info]     [release dwell %.1fs]" % GRASP_DWELL_S)
            time.sleep(GRASP_DWELL_S)
            if interrupted["flag"]: break

            print("[info]  -> retreat to place approach")
            cart_move_l(dash, PLACE_APPROACH)
            wait_pose(dash, PLACE_APPROACH)

        print("[info] -> home (final)")
        joint_move(dash, HOME_DEG)
        wait_joints(dash, HOME_DEG)

    except Exception as e:
        print("[error]", e, file=sys.stderr)
    finally:
        if not args.skip_disable:
            print("[info] disable :", dash.send("DisableRobot()"))
        dash.close()
        print("[info] done.")


if __name__ == "__main__":
    main()

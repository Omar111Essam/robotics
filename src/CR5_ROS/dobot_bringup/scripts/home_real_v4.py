#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Send the Dobot CR5 (V4 firmware) back to its home pose.

Talks directly to the controller's dashboard port (TCP 29999) using the
Dobot text protocol -- same approach as wave_real_v4.py / pick_place_real_v4.py.

Sequence:
  1. Connect, ClearError, EnableRobot, SpeedFactor.
  2. JointMovJ to HOME_DEG.
  3. Wait until arrived, DisableRobot, close.

Usage:
    python3 home_real_v4.py
    python3 home_real_v4.py --ip 192.168.5.1 --speed 15
    python3 home_real_v4.py --skip-disable    # leave brakes off afterwards
"""

import argparse
import re
import signal
import socket
import sys
import time

# Candle / vertical-stretch pose: all links aligned and pointing straight up.
# J2=-90 raises the upper arm to vertical; J3..J6=0 keeps everything collinear.
# Note: this is a wrist singularity -- only enter/leave it with joint moves.
HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class DobotDashboard(object):
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

    def get_angles(self):
        reply = self.send("GetAngle()")
        m = re.search(r"\{([^}]*)\}", reply)
        if not m:
            return None
        try:
            return [float(x) for x in m.group(1).split(",")][:6]
        except ValueError:
            return None


def wait_joints(dash, target_deg, tol=1.5, timeout=25.0):
    t0 = time.time()
    cur = None
    while time.time() - t0 < timeout:
        cur = dash.get_angles()
        if cur and len(cur) == 6:
            if max(abs(cur[i] - target_deg[i]) for i in range(6)) < tol:
                return True
        time.sleep(0.1)
    print("[warn] arrival timeout. target=%s last=%s" % (target_deg, cur))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=int, default=15, help="SpeedFactor 1..100")
    ap.add_argument("--skip-disable", action="store_true")
    args = ap.parse_args()

    print("[info] connecting to %s:29999 ..." % args.ip)
    dash = DobotDashboard(args.ip)
    dash.connect()

    interrupted = {"flag": False}
    def _on_sigint(_sig, _frm):
        interrupted["flag"] = True
        print("\n[info] Ctrl+C received, will disable and exit.")
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

    except Exception as e:
        print("[error]", e, file=sys.stderr)
    finally:
        if not args.skip_disable:
            print("[info] disable :", dash.send("DisableRobot()"))
        dash.close()
        print("[info] done.")


if __name__ == "__main__":
    main()

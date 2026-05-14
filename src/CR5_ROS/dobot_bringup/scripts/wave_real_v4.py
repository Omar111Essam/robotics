#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone wave demo for the Dobot CR5 running V4-style firmware.

This script talks DIRECTLY to the controller's dashboard port (TCP 29999)
using the Dobot text protocol. It does NOT use dobot_bringup or ROS.

Why: the WELLBEINGLWB/CR5_ROS dobot_bringup driver expects port 30003
for motion commands, but V4 firmware moved all commands to 29999 and
does not expose 30003 at all -- so dobot_bringup fails on connect.

Sequence:
  1. Connect to 192.168.5.1:29999.
  2. ClearError(), EnableRobot(), SpeedFactor(low).
  3. JointMovJ to a standing home pose. Wait via Sync().
  4. Loop: RelJointMovJ on joint 5 (+amp, -2*amp, +amp).
  5. Return home, DisableRobot.

Usage:
    python3 wave_real_v4.py
    python3 wave_real_v4.py --ip 192.168.5.1 --speed 15 --amp 20 --cycles 3

Run from anywhere -- no ROS environment needed.
"""

import argparse
import re
import signal
import socket
import sys
import time

# All values below are DEGREES.
HOME_DEG = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]


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
        # Some firmwares emit a banner on connect; drain anything ready.
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
        """Send a command and return the (possibly empty) reply string."""
        if self.sock is None:
            raise RuntimeError("not connected")
        payload = (cmd + "\n").encode("ascii")
        self.sock.sendall(payload)

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
                # Reply pattern is "errcode,{...},Cmd();" -- stop once we see ';'.
                if b';' in data:
                    break
        except socket.timeout:
            pass
        finally:
            self.sock.settimeout(self.timeout)
        reply = b"".join(chunks).decode("ascii", errors="replace").strip()
        return reply

    # ---- convenience wrappers ----

    @staticmethod
    def _err_code(reply):
        # Reply format: "<errcode>,{<value>},<Command>();"
        if not reply:
            return None
        try:
            return int(reply.split(",", 1)[0])
        except (ValueError, IndexError):
            return None

    def call(self, cmd, ok_codes=(0,), tolerate_empty=False):
        reply = self.send(cmd)
        code = self._err_code(reply)
        if code is None:
            if tolerate_empty:
                return reply
            raise RuntimeError("No/invalid reply to %r: %r" % (cmd, reply))
        if code not in ok_codes:
            raise RuntimeError("Robot rejected %r: %s" % (cmd, reply))
        return reply

    def get_angles(self):
        """Return [j1..j6] in degrees, or None if not parseable."""
        reply = self.send("GetAngle()")
        m = re.search(r"\{([^}]*)\}", reply)
        if not m:
            return None
        try:
            return [float(x) for x in m.group(1).split(",")][:6]
        except ValueError:
            return None


def wait_until_arrived(dash, target_deg, tol=1.5, timeout=20.0):
    """Poll GetAngle() until each joint is within tol of target, or timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        cur = dash.get_angles()
        if cur is not None and len(cur) == 6:
            if max(abs(cur[i] - target_deg[i]) for i in range(6)) < tol:
                return True
        time.sleep(0.1)
    print("[warn] arrival timeout. target=%s last=%s" % (target_deg, cur))
    return False


def joint_move(dash, joints_deg):
    cmd = "JointMovJ(%s)" % ",".join("%.4f" % v for v in joints_deg)
    reply = dash.send(cmd)
    code = DobotDashboard._err_code(reply)
    if code != 0:
        # V4 fallback syntax
        cmd2 = "MovJ(joint={%s})" % ",".join("%.4f" % v for v in joints_deg)
        reply = dash.send(cmd2)
        code = DobotDashboard._err_code(reply)
        if code != 0:
            raise RuntimeError("Both JointMovJ and MovJ(joint=) rejected: %s" % reply)
    return reply


def rel_joint_move(dash, offsets_deg):
    cmd = "RelJointMovJ(%s)" % ",".join("%.4f" % v for v in offsets_deg)
    reply = dash.send(cmd)
    code = DobotDashboard._err_code(reply)
    if code != 0:
        # alternate spelling on some firmwares
        cmd2 = "RelMovJ(%s)" % ",".join("%.4f" % v for v in offsets_deg)
        reply = dash.send(cmd2)
        code = DobotDashboard._err_code(reply)
        if code != 0:
            raise RuntimeError("Rel move rejected: %s" % reply)
    return reply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=int, default=15, help="SpeedFactor 1..100")
    ap.add_argument("--amp", type=float, default=20.0, help="wrist sway in degrees")
    ap.add_argument("--cycles", type=int, default=3)
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

        # Give the controller a beat to finish enabling.
        time.sleep(1.0)

        print("[info] -> home %s" % HOME_DEG)
        joint_move(dash, HOME_DEG)
        wait_until_arrived(dash, HOME_DEG, timeout=25.0)

        for i in range(args.cycles):
            if interrupted["flag"]:
                break
            print("[info] --- wave %d / %d ---" % (i + 1, args.cycles))
            # +amp
            t = list(HOME_DEG); t[4] += args.amp
            rel_joint_move(dash, [0, 0, 0, 0, +args.amp, 0])
            wait_until_arrived(dash, t)
            if interrupted["flag"]: break
            # -2*amp (other side)
            t = list(HOME_DEG); t[4] -= args.amp
            rel_joint_move(dash, [0, 0, 0, 0, -2 * args.amp, 0])
            wait_until_arrived(dash, t)
            if interrupted["flag"]: break
            # +amp (back to home)
            rel_joint_move(dash, [0, 0, 0, 0, +args.amp, 0])
            wait_until_arrived(dash, HOME_DEG)

        # Always finish at home.
        print("[info] -> home (final)")
        joint_move(dash, HOME_DEG)
        wait_until_arrived(dash, HOME_DEG, timeout=25.0)

    except Exception as e:
        print("[error]", e, file=sys.stderr)
    finally:
        if not args.skip_disable:
            print("[info] disable :", dash.send("DisableRobot()"))
        dash.close()
        print("[info] done.")


if __name__ == "__main__":
    main()

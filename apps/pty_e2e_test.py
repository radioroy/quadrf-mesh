#!/usr/bin/env python3
"""E2E test: communicate with quadrf-lora-phy --legacy-pty via Meshtastic SerialInterface.

Verifies:
  1. want_config handshake (my_info / channel / complete)
  2. sendtext -> digital/OTA loopback FromRadio.packet with matching text
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pty", help="Existing PTY slave path")
    ap.add_argument("--daemon", default="build/apps/quadrf-lora-phy")
    ap.add_argument("--ota", action="store_true")
    ap.add_argument("--amplitude", type=float, default=None)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--symlink", default="/tmp/quadrf-lora-phy")
    args = ap.parse_args()

    daemon = None
    pty = args.pty
    if pty is None:
        cmd = [
            args.daemon,
            "--legacy-pty",
            "--preset",
            "shortturbo",
            "--symlink",
            args.symlink,
        ]
        if args.ota:
            cmd.append("--ota")
            if args.amplitude is None:
                args.amplitude = 0.15
        if args.amplitude is not None:
            cmd.extend(["--amplitude", str(args.amplitude)])
        daemon = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        # Wait for PTY= line or symlink
        deadline = time.time() + 20
        while time.time() < deadline:
            if os.path.islink(args.symlink) or os.path.exists(args.symlink):
                pty = os.path.realpath(args.symlink)
                break
            if daemon.poll() is not None:
                out = daemon.stdout.read() if daemon.stdout else ""
                print("daemon exited early:\n", out, file=sys.stderr)
                return 1
            time.sleep(0.1)
        if pty is None:
            print("timed out waiting for PTY symlink", file=sys.stderr)
            daemon.terminate()
            return 1
        print(f"using PTY {pty}")

    # Import after daemon is up so failures are clearer
    from pubsub import pub
    from meshtastic.serial_interface import SerialInterface

    got = {"packet": None, "connected": False}

    def on_receive(packet, interface):  # noqa: ARG001
        got["packet"] = packet

    def on_connection(interface, topic=pub.AUTO_TOPIC):  # noqa: ARG001
        got["connected"] = True

    pub.subscribe(on_receive, "meshtastic.receive")
    pub.subscribe(on_connection, "meshtastic.connection.established")

    iface = None
    try:
        iface = SerialInterface(devPath=pty, noProto=False)
        deadline = time.time() + args.timeout
        while time.time() < deadline and not got["connected"]:
            time.sleep(0.1)
        if not got["connected"]:
            # SerialInterface may already be past handshake; check myInfo
            if getattr(iface, "myInfo", None) is None:
                print("FAIL: no connection / myInfo", file=sys.stderr)
                return 1
            got["connected"] = True
        print(f"connected my_node={iface.myInfo.my_node_num:#x}")

        text = "quadrf-pty-e2e"
        print(f"sendtext: {text}")
        iface.sendText(text)

        deadline = time.time() + args.timeout
        while time.time() < deadline:
            pkt = got["packet"]
            if pkt is not None:
                decoded = pkt.get("decoded") or {}
                payload = decoded.get("payload") or decoded.get("text")
                if isinstance(payload, bytes):
                    payload = payload.decode("utf-8", errors="replace")
                # meshtastic may put text at top level
                text_rx = pkt.get("text") or decoded.get("text") or payload
                print(f"rx packet: {pkt}")
                if text_rx == text or (isinstance(text_rx, str) and text in text_rx):
                    print("PASS: e2e text loopback")
                    return 0
                # keep waiting for a matching packet
                got["packet"] = None
            time.sleep(0.1)
        print("FAIL: timed out waiting for loopback text", file=sys.stderr)
        return 1
    finally:
        try:
            if iface is not None:
                iface.close()
        except Exception:
            pass
        if daemon is not None:
            daemon.terminate()
            try:
                daemon.wait(timeout=5)
            except subprocess.TimeoutExpired:
                daemon.kill()


if __name__ == "__main__":
    sys.exit(main())

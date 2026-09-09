#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
"""
Automated bidirectional LoRa test harness for QuadRF mesh nodes.
Measures delivery success rate, SNR distributions, CFO, and collision dynamics.
"""

import argparse
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

DEFAULT_HOST1 = os.environ.get("QUADRF_HOST1", "quadrf.local")
DEFAULT_HOST2 = os.environ.get("QUADRF_HOST2", "quadrf-2.local")
DEFAULT_MESHTASTIC_BIN = os.environ.get(
    "MESHTASTIC_BIN", shutil.which("meshtastic") or "meshtastic"
)


def ssh_pass() -> str:
    p = os.environ.get("QUADRF_PASS", "")
    if not p:
        raise SystemExit("set QUADRF_PASS in the environment")
    return p

RX_RE = re.compile(
    r"phy RX air=(?P<air>\d+) snr=(?P<snr>[\d\.\-]+) cfo=(?P<cfo>[\d\.\-]+) ppm=(?P<ppm>[\d\.\-]+)(?P<echo> echo-drop)?"
)
ROUTER_RX_RE = re.compile(
    r"Received text msg from=(?P<from>0x[0-9a-f]+), id=(?P<id>0x[0-9a-f]+), msg=(?P<msg>.+)"
)
TX_RE = re.compile(r"tx RF unmute")


@dataclass
class NodeLogCollector:
    name: str
    host: str
    rx_events: List[dict] = field(default_factory=list)
    text_events: List[dict] = field(default_factory=list)
    tx_count: int = 0
    _running: bool = False
    _phy_proc: Optional[subprocess.Popen] = None
    _mesh_proc: Optional[subprocess.Popen] = None
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def start(self):
        self._running = True
        self._phy_proc = subprocess.Popen(
            [
                "sshpass", "-p", ssh_pass(),
                "ssh", "-o", "StrictHostKeyChecking=no", f"dietpi@{self.host}",
                "sudo journalctl -u quadrf-lora-phy -f -n 0 -o cat"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._mesh_proc = subprocess.Popen(
            [
                "sshpass", "-p", ssh_pass(),
                "ssh", "-o", "StrictHostKeyChecking=no", f"dietpi@{self.host}",
                "sudo journalctl -u quadrf-meshtasticd -f -n 0 -o cat"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._read_phy, daemon=True).start()
        threading.Thread(target=self._read_mesh, daemon=True).start()

    def _read_phy(self):
        for line in iter(self._phy_proc.stdout.readline, ''):
            if not self._running:
                break
            line = line.strip()
            if "tx RF unmute" in line:
                with self._lock:
                    self.tx_count += 1
            m = RX_RE.search(line)
            if m:
                ev = {
                    "time": time.time(),
                    "air": int(m.group("air")),
                    "snr": float(m.group("snr")),
                    "cfo": float(m.group("cfo")),
                    "ppm": float(m.group("ppm")),
                    "is_echo": bool(m.group("echo")),
                    "raw": line,
                }
                with self._lock:
                    self.rx_events.append(ev)

    def _read_mesh(self):
        for line in iter(self._mesh_proc.stdout.readline, ''):
            if not self._running:
                break
            line = line.strip()
            m = ROUTER_RX_RE.search(line)
            if m:
                ev = {
                    "time": time.time(),
                    "from": m.group("from"),
                    "id": m.group("id"),
                    "msg": m.group("msg"),
                    "raw": line,
                }
                with self._lock:
                    self.text_events.append(ev)

    def stop(self):
        self._running = False
        if self._phy_proc:
            self._phy_proc.terminate()
            try:
                self._phy_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._phy_proc.kill()
        if self._mesh_proc:
            self._mesh_proc.terminate()
            try:
                self._mesh_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._mesh_proc.kill()

    def get_peer_rx_events(self, since_time: float) -> List[dict]:
        with self._lock:
            return [e for e in self.rx_events if e["time"] >= since_time and not e["is_echo"]]

    def get_text_events(self, since_time: float) -> List[dict]:
        with self._lock:
            return [e for e in self.text_events if e["time"] >= since_time]


def send_meshtastic_text(host: str, msg: str, meshtastic_bin: str = DEFAULT_MESHTASTIC_BIN) -> bool:
    try:
        res = subprocess.run(
            [meshtastic_bin, "--host", f"{host}:4403", "--sendtext", msg],
            capture_output=True,
            text=True,
            timeout=15,
        )
        return res.returncode == 0
    except Exception as e:
        print(f"Error sending to {host}: {e}")
        return False


def run_campaign(
    host1: str = DEFAULT_HOST1,
    host2: str = DEFAULT_HOST2,
    rounds: int = 10,
    pause_s: float = 10.0,
    rx_timeout: float = 6.0,
    meshtastic_bin: str = DEFAULT_MESHTASTIC_BIN,
):
    print(f"Starting test campaign between {host1} and {host2}: {rounds} ping-pong rounds, pause={pause_s}s, rx_timeout={rx_timeout}s")
    c1 = NodeLogCollector(name=f"Q1 ({host1})", host=host1)
    c2 = NodeLogCollector(name=f"Q2 ({host2})", host=host2)
    c1.start()
    c2.start()

    time.sleep(1.5)  # Let journalctl connect

    q1_to_q2_results = []
    q2_to_q1_results = []

    try:
        for r in range(1, rounds + 1):
            # Direction 1: Q1 -> Q2
            msg1 = f"ping-q1-r{r}-{int(time.time()) % 10000}"
            print(f"\n--- Round {r}/{rounds}: Q1 -> Q2 ({msg1}) ---")
            t0 = time.time()
            ok1 = send_meshtastic_text(host1, msg1, meshtastic_bin=meshtastic_bin)
            
            # Poll for reception up to rx_timeout
            recvd1 = False
            q2_peer_rx = []
            q2_text = []
            deadline = time.time() + rx_timeout + 6.0
            while time.time() < deadline:
                q2_peer_rx = c2.get_peer_rx_events(t0)
                q2_text = c2.get_text_events(t0)
                if any(msg1 in t["msg"] for t in q2_text) or len(q2_peer_rx) > 0:
                    recvd1 = True
                    break
                time.sleep(0.3)

            snr1 = q2_peer_rx[0]["snr"] if q2_peer_rx else None
            cfo1 = q2_peer_rx[0]["cfo"] if q2_peer_rx else None

            res1 = {
                "round": r,
                "msg": msg1,
                "sent": ok1,
                "decoded": recvd1,
                "snr": snr1,
                "cfo": cfo1,
                "rx_count": len(q2_peer_rx),
                "text_count": len(q2_text),
            }
            q1_to_q2_results.append(res1)
            print(f"  Result Q1->Q2: sent={ok1}, decoded={recvd1}, snr={snr1} dB, cfo={cfo1} Hz, rx_events={len(q2_peer_rx)}")

            # Pause to let rebroadcasts settle
            time.sleep(pause_s)

            # Direction 2: Q2 -> Q1
            msg2 = f"pong-q2-r{r}-{int(time.time()) % 10000}"
            print(f"--- Round {r}/{rounds}: Q2 -> Q1 ({msg2}) ---")
            t1 = time.time()
            ok2 = send_meshtastic_text(host2, msg2, meshtastic_bin=meshtastic_bin)

            recvd2 = False
            q1_peer_rx = []
            q1_text = []
            deadline = time.time() + rx_timeout + 6.0
            while time.time() < deadline:
                q1_peer_rx = c1.get_peer_rx_events(t1)
                q1_text = c1.get_text_events(t1)
                if any(msg2 in t["msg"] for t in q1_text) or len(q1_peer_rx) > 0:
                    recvd2 = True
                    break
                time.sleep(0.3)

            snr2 = q1_peer_rx[0]["snr"] if q1_peer_rx else None
            cfo2 = q1_peer_rx[0]["cfo"] if q1_peer_rx else None

            res2 = {
                "round": r,
                "msg": msg2,
                "sent": ok2,
                "decoded": recvd2,
                "snr": snr2,
                "cfo": cfo2,
                "rx_count": len(q1_peer_rx),
                "text_count": len(q1_text),
            }
            q2_to_q1_results.append(res2)
            print(f"  Result Q2->Q1: sent={ok2}, decoded={recvd2}, snr={snr2} dB, cfo={cfo2} Hz, rx_events={len(q1_peer_rx)}")

            if r < rounds:
                time.sleep(pause_s)

    finally:
        c1.stop()
        c2.stop()

    # Print summary statistics
    print("\n" + "=" * 60)
    print("CAMPAIGN SUMMARY")
    print("=" * 60)

    def print_dir_stats(label: str, results: List[dict]):
        total = len(results)
        decoded = sum(1 for r in results if r["decoded"])
        rate = (decoded / total * 100.0) if total else 0.0
        snrs = [r["snr"] for r in results if r["snr"] is not None]
        cfos = [r["cfo"] for r in results if r["cfo"] is not None]
        avg_snr = sum(snrs) / len(snrs) if snrs else 0.0
        min_snr = min(snrs) if snrs else 0.0
        max_snr = max(snrs) if snrs else 0.0
        avg_cfo = sum(cfos) / len(cfos) if cfos else 0.0

        print(f"Direction {label}:")
        print(f"  Packets Sent:     {total}")
        print(f"  Packets Decoded:  {decoded} ({rate:.1f}%)")
        print(f"  SNR (mean/min/max): {avg_snr:.2f} / {min_snr:.2f} / {max_snr:.2f} dB")
        print(f"  CFO (mean):       {avg_cfo:.1f} Hz")

    print_dir_stats(f"Q1 -> Q2 ({host1} -> {host2})", q1_to_q2_results)
    print()
    print_dir_stats(f"Q2 -> Q1 ({host2} -> {host1})", q2_to_q1_results)
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bidirectional LoRa test harness for QuadRF mesh nodes.")
    parser.add_argument("--host1", default=DEFAULT_HOST1, help=f"Host/IP for node 1 (default: {DEFAULT_HOST1})")
    parser.add_argument("--host2", default=DEFAULT_HOST2, help=f"Host/IP for node 2 (default: {DEFAULT_HOST2})")
    parser.add_argument("--meshtastic", default=DEFAULT_MESHTASTIC_BIN, help=f"Meshtastic CLI binary (default: {DEFAULT_MESHTASTIC_BIN})")
    parser.add_argument("--rounds", type=int, default=10, help="Number of ping-pong rounds")
    parser.add_argument("--pause", type=float, default=10.0, help="Pause seconds between sends")
    parser.add_argument("--timeout", type=float, default=6.0, help="Timeout seconds for rx wait")
    args = parser.parse_args()
    run_campaign(
        host1=args.host1,
        host2=args.host2,
        rounds=args.rounds,
        pause_s=args.pause,
        rx_timeout=args.timeout,
        meshtastic_bin=args.meshtastic,
    )

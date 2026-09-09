#!/usr/bin/env python3
"""Exact TX sample-slip measurement via correlation against a known sequence.

TX a 1-second white (PRBS-derived) complex sequence on repeat through the
digital loopback. Every RX block is matched against the full reference by
FFT cross-correlation, giving the absolute TX offset of that block with
single-sample resolution and a full second of unambiguous range. Slips at
DSI frame boundaries appear as steps in (tx_offset - rx_time).
"""

import os
import subprocess
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32

FS = 1e6
PAT_SEC = 1.0
CAP_SEC = 5.0
BLK = 2048            # correlation block (2 ms)
HOP = 4096            # analyze one block every 4.1 ms (< frame period)
JTAG = os.environ.get("QUADRF_JTAG", "/usr/bin/quadrf-jtag")


def jtag_read(addr):
    out = subprocess.run([JTAG, "read", f"0x{addr:02X}"], capture_output=True,
                         text=True).stdout
    return int(out.rsplit("0x", 1)[1], 16)


def jtag_write(addr, val):
    subprocess.run([JTAG, "write", f"0x{addr:02X}", f"0x{val:04X}"],
                   capture_output=True, text=True)


def main():
    n_pat = int(FS * PAT_SEC)
    rng = np.random.default_rng(12345)
    # band-limit to 400 kHz so the loopback path passes it cleanly
    spec = (rng.standard_normal(n_pat) + 1j * rng.standard_normal(n_pat))
    f = np.fft.fftfreq(n_pat, 1 / FS)
    spec[np.abs(f) > 400e3] = 0
    pat = np.fft.ifft(spec)
    pat = (0.55 * pat / np.abs(pat).std() / 3).astype(np.complex64)
    np.clip(pat.view(np.float32), -0.95, 0.95, out=pat.view(np.float32))
    print(f"pattern rms {np.sqrt(np.mean(np.abs(pat)**2)):.3f}")

    saved = jtag_read(0x2E)
    jtag_write(0x2E, 0x0004)
    try:
        dev = SoapySDR.Device(dict(driver="mipi"))
        dev.setSampleRate(SOAPY_SDR_TX, 0, FS)
        dev.setSampleRate(SOAPY_SDR_RX, 0, FS)
        txs = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0])
        rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        mtu_tx = dev.getStreamMTU(txs)
        mtu_rx = dev.getStreamMTU(rxs)

        rx_buf = np.zeros(int(FS * (CAP_SEC + 1)), np.complex64)
        rx_pos = [0]
        running = [True]

        def tx_run():
            off = 0
            buf = np.empty(mtu_tx, np.complex64)
            while running[0]:
                idx = (off + np.arange(mtu_tx)) % n_pat
                buf[:] = pat[idx]
                r = dev.writeStream(txs, [buf], mtu_tx)
                if r.ret > 0:
                    off += r.ret

        def rx_run():
            tmp = np.zeros(mtu_rx, np.complex64)
            while running[0]:
                if rx_pos[0] + mtu_rx > len(rx_buf):
                    break
                r = dev.readStream(rxs, [tmp], mtu_rx)
                if r.ret > 0:
                    rx_buf[rx_pos[0]:rx_pos[0] + r.ret] = tmp[:r.ret]
                    rx_pos[0] += r.ret

        tt = threading.Thread(target=tx_run)
        rt = threading.Thread(target=rx_run)
        tt.start()
        time.sleep(0.05)
        rt.start()
        dev.activateStream(rxs)
        dev.activateStream(txs)
        time.sleep(CAP_SEC)
        running[0] = False
        tt.join()
        rt.join()
        dev.deactivateStream(txs)
        dev.deactivateStream(rxs)
        dev.closeStream(txs)
        dev.closeStream(rxs)
    finally:
        jtag_write(0x2E, saved)

    rx = rx_buf[:rx_pos[0]][int(1.0 * FS):]
    print(f"analyzing {rx.size} samples, rms {np.sqrt(np.mean(np.abs(rx)**2)):.5f}")

    # FFT of the reference once; correlate each block
    P = np.fft.fft(pat)
    n_blocks = (rx.size - BLK) // HOP
    offs = np.empty(n_blocks)
    quality = np.empty(n_blocks)
    for b in range(n_blocks):
        seg = np.zeros(n_pat, np.complex64)
        seg[:BLK] = rx[b * HOP: b * HOP + BLK]
        xc = np.abs(np.fft.ifft(np.fft.fft(seg) * np.conj(P)))
        k = int(np.argmax(xc))
        offs[b] = k
        quality[b] = xc[k] / (np.median(xc) + 1e-12)

    t_b = np.arange(n_blocks) * HOP / FS
    # slip = how far TX content lags behind real time
    slip = np.unwrap((offs + t_b * FS) % n_pat, period=n_pat)
    slip -= slip[0]
    ok = quality > 8

    d = np.diff(slip)
    step_i = np.where(np.abs(d) > 8)[0]
    merged = []
    for i in step_i:
        if not (ok[i] and ok[i + 1]):
            continue
        if merged and i - merged[-1][0] <= 1:
            merged[-1] = (i, merged[-1][1] + d[i])
        else:
            merged.append((i, d[i]))
    sizes = np.array([m[1] for m in merged])
    locs = np.array([t_b[m[0]] for m in merged])

    print(f"\ncorrelation ok blocks: {ok.mean()*100:.1f}%")
    print(f"slips > 8 samples: {len(merged)}")
    for tl, s in list(zip(locs, sizes))[:30]:
        print(f"  {tl:8.4f}s  {s:+8.1f} samples")
    if len(locs) > 1:
        sp = np.diff(locs) * 1e3
        print(f"spacing: median {np.median(sp):.3f} ms  (DSI frame 19.272 ms)")
        print(f"sizes: median {np.median(sizes):+.1f}  mean {sizes.mean():+.1f}")
        print(f"net: {sizes.sum():+.0f} samples over {t_b[-1]:.2f} s "
              f"= {sizes.sum()/t_b[-1]/FS*1e6:+.0f} ppm contribution")

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    axes[0].plot(t_b[ok], slip[ok], ".", markersize=2)
    for tl in locs:
        axes[0].axvline(tl, color="C3", alpha=0.3, linewidth=0.7)
    axes[0].set_ylabel("TX slip (samples)")
    axes[0].set_title(f"Absolute TX offset error vs time  ({len(merged)} slips)")
    axes[0].grid(alpha=0.3)

    z = ok & (t_b < 0.5)
    axes[1].plot(t_b[z], slip[z], ".-", markersize=3, linewidth=0.5)
    axes[1].set_ylabel("TX slip (samples)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title("zoom: first 500 ms")
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("phy_dumps/prbs_slip.png", dpi=130, bbox_inches="tight")
    print("saved phy_dumps/prbs_slip.png")


if __name__ == "__main__":
    main()

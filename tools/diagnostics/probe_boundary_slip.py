#!/usr/bin/env python3
"""Disambiguate the DSI frame-boundary discontinuity.

TX a single slow linear FM sweep (unique frequency -> unique TX time)
through the FPGA digital loopback. The received instantaneous frequency
then reveals, at every discontinuity, exactly where in TX time the
stream jumped: forward = samples skipped, backward = samples repeated.
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
SWEEP_SEC = 30.0          # one-shot sweep, never repeats during capture
F0, F1 = 50e3, 450e3      # sweep range
CAP_SEC = 6.0
JTAG = os.environ.get("QUADRF_JTAG", "/usr/bin/quadrf-jtag")


def jtag_read(addr):
    out = subprocess.run([JTAG, "read", f"0x{addr:02X}"], capture_output=True,
                         text=True).stdout
    return int(out.rsplit("0x", 1)[1], 16)


def jtag_write(addr, val):
    subprocess.run([JTAG, "write", f"0x{addr:02X}", f"0x{val:04X}"],
                   capture_output=True, text=True)


def main():
    n_sweep = int(FS * SWEEP_SEC)
    t = np.arange(n_sweep) / FS
    k = (F1 - F0) / SWEEP_SEC
    phase = 2 * np.pi * (F0 * t + 0.5 * k * t * t)
    tx = (0.7 * np.exp(1j * phase)).astype(np.complex64)

    saved = jtag_read(0x2E)
    print(f"saved 0x2E = 0x{saved:04X}")
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

        def tx_thread():
            off = 0
            while running[0] and off + mtu_tx <= len(tx):
                r = dev.writeStream(txs, [tx[off:off + mtu_tx]], mtu_tx)
                if r.ret > 0:
                    off += r.ret
                elif r.ret < 0 and r.ret != SoapySDR.SOAPY_SDR_TIMEOUT:
                    break

        def rx_thread():
            tmp = np.zeros(mtu_rx, np.complex64)
            while running[0]:
                if rx_pos[0] + mtu_rx > len(rx_buf):
                    break
                r = dev.readStream(rxs, [tmp], mtu_rx)
                if r.ret > 0:
                    rx_buf[rx_pos[0]:rx_pos[0] + r.ret] = tmp[:r.ret]
                    rx_pos[0] += r.ret
                elif r.ret < 0 and r.ret != SoapySDR.SOAPY_SDR_TIMEOUT:
                    break

        tt = threading.Thread(target=tx_thread)
        rt = threading.Thread(target=rx_thread)
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
        print(f"restored 0x2E = 0x{saved:04X}")

    rx = rx_buf[:rx_pos[0]]
    print(f"captured {rx.size} samples")
    rx = rx[int(1.0 * FS):]  # drop startup

    # instantaneous frequency per 256-sample block -> implied TX time
    blk = 256
    nb = rx.size // blk
    x = rx[:nb * blk].reshape(nb, blk)
    prod = x[:, 1:] * np.conj(x[:, :-1])
    f_inst = np.angle(prod.mean(axis=1)) * FS / (2 * np.pi)
    t_rx = np.arange(nb) * blk / FS
    tx_time = (f_inst - F0) / k          # seconds into the TX sweep
    amp = np.abs(x).mean(axis=1)

    # implied TX time should advance at ~1 s/s (times the ~1e-3 rate offset).
    # discontinuities: block-to-block steps beyond the sweep's natural step
    d_txtime = np.diff(tx_time) * FS     # in TX samples
    natural = blk * 1.0                  # nominal advance per block
    resid = d_txtime - natural
    step_idx = np.where(np.abs(resid) > 400)[0]

    # merge adjacent detections
    merged = []
    for i in step_idx:
        if merged and i - merged[-1][0] <= 2:
            merged[-1] = (i, merged[-1][1] + resid[i])
        else:
            merged.append((i, resid[i]))

    print(f"\ndiscontinuities > 400 TX samples: {len(merged)}")
    sizes = np.array([m[1] for m in merged])
    locs = np.array([t_rx[m[0]] for m in merged])
    if len(merged):
        print("first 20 (time s, size in TX samples; + = skip, - = repeat):")
        for tloc, s in list(zip(locs, sizes))[:20]:
            print(f"  {tloc:8.4f}s  {s:+9.1f}")
        if len(locs) > 1:
            sp = np.diff(locs) * 1e3
            print(f"spacing: median {np.median(sp):.2f} ms  "
                  f"(DSI frame period = 19.27 ms)")
        print(f"median size: {np.median(sizes):+.1f} TX samples")
        print(f"net over capture: {sizes.sum():+.0f} TX samples "
              f"in {t_rx[-1]:.2f} s")

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(t_rx, tx_time, linewidth=0.7)
    axes[0].set_ylabel("implied TX time (s)")
    axes[0].set_title("TX time recovered from sweep frequency")
    axes[0].grid(alpha=0.3)

    detr = tx_time - t_rx * np.polyfit(t_rx, tx_time, 1)[0]
    axes[1].plot(t_rx, (detr - detr[0]) * FS, linewidth=0.7)
    for tloc in locs:
        axes[1].axvline(tloc, color="C3", alpha=0.25, linewidth=0.7)
    axes[1].set_ylabel("TX-time residual (samples)")
    axes[1].set_title("Detrended: jumps = skipped(+)/repeated(-) samples")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t_rx, 20 * np.log10(amp + 1e-9), linewidth=0.7)
    axes[2].set_ylabel("block amp (dB)")
    axes[2].set_xlabel("RX time (s)")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("phy_dumps/boundary_slip.png", dpi=130, bbox_inches="tight")
    print("saved phy_dumps/boundary_slip.png")
    rx.tofile("phy_dumps/boundary_slip_rx.iq")


if __name__ == "__main__":
    main()

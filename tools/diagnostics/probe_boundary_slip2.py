#!/usr/bin/env python3
"""Measure DSI frame-boundary sample slip, unambiguously.

TX a repeating 0.2 s linear sweep (k = 2 MHz/s). Mix RX with the model
sweep; the residual tone frequency f_res = k * (timing offset). A slip of
D samples at a frame boundary steps f_res by k*D/fs = 10 Hz per sample.
Track f_res with short phase-slope blocks: 1 kHz steps = 100-sample slips
resolve cleanly, ambiguity only beyond +-100k samples.
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
PERIOD = 0.2              # sweep period, s
SPAN = 400e3              # sweep span, Hz (-200k .. +200k)
CAP_SEC = 5.0
JTAG = os.environ.get("QUADRF_JTAG", "/usr/bin/quadrf-jtag")


def jtag_read(addr):
    out = subprocess.run([JTAG, "read", f"0x{addr:02X}"], capture_output=True,
                         text=True).stdout
    return int(out.rsplit("0x", 1)[1], 16)


def jtag_write(addr, val):
    subprocess.run([JTAG, "write", f"0x{addr:02X}", f"0x{val:04X}"],
                   capture_output=True, text=True)


def sweep_iq(n_total):
    n_per = int(FS * PERIOD)
    k = SPAN / PERIOD
    t = np.arange(n_per) / FS
    ph = 2 * np.pi * (-SPAN / 2 * t + 0.5 * k * t * t)
    one = (0.7 * np.exp(1j * ph)).astype(np.complex64)
    reps = int(np.ceil(n_total / n_per))
    return np.tile(one, reps)[:n_total], k, n_per


def main():
    tx, k, n_per = sweep_iq(int(FS * (CAP_SEC + 3)))

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
            while running[0] and off + mtu_tx <= len(tx):
                r = dev.writeStream(txs, [tx[off:off + mtu_tx]], mtu_tx)
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
    print(f"analyzing {rx.size} samples")

    # mix with the model sweep at every possible alignment: correlate one
    # period to find the alignment first
    n_corr = n_per
    model = sweep_iq(n_per)[0]
    seg = rx[:n_corr].astype(np.complex64)
    X = np.fft.fft(seg * 1.0)
    M = np.fft.fft(model)
    xc = np.abs(np.fft.ifft(X * np.conj(M)))
    align = int(np.argmax(xc))
    print(f"alignment: {align} samples (corr peak/median "
          f"{xc[align]/np.median(xc):.1f})")

    # build model stream aligned to rx
    reps = int(np.ceil((rx.size + align) / n_per)) + 1
    model_full = np.tile(model, reps)[align:align + rx.size]
    mixed = rx * np.conj(model_full)

    # residual frequency via phase slope in 512-sample blocks (0.512 ms)
    blk = 512
    nb = mixed.size // blk
    m = mixed[:nb * blk].reshape(nb, blk)
    prod = m[:, 1:] * np.conj(m[:, :-1])
    f_res = np.angle(prod.mean(axis=1)) * FS / (2 * np.pi)
    amp = np.abs(m).mean(axis=1)
    t_b = np.arange(nb) * blk / FS

    # timing offset in TX samples (mod 200k)
    slip = f_res / k * FS

    d = np.diff(slip)
    steps = np.where(np.abs(d) > 40)[0]
    merged = []
    for i in steps:
        if merged and i - merged[-1][0] <= 2:
            merged[-1] = (i, merged[-1][1] + d[i])
        else:
            merged.append((i, d[i]))
    sizes = np.array([m2[1] for m2 in merged])
    locs = np.array([t_b[m2[0]] for m2 in merged])

    print(f"\nslips > 40 samples: {len(merged)}")
    for tloc, s in list(zip(locs, sizes))[:25]:
        print(f"  {tloc:8.4f}s  {s:+9.1f} samples")
    if len(locs) > 1:
        sp = np.diff(locs) * 1e3
        print(f"spacing: median {np.median(sp):.2f} ms (DSI frame 19.27 ms)")
        print(f"median size {np.median(sizes):+.1f}, "
              f"mean {sizes.mean():+.1f} samples")
        print(f"implied rate loss: {np.median(sizes)/0.01927/FS*1e6:+.0f} ppm "
              f"if every frame")

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    axes[0].plot(t_b, slip, linewidth=0.7)
    for tloc in locs:
        axes[0].axvline(tloc, color="C3", alpha=0.3, linewidth=0.7)
    axes[0].set_ylabel("timing offset (TX samples)")
    axes[0].set_title(f"TX timing offset from sweep residual  "
                      f"({len(merged)} slips)")
    axes[0].grid(alpha=0.3)

    z = slice(0, int(0.25 / (blk / FS)))
    axes[1].plot(t_b[z], slip[z], ".-", markersize=2, linewidth=0.6)
    axes[1].set_ylabel("timing offset (samples)")
    axes[1].set_title("zoom: first 250 ms")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t_b, 20 * np.log10(amp + 1e-9), linewidth=0.6)
    axes[2].set_ylabel("block amp (dB)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("phy_dumps/boundary_slip2.png", dpi=130, bbox_inches="tight")
    print("saved phy_dumps/boundary_slip2.png")


if __name__ == "__main__":
    main()

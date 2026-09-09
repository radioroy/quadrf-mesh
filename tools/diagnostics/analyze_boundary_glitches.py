#!/usr/bin/env python3
"""Quantify periodic timing glitches in a digital-loopback tone capture.

The DSI TX path is suspected to lose a few samples at frame boundaries.
A sample slip of n at the DSI line rate shows up in the 1 MSps host
capture as a tone phase step of 2*pi*f_tone*n/fs_line.  This script
tracks the tone phase residual at fine time resolution, detects steps,
and checks their size and spacing against the DSI frame period.
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    return raw[0::2] + 1j * raw[1::2]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default="phy_dumps/probe_fixed")
    ap.add_argument("--skip", type=float, default=1.0)
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    dump = Path(args.dir)
    manifest = json.loads((dump / "probe_manifest.json").read_text())
    fs = float(manifest["sample_rate_hz"])
    f_tx = float(manifest["tone_hz"])

    x = load_iq(dump / "probe_rx_tone.iq")
    x = x[int(args.skip * fs):]

    # precise tone frequency from a long FFT
    n = 1 << 21
    seg = x[:n] * np.hanning(n)
    spec = np.abs(np.fft.fft(seg))
    freqs = np.fft.fftfreq(n, 1.0 / fs)
    guard = int(n * 2e3 / fs)
    spec[:guard] = 0
    spec[-guard:] = 0
    pk = int(np.argmax(spec))
    a, b, c = spec[pk - 1], spec[pk], spec[pk + 1]
    delta = 0.5 * (a - c) / (a - 2 * b + c)
    f_rx = freqs[pk] + delta * fs / n

    # phase residual after mixing the tone down; slope removed
    k = np.arange(x.size)
    mixed = x * np.exp(-2j * np.pi * f_rx * k / fs)
    dec = 8  # 8 us resolution
    m = mixed[: (x.size // dec) * dec].reshape(-1, dec).mean(axis=1)
    amp = np.abs(m)
    ph = np.unwrap(np.angle(m))
    t = np.arange(ph.size) * dec / fs
    coef = np.polyfit(t, ph, 1)
    res = ph - np.polyval(coef, t)
    # slip in host samples: full cycle of tone phase = fs/f_tx samples
    slip = res / (2 * np.pi) * fs / f_tx

    # step detection on a smoothed derivative
    win = 8
    kern = np.ones(win) / win
    smooth = np.convolve(slip, kern, mode="same")
    dstep = np.diff(smooth)

    # spectrum of the slip signal to find the glitch repetition rate
    slip_ac = slip - slip.mean()
    nfft = 1 << int(np.floor(np.log2(slip_ac.size)))
    sspec = np.abs(np.fft.rfft(slip_ac[:nfft] * np.hanning(nfft)))
    sfreq = np.fft.rfftfreq(nfft, dec / fs)
    lo = np.searchsorted(sfreq, 2.0)
    hi = np.searchsorted(sfreq, 500.0)
    pk_i = lo + int(np.argmax(sspec[lo:hi]))
    f_glitch = sfreq[pk_i]

    # fold the slip onto the detected period to see the boundary shape
    period = 1.0 / f_glitch
    phase_in_period = np.mod(t, period) / period
    nbins = 200
    idx = np.minimum((phase_in_period * nbins).astype(int), nbins - 1)
    folded = np.zeros(nbins)
    counts = np.zeros(nbins)
    # detrend slowly (remove drift over many periods) before folding
    slow = np.convolve(slip, np.ones(1024) / 1024, mode="same")
    fast = slip - slow
    np.add.at(folded, idx, fast)
    np.add.at(counts, idx, 1)
    folded /= np.maximum(counts, 1)

    pp = folded.max() - folded.min()

    print(f"tone: {f_rx/1e3:.4f} kHz  ({(f_rx/f_tx-1)*1e6:+.1f} ppm)")
    print(f"slip range: {slip.min():+.1f} .. {slip.max():+.1f} host samples")
    print(f"dominant glitch rate: {f_glitch:.2f} Hz (period {period*1e3:.2f} ms)")
    print(f"folded waveform peak-to-peak: {pp:.2f} host samples")
    print(f"amplitude dropouts (<50% median): "
          f"{np.mean(amp < 0.5*np.median(amp))*100:.3f}% of time")

    fig, axes = plt.subplots(2, 2, figsize=(15, 9))

    ax = axes[0, 0]
    ax.plot(t, slip, linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip (host samples)")
    ax.set_title("Tone phase residual as accumulated sample slip")
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    zoom = slice(0, int(0.2 / (dec / fs)))
    ax.plot(t[zoom], slip[zoom], linewidth=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip (host samples)")
    ax.set_title("Zoom: first 200 ms")
    ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ax.semilogy(sfreq[1:hi], sspec[1:hi] + 1e-9, linewidth=0.6)
    ax.axvline(f_glitch, color="C3", linestyle="--",
               label=f"{f_glitch:.1f} Hz")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("|FFT(slip)|")
    ax.set_title("Slip spectrum (glitch repetition rate)")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[1, 1]
    ax.plot(np.arange(nbins) / nbins * period * 1e3, folded, linewidth=0.9)
    ax.set_xlabel(f"Time within {period*1e3:.2f} ms period (ms)")
    ax.set_ylabel("Slip (host samples)")
    ax.set_title(f"Slip folded on glitch period  p-p {pp:.2f} samples")
    ax.grid(alpha=0.3)

    fig.suptitle(
        f"DSI boundary glitch analysis  tone {(f_rx/f_tx-1)*1e6:+.0f} ppm  "
        f"glitch {f_glitch:.1f} Hz  p-p {pp:.2f} samples @ {fs/1e6:.0f} MSps host",
        fontsize=11)
    fig.tight_layout()
    save = Path(args.save) if args.save else dump / "boundary_glitches.png"
    fig.savefig(save, dpi=130, bbox_inches="tight")
    print(f"saved {save}")


if __name__ == "__main__":
    main()

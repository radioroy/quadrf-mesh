#!/usr/bin/env python3
"""Analyze digital_loopback_probe dumps.

Answers three questions about the FPGA digital loopback path:
  1. What is the effective TX->RX sample rate ratio (tone frequency scaling)?
  2. Are there periodic discontinuities (sample loss at DSI frame boundaries)?
  3. How does the dechirped bin trajectory behave over time (ppm drift, jumps)?
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    return raw[0::2] + 1j * raw[1::2]


def measure_tone(rx: np.ndarray, fs: float, tone_tx_hz: float, skip_sec: float = 1.0) -> dict:
    x = rx[int(skip_sec * fs):]
    n = 1 << 20
    if x.size < n:
        n = 1 << int(np.floor(np.log2(x.size)))
    seg = x[:n] * np.hanning(n)
    spec = np.fft.fftshift(np.fft.fft(seg))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / fs))
    mags = np.abs(spec)

    # ignore DC region
    dc_guard = int(n * 2e3 / fs)
    center = n // 2
    mags_nodc = mags.copy()
    mags_nodc[center - dc_guard:center + dc_guard] = 0.0
    peak_idx = int(np.argmax(mags_nodc))

    # parabolic interpolation for sub-bin accuracy
    if 0 < peak_idx < n - 1:
        a, b, c = mags_nodc[peak_idx - 1:peak_idx + 2]
        denom = a - 2 * b + c
        delta = 0.5 * (a - c) / denom if abs(denom) > 1e-12 else 0.0
    else:
        delta = 0.0
    f_peak = freqs[peak_idx] + delta * fs / n

    # instantaneous frequency over time (block-averaged phase slope)
    block = 4096
    nblocks = x.size // block
    t_blocks = np.arange(nblocks) * block / fs
    f_blocks = np.empty(nblocks)
    for i in range(nblocks):
        seg_b = x[i * block:(i + 1) * block]
        prod = seg_b[1:] * np.conj(seg_b[:-1])
        f_blocks[i] = np.angle(np.mean(prod)) * fs / (2 * np.pi)

    # phase residual against the fitted tone: steps = sample slips.
    # Mix down by f_peak, then unwrap the remaining phase.
    k = np.arange(x.size)
    mixed = x * np.exp(-2j * np.pi * f_peak * k / fs)
    # decimate the residual phase to keep memory sane
    dec = 64
    ph = np.angle(mixed[: (x.size // dec) * dec].reshape(-1, dec).mean(axis=1))
    ph = np.unwrap(ph)
    t_ph = np.arange(ph.size) * dec / fs
    # remove any leftover linear trend (peak estimation error)
    coef = np.polyfit(t_ph, ph, 1)
    ph_res = ph - np.polyval(coef, t_ph)
    # slips in samples: dphase = 2*pi*f_tx*n_slip/fs when n_slip samples repeat/drop
    slip_samples = ph_res * fs / (2 * np.pi * tone_tx_hz)

    dstep = np.diff(slip_samples)
    step_thresh = 5.0  # samples
    step_idx = np.where(np.abs(dstep) > step_thresh)[0]
    # merge adjacent detections
    merged = []
    for i in step_idx:
        if merged and i - merged[-1][0] < 4:
            merged[-1] = (i, merged[-1][1] + dstep[i])
        else:
            merged.append((i, dstep[i]))
    step_locs = np.array([m[0] for m in merged], dtype=np.int64)
    step_sizes = np.array([m[1] for m in merged])

    return {
        "f_peak_hz": float(f_peak),
        "freqs": freqs,
        "mags_db": 20 * np.log10(mags + 1e-12),
        "t_blocks": t_blocks,
        "f_blocks": f_blocks,
        "t_ph": t_ph,
        "slip_samples": slip_samples,
        "step_t": t_ph[step_locs] if step_locs.size else np.array([]),
        "step_sizes": step_sizes,
        "skip_sec": skip_sec,
    }


def analyze_chirp(rx: np.ndarray, fs: float, bw: float, n_fft: int,
                  skip_sec: float = 1.0) -> dict:
    x = rx[int(skip_sec * fs):]

    t_s = n_fft / fs  # symbol duration at the DSP rate (oversampled full symbol)
    n = np.arange(n_fft)
    t = n / fs
    phase = 2.0 * np.pi * (-0.5 * bw * t + 0.5 * (bw / t_s) * t * t)
    down = np.exp(-1j * phase).astype(np.complex64)

    # coarse alignment: strongest single dechirp peak over one symbol of offsets
    best_off, best_mag = 0, 0.0
    for off in range(0, n_fft, 32):
        seg = x[off:off + n_fft]
        if seg.size < n_fft:
            break
        mag = float(np.max(np.abs(np.fft.fft(seg * down))))
        if mag > best_mag:
            best_mag, best_off = mag, off

    n_syms = (x.size - best_off) // n_fft
    n_syms = min(n_syms, 900)
    bins = np.empty(n_syms)
    mags = np.empty(n_syms)
    for s in range(n_syms):
        seg = x[best_off + s * n_fft: best_off + (s + 1) * n_fft]
        spec = np.abs(np.fft.fft(seg * down))
        b = int(np.argmax(spec))
        # sub-bin interpolation
        a, m, c = spec[(b - 1) % n_fft], spec[b], spec[(b + 1) % n_fft]
        denom = a - 2 * m + c
        delta = 0.5 * (a - c) / denom if abs(denom) > 1e-12 else 0.0
        bins[s] = b + delta
        mags[s] = m

    # unwrap the bin trajectory (mod n_fft) so drift is a clean line
    unwrapped = bins.copy()
    for i in range(1, n_syms):
        d = unwrapped[i] - unwrapped[i - 1]
        if d > n_fft / 2:
            unwrapped[i:] -= n_fft
        elif d < -n_fft / 2:
            unwrapped[i:] += n_fft

    sym_idx = np.arange(n_syms)
    slope, intercept = np.polyfit(sym_idx, unwrapped, 1)
    residual = unwrapped - (slope * sym_idx + intercept)

    # bins shift 0.5 bin per sample of timing offset at fs=2*bw
    bins_per_sample = (bw / t_s) / (fs / n_fft) / fs  # = bw*n_fft/(t_s*fs^2)
    samples_per_symbol_drift = slope / (bins_per_sample * fs) * fs  # samples/sym
    ppm = samples_per_symbol_drift / n_fft * 1e6

    # discontinuities: residual steps larger than 2 bins
    dres = np.abs(np.diff(residual))
    jump_idx = np.where(dres > 2.0)[0]

    return {
        "align_offset": best_off,
        "bins": bins,
        "unwrapped": unwrapped,
        "mags": mags,
        "slope_bins_per_sym": float(slope),
        "residual": residual,
        "jump_idx": jump_idx,
        "bins_per_sample": float(bins_per_sample * fs),
        "drift_samples_per_sym": float(samples_per_symbol_drift),
        "ppm": float(ppm),
        "skip_sec": skip_sec,
        "n_syms": n_syms,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", default="phy_dumps")
    parser.add_argument("--save", default=None)
    args = parser.parse_args()

    dump_dir = Path(args.dir)
    with (dump_dir / "probe_manifest.json").open() as f:
        manifest = json.load(f)

    fs = float(manifest["sample_rate_hz"])
    tone_hz = float(manifest["tone_hz"])
    bw = float(manifest["bandwidth_hz"])
    n_fft = int(manifest["fft_size"])

    rx_tone = load_iq(dump_dir / "probe_rx_tone.iq")
    rx_chirp = load_iq(dump_dir / "probe_rx_chirp.iq")

    tone = measure_tone(rx_tone, fs, tone_hz)
    chirp = analyze_chirp(rx_chirp, fs, bw, n_fft)

    rate_ratio = tone["f_peak_hz"] / tone_hz
    tone_ppm = (rate_ratio - 1.0) * 1e6

    print(f"Tone: TX {tone_hz/1e3:.1f} kHz -> RX peak {tone['f_peak_hz']/1e3:.4f} kHz")
    print(f"  frequency ratio {rate_ratio:.8f}  ({tone_ppm:+.1f} ppm)")
    n_steps = len(tone["step_sizes"])
    print(f"  phase steps >5 samples: {n_steps}")
    if n_steps:
        print(f"  step sizes (samples): {np.round(tone['step_sizes'], 1)[:20]}")
        if n_steps > 1:
            spacing = np.diff(tone["step_t"])
            print(f"  step spacing: median {np.median(spacing)*1e3:.2f} ms")
    print(f"Chirp: align {chirp['align_offset']}, {chirp['n_syms']} symbols")
    print(f"  bin drift {chirp['slope_bins_per_sym']:+.4f} bins/sym "
          f"= {chirp['drift_samples_per_sym']:+.4f} samples/sym ({chirp['ppm']:+.1f} ppm)")
    print(f"  residual jumps >2 bins: {len(chirp['jump_idx'])}")
    if len(chirp["jump_idx"]) > 1:
        spacing = np.diff(chirp["jump_idx"])
        print(f"  jump spacing (symbols): median {np.median(spacing):.1f}, "
              f"values {spacing[:20]}")

    fig, axes = plt.subplots(3, 2, figsize=(16, 13))

    ax = axes[0, 0]
    ax.plot(tone["freqs"] / 1e3, tone["mags_db"], linewidth=0.6)
    ax.axvline(tone_hz / 1e3, color="C2", linestyle="--", label=f"TX {tone_hz/1e3:.0f} kHz")
    ax.axvline(tone["f_peak_hz"] / 1e3, color="C3", linestyle=":",
               label=f"RX {tone['f_peak_hz']/1e3:.3f} kHz")
    ax.set_xlim(-fs / 2e3, fs / 2e3)
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title(f"Tone spectrum  ratio={rate_ratio:.6f} ({tone_ppm:+.0f} ppm)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    ax.plot(tone["t_ph"], tone["slip_samples"], linewidth=0.7)
    for t_step in tone["step_t"]:
        ax.axvline(t_step, color="C3", alpha=0.3, linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Accumulated slip (samples)")
    n_steps = len(tone["step_sizes"])
    med_step = float(np.median(tone["step_sizes"])) if n_steps else 0.0
    ax.set_title(f"Tone phase residual as sample slip  steps={n_steps} median={med_step:.1f}")
    ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ax.plot(chirp["bins"], ".", markersize=3)
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Peak bin (mod n_fft)")
    ax.set_title(f"Dechirped peak bin per symbol  slope={chirp['slope_bins_per_sym']:+.3f} b/sym")
    ax.grid(alpha=0.3)

    ax = axes[1, 1]
    ax.plot(chirp["residual"], linewidth=0.8)
    for j in chirp["jump_idx"]:
        ax.axvline(j, color="C3", alpha=0.3, linewidth=0.8)
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Residual (bins)")
    ax.set_title(f"Bin trajectory residual after linear fit  jumps={len(chirp['jump_idx'])}")
    ax.grid(alpha=0.3)

    ax = axes[2, 0]
    seg = rx_chirp[int(chirp["skip_sec"] * fs):]
    nper = 1024
    ax.specgram(seg[: int(fs)], NFFT=nper, Fs=fs, noverlap=nper // 2, cmap="viridis",
                scale="dB")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    ax.set_title("RX chirp spectrogram (1 s)")

    ax = axes[2, 1]
    ax.plot(20 * np.log10(chirp["mags"] + 1e-12), linewidth=0.8)
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Peak magnitude (dB)")
    ax.set_title("Dechirp peak magnitude per symbol")
    ax.grid(alpha=0.3)

    fig.suptitle(
        f"Digital loopback probe  fs={fs/1e6:.2f} MSps  "
        f"tone ratio {rate_ratio:.6f} ({tone_ppm:+.0f} ppm)  "
        f"chirp drift {chirp['ppm']:+.0f} ppm  jumps {len(chirp['jump_idx'])}",
        fontsize=11,
    )
    fig.tight_layout()

    save = Path(args.save) if args.save else dump_dir / "probe_dashboard.png"
    fig.savefig(save, dpi=130, bbox_inches="tight")
    print(f"Saved {save}")


if __name__ == "__main__":
    main()

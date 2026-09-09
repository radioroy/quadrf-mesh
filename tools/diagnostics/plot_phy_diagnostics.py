#!/usr/bin/env python3
"""Offline PHY diagnostic dashboard for IQ debug dumps and clock_coherence_test JSON."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class PeakResult:
    bin: int
    magnitude: float


@dataclass
class RxStats:
    rms: float
    peak: float
    rms_dbfs: float
    peak_dbfs: float
    clip_fraction: float


@dataclass
class SymbolResult:
    index: int
    region: str
    peak_bin: int
    magnitude: float
    expected_bin: int | None
    dist_expected: int | None


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 2 != 0:
        raise ValueError(f"{path}: expected interleaved I/Q pairs, got {raw.size} floats")
    return raw[0::2] + 1j * raw[1::2]


def load_manifest(dump_dir: Path) -> dict:
    manifest_path = dump_dir / "manifest.json"
    with manifest_path.open(encoding="utf-8") as f:
        return json.load(f)


def samples_per_symbol(bandwidth_hz: float, spreading_factor: int = 11) -> int:
    return 1 << spreading_factor


def symbol_duration_sec(bandwidth_hz: float, spreading_factor: int = 11) -> float:
    return samples_per_symbol(bandwidth_hz, spreading_factor) / bandwidth_hz


def generate_down_chirp(sample_rate_hz: float, bandwidth_hz: float, n_fft: int) -> np.ndarray:
    t_s = symbol_duration_sec(bandwidth_hz)
    n = np.arange(n_fft, dtype=np.float64)
    t = n / sample_rate_hz
    phase = 2.0 * math.pi * (-0.5 * bandwidth_hz * t + 0.5 * (bandwidth_hz / t_s) * t * t)
    up = np.exp(1j * phase)
    return np.conj(up).astype(np.complex64)


def dechirp(rx: np.ndarray, ref_down: np.ndarray) -> np.ndarray:
    return rx * ref_down


def demod_symbol(rx: np.ndarray, ref_down: np.ndarray, n_fft: int) -> PeakResult:
    work = dechirp(rx[:n_fft], ref_down)
    spectrum = np.fft.fft(work)
    mags_sq = np.abs(spectrum) ** 2
    peak_bin = int(np.argmax(mags_sq))
    return PeakResult(bin=peak_bin, magnitude=float(mags_sq[peak_bin]))


def bin_dist(a: int, b: int, n_fft: int) -> int:
    dist = abs(a - b)
    return min(dist, n_fft - dist)


def bin_close(a: int, b: int, n_fft: int, tolerance: int = 2) -> bool:
    dist = abs(a - b)
    return dist <= tolerance or dist >= n_fft - tolerance


def wrapped_dist0(peak_bin: int, n_fft: int) -> int:
    wrapped = peak_bin % n_fft
    return min(wrapped, n_fft - wrapped)


def compute_rx_stats(
    samples: np.ndarray, full_scale: float = 127.0, clip_threshold: float = 120.0
) -> RxStats:
    if samples.size == 0:
        return RxStats(0.0, 0.0, -100.0, -100.0, 0.0)

    mag_i = np.abs(samples.real)
    mag_q = np.abs(samples.imag)
    mag_sq = mag_i * mag_i + mag_q * mag_q
    rms = float(np.sqrt(np.mean(mag_sq)))
    peak = float(np.sqrt(np.max(mag_sq)))
    clips = int(np.sum((mag_i > clip_threshold) | (mag_q > clip_threshold)))
    return RxStats(
        rms=rms,
        peak=peak,
        rms_dbfs=20.0 * math.log10(rms / full_scale + 1e-12),
        peak_dbfs=20.0 * math.log10(peak / full_scale + 1e-12),
        clip_fraction=clips / samples.size,
    )


def parse_sync_bins(manifest: dict) -> tuple[int | None, int | None, str]:
    if "sync_word_bin0" in manifest and "sync_word_bin1" in manifest:
        b0 = int(manifest["sync_word_bin0"])
        b1 = int(manifest["sync_word_bin1"])
        return b0, b1, f"sync_bins={b0},{b1}"
    if "sync_word_bin" in manifest:
        b = int(manifest["sync_word_bin"])
        return None, b, f"sync_bin={b}"
    return None, None, "sync_bin=?"


def infer_frame_layout(tx_samples: int, n_fft: int, preamble_chirps: int) -> dict:
    symbols = tx_samples / n_fft
    has_sfd = symbols > preamble_chirps + 2 + 0.1
    preamble_samples = preamble_chirps * n_fft
    sync_samples = 2 * n_fft
    sfd_samples = tx_samples - preamble_samples - sync_samples if has_sfd else 0
    sfd_symbols = 2 if has_sfd else 0
    return {
        "has_sfd": has_sfd,
        "preamble_samples": preamble_samples,
        "sync_samples": sync_samples,
        "sfd_samples": sfd_samples,
        "sfd_symbols": sfd_symbols,
        "total_symbols": preamble_chirps + 2 + sfd_symbols,
    }


def alignment_scores(
    captured: np.ndarray, ref_down: np.ndarray, n_fft: int, preamble_count: int
) -> tuple[list[int], list[float], int, int]:
    max_offset = min(n_fft, captured.size - preamble_count * n_fft)
    offsets: list[int] = []
    scores: list[float] = []
    best_offset = 0
    best_hits = 0
    best_mag_sum = 0.0

    for offset in range(0, max_offset, 8):
        hits = 0
        mag_sum = 0.0
        valid = True
        for sym in range(preamble_count):
            pos = offset + sym * n_fft
            if pos + n_fft > captured.size:
                valid = False
                break
            peak = demod_symbol(captured[pos:], ref_down, n_fft)
            if wrapped_dist0(peak.bin, n_fft) <= 16:
                hits += 1
            mag_sum += peak.magnitude
        if not valid:
            continue
        score = hits * 1e12 + mag_sum
        offsets.append(offset)
        scores.append(score)
        if hits > best_hits or (hits == best_hits and mag_sum > best_mag_sum):
            best_hits = hits
            best_mag_sum = mag_sum
            best_offset = offset

    return offsets, scores, best_offset, best_hits


def demod_symbol_timeline(
    captured: np.ndarray,
    ref_down: np.ndarray,
    n_fft: int,
    align_offset: int,
    preamble_chirps: int,
    sync_bin0: int | None,
    sync_bin1: int | None,
    sfd_symbols: int,
) -> list[SymbolResult]:
    results: list[SymbolResult] = []
    sym_idx = 0

    for i in range(preamble_chirps):
        pos = align_offset + i * n_fft
        if pos + n_fft > captured.size:
            break
        peak = demod_symbol(captured[pos:], ref_down, n_fft)
        results.append(
            SymbolResult(
                index=sym_idx,
                region="preamble",
                peak_bin=peak.bin,
                magnitude=peak.magnitude,
                expected_bin=0,
                dist_expected=wrapped_dist0(peak.bin, n_fft),
            )
        )
        sym_idx += 1

    sync_expected = [sync_bin0, sync_bin1]
    for i, expected in enumerate(sync_expected):
        pos = align_offset + (preamble_chirps + i) * n_fft
        if pos + n_fft > captured.size:
            break
        peak = demod_symbol(captured[pos:], ref_down, n_fft)
        dist = bin_dist(peak.bin % n_fft, expected, n_fft) if expected is not None else None
        results.append(
            SymbolResult(
                index=sym_idx,
                region="sync",
                peak_bin=peak.bin,
                magnitude=peak.magnitude,
                expected_bin=expected,
                dist_expected=dist,
            )
        )
        sym_idx += 1

    for i in range(sfd_symbols):
        pos = align_offset + (preamble_chirps + 2 + i) * n_fft
        if pos + n_fft > captured.size:
            break
        peak = demod_symbol(captured[pos:], ref_down, n_fft)
        results.append(
            SymbolResult(
                index=sym_idx,
                region="sfd",
                peak_bin=peak.bin,
                magnitude=peak.magnitude,
                expected_bin=None,
                dist_expected=None,
            )
        )
        sym_idx += 1

    return results


def global_scan(
    captured: np.ndarray,
    ref_down: np.ndarray,
    n_fft: int,
    sync_bin0: int | None,
    sync_bin1: int | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    step = n_fft // 8
    offsets: list[int] = []
    preamble_hits: list[int] = []
    sync_hits: list[int] = []
    total_preamble = 0
    total_sync = 0

    for offset in range(0, captured.size - n_fft + 1, step):
        peak = demod_symbol(captured[offset:], ref_down, n_fft)
        wrapped = peak.bin % n_fft
        p_hit = 1 if wrapped_dist0(wrapped, n_fft) <= 16 else 0
        s_hit = 0
        if sync_bin0 is not None and bin_close(wrapped, sync_bin0, n_fft):
            s_hit = 1
        elif sync_bin1 is not None and bin_close(wrapped, sync_bin1, n_fft):
            s_hit = 1
        offsets.append(offset)
        preamble_hits.append(p_hit)
        sync_hits.append(s_hit)
        total_preamble += p_hit
        total_sync += s_hit

    return (
        np.asarray(offsets, dtype=np.int64),
        np.asarray(preamble_hits, dtype=np.int8),
        np.asarray(sync_hits, dtype=np.int8),
        total_preamble,
        total_sync,
    )


def evaluate_pass_fail(
    symbols: list[SymbolResult],
    n_fft: int,
    sync_bin0: int | None,
    sync_bin1: int | None,
    stats: RxStats,
) -> tuple[bool, bool, bool]:
    preamble_syms = [s for s in symbols if s.region == "preamble"]
    sync_syms = [s for s in symbols if s.region == "sync"]

    preamble_mag = max((s.magnitude for s in preamble_syms), default=0.0)
    preamble_hits = sum(1 for s in preamble_syms if s.dist_expected is not None and s.dist_expected <= 16)

    sync_ok = False
    if len(sync_syms) >= 2 and sync_bin0 is not None and sync_bin1 is not None:
        s0 = sync_syms[0].peak_bin % n_fft
        s1 = sync_syms[1].peak_bin % n_fft
        sync_ok = bin_close(s0, sync_bin0, n_fft) and bin_close(s1, sync_bin1, n_fft)

    preamble_ok = preamble_mag > 1e-5 and preamble_hits >= 12
    clip_ok = stats.clip_fraction < 0.05
    return preamble_ok, sync_ok, clip_ok


def _welch_numpy(samples: np.ndarray, fs: float, nperseg: int) -> tuple[np.ndarray, np.ndarray]:
    step = max(1, nperseg // 2)
    window = np.hanning(nperseg)
    scale = fs * float(np.sum(window**2))
    accum = np.zeros(nperseg, dtype=np.float64)
    count = 0
    for start in range(0, samples.size - nperseg + 1, step):
        seg = samples[start : start + nperseg] * window
        accum += np.abs(np.fft.fft(seg)) ** 2
        count += 1
    psd = accum / (count * scale) if count > 0 else accum
    freqs = np.fft.fftshift(np.fft.fftfreq(nperseg, d=1.0 / fs))
    return freqs, np.fft.fftshift(psd)


def welch_psd_db(
    samples: np.ndarray, fs: float, nperseg: int | None = None
) -> tuple[np.ndarray, np.ndarray]:
    n = samples.size
    if nperseg is None:
        nperseg = min(4096, n)
    nperseg = max(16, min(nperseg, n))

    try:
        from scipy.signal import welch

        freqs, psd = welch(samples, fs=fs, nperseg=nperseg, return_onesided=False)
        freqs = np.fft.fftshift(freqs)
        psd = np.fft.fftshift(psd)
    except ImportError:
        freqs, psd = _welch_numpy(samples, fs, nperseg)

    return freqs, 10.0 * np.log10(psd + 1e-30)


def compute_snr_db(mags: np.ndarray, guard: int = 8) -> tuple[int, float, float]:
    peak_bin = int(np.argmax(mags))
    peak_pwr = float(mags[peak_bin] ** 2)
    if peak_bin >= guard:
        left = mags[: peak_bin - guard]
    else:
        left = mags[:0]
    if peak_bin + guard + 1 < mags.size:
        right = mags[peak_bin + guard + 1 :]
    else:
        right = mags[0:0]
    noise_bins = np.concatenate([left, right])
    if noise_bins.size == 0:
        noise_floor = float(np.median(mags))
    else:
        noise_floor = float(np.median(noise_bins))
    snr_db = 10.0 * np.log10(peak_pwr / (noise_floor**2 + 1e-30))
    return peak_bin, snr_db, noise_floor


def plot_fft_stem(
    ax: plt.Axes,
    spectrum: np.ndarray,
    title: str,
    ref_bins: list[tuple[int, str]],
    peak_bin: int | None = None,
    noise_floor: float | None = None,
) -> None:
    mags = np.abs(spectrum)
    bins = np.arange(mags.size)
    markerline, stemlines, _ = ax.stem(bins, mags, linefmt="C0-", markerfmt="C0o", basefmt=" ")
    plt.setp(stemlines, linewidth=0.8, alpha=0.7)
    plt.setp(markerline, markersize=2)

    if peak_bin is None:
        peak_bin = int(np.argmax(mags))
    if noise_floor is None:
        _, _, noise_floor = compute_snr_db(mags)

    for ref_bin, label in ref_bins:
        ax.axvline(ref_bin, color="gray", linestyle="--", linewidth=1, alpha=0.6, label=label)
    ax.axvline(peak_bin, color="C3", linestyle="-", linewidth=1.2, label=f"peak {peak_bin}")
    ax.axhline(noise_floor, color="C2", linestyle=":", linewidth=1, label=f"noise {noise_floor:.2e}")
    ax.set_xlabel("FFT Bin")
    ax.set_ylabel("|X[k]|")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)


def load_clock_report(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def plot_clock_coherence(report_path: Path, save_path: Path | None, show: bool) -> None:
    """Plot preamble bin drift and fs estimates from clock_coherence_test JSON."""
    report = load_clock_report(report_path)

    phase4 = report.get("phase4", {})
    bins = phase4.get("preamble_peak_bins", [])
    slope = float(phase4.get("preamble_slope_bins_per_sym", 0.0))
    intercept = float(phase4.get("preamble_intercept", 0.0))
    fs_nominal = float(report.get("fs_nominal_hz", 1e6))
    fs_ota = float(report.get("phase3", {}).get("fs_ota_hz", fs_nominal))
    fs_css = float(phase4.get("fs_css_hz", fs_nominal))
    fs_wall = float(report.get("phase2", {}).get("fs_wall_hz", fs_nominal))
    verdict = report.get("verdict", "?")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax_bins = axes[0]
    if bins:
        xs = np.arange(len(bins))
        ys = np.array(bins, dtype=float)
        ax_bins.scatter(xs, ys, c="C0", s=50, zorder=3, label="measured bins")
        fit_y = intercept + slope * xs
        ax_bins.plot(xs, fit_y, "C3--", linewidth=1.5, label=f"fit slope={slope:.3f} bin/sym")
        ax_bins.axhline(0, color="gray", linestyle=":", linewidth=1, alpha=0.6)
        ax_bins.set_xlabel("Preamble symbol index")
        ax_bins.set_ylabel("Peak FFT bin")
        ax_bins.set_title("OTA CSS preamble bin drift (Phase 4)")
        ax_bins.legend(loc="best", fontsize=8)
        ax_bins.grid(True, alpha=0.3)
    else:
        ax_bins.text(
            0.5,
            0.5,
            "No preamble_peak_bins in report",
            ha="center",
            va="center",
            transform=ax_bins.transAxes,
        )
        ax_bins.set_title("Preamble bin drift (no data)")

    ax_table = axes[1]
    ax_table.axis("off")
    rows = [
        ("Verdict", verdict),
        ("fs_nominal", f"{fs_nominal / 1e6:.6f} MSps"),
        ("fs_wall", f"{fs_wall / 1e6:.6f} MSps"),
        ("fs_ota", f"{fs_ota / 1e6:.6f} MSps"),
        ("fs_css", f"{fs_css / 1e6:.6f} MSps"),
        (
            "ppm_wall",
            f"{report.get('phase2', {}).get('ppm_wall', 0):.1f}",
        ),
        (
            "ppm_ota",
            f"{report.get('phase3', {}).get('ppm_ota', 0):.1f}",
        ),
        (
            "ppm_css",
            f"{phase4.get('ppm_css', 0):.1f}",
        ),
        ("preamble_slope", f"{slope:.4f} bin/sym"),
        (
            "preamble_hits",
            f"{phase4.get('preamble_hits', 0)}/16",
        ),
    ]
    table_text = "\n".join(f"{k:16s} {v}" for k, v in rows)
    ax_table.text(
        0.05,
        0.95,
        table_text,
        transform=ax_table.transAxes,
        fontsize=10,
        verticalalignment="top",
        fontfamily="monospace",
    )
    ax_table.set_title("Clock coherence estimates")

    fig.suptitle(f"Clock Coherence Test  verdict={verdict}", fontsize=11)
    fig.tight_layout()

    if save_path is None:
        save_path = report_path.parent / "clock_coherence.png"
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved clock coherence plot to {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_dashboard(dump_dir: Path, save_path: Path | None, show: bool) -> None:
    manifest = load_manifest(dump_dir)

    tx_path = dump_dir / "tx_preamble_sync_sfd.iq"
    if not tx_path.exists():
        tx_path = dump_dir / "tx_preamble_sync.iq"
    tx = load_iq(tx_path)
    rx = load_iq(dump_dir / "rx_raw.iq")

    sample_rate = float(manifest["sample_rate_hz"])
    bandwidth = float(manifest["bandwidth_hz"])
    fft_size = int(manifest["fft_size"])
    preamble_chirps = int(manifest.get("preamble_chirps", 16))
    align_offset = int(manifest["align_offset"])
    tx_samples = int(manifest.get("tx_samples", tx.size))
    sync_bin0, sync_bin1, sync_label = parse_sync_bins(manifest)

    ref_down = generate_down_chirp(sample_rate, bandwidth, fft_size)
    frame = infer_frame_layout(tx_samples, fft_size, preamble_chirps)
    stats = compute_rx_stats(rx)

    align_offsets, align_scores, recomputed_align, recomputed_hits = alignment_scores(
        rx, ref_down, fft_size, preamble_chirps
    )
    symbols = demod_symbol_timeline(
        rx,
        ref_down,
        fft_size,
        align_offset,
        preamble_chirps,
        sync_bin0,
        sync_bin1,
        frame["sfd_symbols"],
    )
    scan_offsets, scan_preamble, scan_sync, global_preamble_hits, global_sync_hits = global_scan(
        rx, ref_down, fft_size, sync_bin0, sync_bin1
    )
    preamble_ok, sync_ok, clip_ok = evaluate_pass_fail(
        symbols,
        fft_size,
        sync_bin0,
        sync_bin1,
        stats,
    )

    dechirped = dechirp(rx[align_offset : align_offset + fft_size], ref_down)
    preamble_fft = np.fft.fft(dechirped)
    preamble_mags = np.abs(preamble_fft)
    peak_bin, snr_db, noise_floor = compute_snr_db(preamble_mags)

    sync_fft: list[np.ndarray] = []
    for i in range(2):
        pos = align_offset + (preamble_chirps + i) * fft_size
        if pos + fft_size <= rx.size:
            work = dechirp(rx[pos : pos + fft_size], ref_down)
            sync_fft.append(np.fft.fft(work))
        else:
            sync_fft.append(np.zeros(fft_size, dtype=np.complex64))

    rx_segment_end = align_offset + tx.size
    if rx_segment_end > rx.size:
        raise ValueError(
            f"aligned RX window [{align_offset}:{rx_segment_end}] exceeds capture ({rx.size} samples)"
        )
    rx_aligned = rx[align_offset:rx_segment_end]

    pass_label = "PASS" if preamble_ok and sync_ok and clip_ok else "WARN"
    align_warn = "" if recomputed_align == align_offset else f" (recomputed={recomputed_align})"

    fig = plt.figure(figsize=(18, 18))
    gs = fig.add_gridspec(7, 2, hspace=0.45, wspace=0.28)

    # Row 0: TX vs RX
    ax_tx_rx = fig.add_subplot(gs[0, 0])
    tx_t = np.arange(tx.size) / sample_rate * 1e3
    rx_t = (np.arange(rx_aligned.size) + align_offset) / sample_rate * 1e3
    ax_tx_rx.plot(tx_t, np.abs(tx), label="TX |x|", alpha=0.85)
    ax_tx_rx.plot(rx_t, np.abs(rx_aligned), label="RX |x| (aligned)", alpha=0.75)

    pre_end = frame["preamble_samples"] / sample_rate * 1e3
    sync_end = (frame["preamble_samples"] + frame["sync_samples"]) / sample_rate * 1e3
    tx_end = tx.size / sample_rate * 1e3
    ax_tx_rx.axvspan(0, pre_end, alpha=0.08, color="C0", label="preamble")
    ax_tx_rx.axvspan(pre_end, sync_end, alpha=0.08, color="C1", label="sync")
    if frame["has_sfd"]:
        ax_tx_rx.axvspan(sync_end, tx_end, alpha=0.08, color="C2", label="SFD")
    align_ms = align_offset / sample_rate * 1e3
    ax_tx_rx.axvline(align_ms, color="C3", linestyle="--", linewidth=1, label=f"align {align_offset}")
    ax_tx_rx.set_xlabel("Time (ms)")
    ax_tx_rx.set_ylabel("Magnitude")
    ax_tx_rx.set_title(
        f"TX vs RX  RMS={stats.rms_dbfs:.1f} dBFS  clip={stats.clip_fraction * 100:.2f}%"
    )
    ax_tx_rx.legend(loc="upper right", fontsize=7)
    ax_tx_rx.grid(True, alpha=0.3)

    # Row 0: spectrogram
    ax_spec = fig.add_subplot(gs[0, 1])
    nperseg = min(fft_size, max(256, fft_size // 4))
    _, _, _, im = ax_spec.specgram(
        rx.real,
        NFFT=nperseg,
        Fs=sample_rate,
        noverlap=nperseg // 2,
        cmap="viridis",
        scale="dB",
    )
    ax_spec.set_xlabel("Time (s)")
    ax_spec.set_ylabel("Frequency (Hz)")
    ax_spec.set_title("RX Raw Spectrogram")
    fig.colorbar(im, ax=ax_spec, label="Power (dB)")

    # Row 1: alignment scores
    ax_align = fig.add_subplot(gs[1, :])
    ax_align.plot(align_offsets, align_scores, linewidth=1.0, color="C0")
    ax_align.axvline(align_offset, color="C3", linestyle="--", linewidth=1.2, label=f"manifest {align_offset}")
    if recomputed_align != align_offset:
        ax_align.axvline(
            recomputed_align, color="C4", linestyle=":", linewidth=1.2, label=f"recomputed {recomputed_align}"
        )
    ax_align.set_xlabel("RX offset (samples)")
    ax_align.set_ylabel("Alignment score")
    ax_align.set_title(
        f"Alignment search (offset={recomputed_align}, hits={recomputed_hits}/{preamble_chirps})"
    )
    ax_align.legend(loc="upper right", fontsize=8)
    ax_align.grid(True, alpha=0.3)

    # Row 2: symbol peak-bin timeline
    ax_timeline = fig.add_subplot(gs[2, :])
    region_colors = {"preamble": "C0", "sync": "C1", "sfd": "C2"}
    for sym in symbols:
        color = region_colors.get(sym.region, "C3")
        ax_timeline.scatter(sym.index, sym.peak_bin % fft_size, c=color, s=40, zorder=3)
    if symbols:
        ax_timeline.plot(
            [s.index for s in symbols],
            [s.peak_bin % fft_size for s in symbols],
            color="gray",
            linewidth=0.8,
            alpha=0.5,
        )
    ax_timeline.axhline(0, color="gray", linestyle="--", linewidth=1, alpha=0.6, label="bin 0")
    if sync_bin0 is not None:
        ax_timeline.axhline(sync_bin0, color="C1", linestyle=":", linewidth=1, alpha=0.8, label=f"sync0 {sync_bin0}")
    if sync_bin1 is not None:
        ax_timeline.axhline(sync_bin1, color="C4", linestyle=":", linewidth=1, alpha=0.8, label=f"sync1 {sync_bin1}")
    for sym in symbols:
        if sym.dist_expected is not None and sym.expected_bin is not None:
            ax_timeline.annotate(
                f"d={sym.dist_expected}",
                (sym.index, sym.peak_bin % fft_size),
                textcoords="offset points",
                xytext=(0, 6),
                fontsize=6,
                ha="center",
            )
    ax_timeline.set_xlabel("Symbol index (from align)")
    ax_timeline.set_ylabel("Peak FFT bin")
    ax_timeline.set_title("Per-symbol peak bins (preamble=blue, sync=orange, SFD=green)")
    ax_timeline.legend(loc="upper right", fontsize=7, ncol=2)
    ax_timeline.grid(True, alpha=0.3)

    # Row 3: dechirped preamble sym 0
    ax_dechirp = fig.add_subplot(gs[3, 0])
    n = np.arange(dechirped.size)
    ax_dechirp.plot(n, dechirped.real, label="I", alpha=0.8)
    ax_dechirp.plot(n, dechirped.imag, label="Q", alpha=0.8)
    ax_dechirp.plot(n, np.abs(dechirped), label="|x|", linewidth=1.5)
    ax_dechirp.set_xlabel("Sample")
    ax_dechirp.set_ylabel("Amplitude")
    ax_dechirp.set_title("De-chirped Preamble Symbol 0")
    ax_dechirp.legend(loc="upper right", fontsize=8)
    ax_dechirp.grid(True, alpha=0.3)

    # Row 3: preamble FFT
    ax_fft = fig.add_subplot(gs[3, 1])
    ref_bins: list[tuple[int, str]] = [(0, "bin 0")]
    if sync_bin0 is not None:
        ref_bins.append((sync_bin0, f"sync0 {sync_bin0}"))
    if sync_bin1 is not None:
        ref_bins.append((sync_bin1, f"sync1 {sync_bin1}"))
    plot_fft_stem(
        ax_fft,
        preamble_fft,
        f"Preamble sym0 FFT  SNR={snr_db:.1f} dB",
        ref_bins,
        peak_bin=peak_bin,
        noise_floor=noise_floor,
    )

    # Row 4: sync FFTs
    sync_expected = [sync_bin0, sync_bin1]
    for col, (spectrum, expected) in enumerate(zip(sync_fft, sync_expected)):
        ax_sync = fig.add_subplot(gs[4, col])
        sym = symbols[preamble_chirps + col] if len(symbols) > preamble_chirps + col else None
        peak = sym.peak_bin if sym else int(np.argmax(np.abs(spectrum)))
        dist_str = f"  d={sym.dist_expected}" if sym and sym.dist_expected is not None else ""
        refs: list[tuple[int, str]] = [(0, "bin 0")]
        if expected is not None:
            refs.append((expected, f"expected {expected}"))
        plot_fft_stem(
            ax_sync,
            spectrum,
            f"Sync sym{col}  peak={peak % fft_size}{dist_str}",
            refs,
            peak_bin=peak % fft_size,
        )

    # Row 5: global scan hits
    ax_scan = fig.add_subplot(gs[5, :])
    scan_t = scan_offsets / sample_rate
    ax_scan.scatter(
        scan_t[scan_preamble == 1],
        np.ones(np.sum(scan_preamble == 1)),
        c="C0",
        s=12,
        marker="|",
        label=f"preamble hits ({global_preamble_hits})",
    )
    ax_scan.scatter(
        scan_t[scan_sync == 1],
        np.ones(np.sum(scan_sync == 1)) * 1.15,
        c="C1",
        s=12,
        marker="|",
        label=f"sync hits ({global_sync_hits})",
    )
    ax_scan.axvline(align_offset / sample_rate, color="C3", linestyle="--", linewidth=1, label="align")
    ax_scan.set_xlabel("Time (s)")
    ax_scan.set_yticks([1.0, 1.15])
    ax_scan.set_yticklabels(["preamble", "sync"])
    ax_scan.set_title("Global sliding-window detection (informational only, not used for PASS/WARN)")
    ax_scan.legend(loc="upper right", fontsize=8)
    ax_scan.grid(True, alpha=0.3, axis="x")
    ax_scan.set_ylim(0.85, 1.35)

    # Row 6: de-chirped preamble waterfall + Welch PSD
    ax_dechirp_full = fig.add_subplot(gs[6, 0])
    dechirped_full_path = dump_dir / "rx_dechirped_preamble_full.iq"
    if dechirped_full_path.exists():
        dechirped_full = load_iq(dechirped_full_path)
        _, _, _, im_full = ax_dechirp_full.specgram(
            dechirped_full.real,
            NFFT=fft_size,
            Fs=sample_rate,
            noverlap=fft_size // 2,
            cmap="viridis",
            scale="dB",
        )
        ax_dechirp_full.set_xlabel("Time (s)")
        ax_dechirp_full.set_ylabel("Frequency (Hz)")
        ax_dechirp_full.set_title(
            "De-chirped Preamble Waterfall "
            "(horizontal=stable bin; staircase=dropped samples; slope=BW/SR mismatch)"
        )
        fig.colorbar(im_full, ax=ax_dechirp_full, label="Power (dB)")
    else:
        ax_dechirp_full.text(
            0.5,
            0.5,
            "rx_dechirped_preamble_full.iq not found",
            ha="center",
            va="center",
            transform=ax_dechirp_full.transAxes,
        )
        ax_dechirp_full.set_title("De-chirped Preamble Waterfall (missing dump)")

    ax_welch = fig.add_subplot(gs[6, 1])
    welch_nperseg = min(fft_size, 4096, rx.size)
    welch_freqs, welch_db = welch_psd_db(rx, sample_rate, nperseg=welch_nperseg)
    dc_idx = int(np.argmin(np.abs(welch_freqs)))
    dc_db = float(welch_db[dc_idx])
    ax_welch.plot(welch_freqs / 1e3, welch_db, linewidth=0.8, color="C0")
    ax_welch.axvline(0.0, color="C3", linestyle="--", linewidth=1, alpha=0.8, label="DC (0 Hz)")
    ax_welch.set_xlabel("Frequency (kHz)")
    ax_welch.set_ylabel("Power (dB)")
    ax_welch.set_title(f"RX Raw Welch PSD  LO leakage at DC: {dc_db:.1f} dB")
    ax_welch.legend(loc="upper right", fontsize=8)
    ax_welch.grid(True, alpha=0.3)

    fig.suptitle(
        f"PHY Diagnostics  {pass_label}  SR={sample_rate / 1e6:.2f} MSps  "
        f"align={align_offset}{align_warn}  {sync_label}  "
        f"preamble_ok={preamble_ok} sync_ok={sync_ok} clip_ok={clip_ok}  "
        f"peak={peak_bin}  SNR={snr_db:.1f} dB",
        fontsize=10,
    )

    if save_path is None:
        save_path = dump_dir / "phy_dashboard.png"
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved dashboard to {save_path}")
    print(
        f"  {pass_label}: preamble_ok={preamble_ok} sync_ok={sync_ok} clip_ok={clip_ok}  "
        f"(global scan: preamble={global_preamble_hits} sync={global_sync_hits}, informational only)"
    )
    if recomputed_align != align_offset:
        print(f"  Note: recomputed align={recomputed_align} differs from manifest {align_offset}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    clock_report = dump_dir / "clock_report.json"
    if clock_report.exists():
        plot_clock_coherence(clock_report, dump_dir / "clock_coherence.png", show=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot PHY debug dumps and clock_coherence JSON")
    parser.add_argument("--dir", dest="dump_dir", default="phy_dumps", help="Dump directory")
    parser.add_argument(
        "--save",
        dest="save_path",
        default=None,
        help="Output PNG path (default: <dir>/phy_dashboard.png)",
    )
    parser.add_argument("--show", action="store_true", help="Show interactive plot window")
    parser.add_argument(
        "--clock-report",
        dest="clock_report",
        default=None,
        help="Plot clock_coherence_test JSON only (path to clock_report.json)",
    )
    parser.add_argument(
        "--clock-save",
        dest="clock_save",
        default=None,
        help="Output PNG for --clock-report (default: alongside JSON)",
    )
    args = parser.parse_args()

    if args.clock_report:
        plot_clock_coherence(
            Path(args.clock_report),
            Path(args.clock_save) if args.clock_save else None,
            args.show,
        )
        return

    dump_dir = Path(args.dump_dir)
    save_path = Path(args.save_path) if args.save_path else None
    plot_dashboard(dump_dir, save_path, args.show)


if __name__ == "__main__":
    main()

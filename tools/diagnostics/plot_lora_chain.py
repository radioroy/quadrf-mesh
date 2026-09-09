#!/usr/bin/env python3
"""Full LoRa TX/RX signal-chain visualization for lora_loopback dumps.

Reconstructs every coding stage offline, demodulates RX IQ through the same
dechirp→FFT→fold kernel used in C++, and writes a MATLAB-style dashboard plus
individual PNGs for time, frequency, IQ, and chip domains.
"""

from __future__ import annotations

import argparse
import json
import math
import textwrap
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from scipy import signal as sp_signal

# ---------------------------------------------------------------------------
# Whitening sequence (matches src/lora/coding.cpp)
# ---------------------------------------------------------------------------
K_WHITENING = np.array(
    [
        0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78, 0xF1,
        0xE3, 0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11,
        0x23, 0x47, 0x8E, 0x1C, 0x38, 0x71, 0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C,
        0xB8, 0x70, 0xE0, 0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49,
        0x93, 0x26, 0x4D, 0x9B, 0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90, 0x20, 0x41, 0x82,
        0x05, 0x0A, 0x15, 0x2B, 0x56, 0xAD, 0x5B, 0xB6, 0x6D, 0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59,
        0xB2, 0x65, 0xCB, 0x96, 0x2C, 0x58, 0xB0, 0x61, 0xC3, 0x87, 0x0F, 0x1F, 0x3E, 0x7D, 0xFB,
        0xF6, 0xED, 0xDB, 0xB7, 0x6F, 0xDE, 0xBD, 0x7A, 0xF5, 0xEB, 0xD7, 0xAE, 0x5D, 0xBA, 0x74,
        0xE8, 0xD1, 0xA2, 0x44, 0x88, 0x10, 0x21, 0x43, 0x86, 0x0D, 0x1B, 0x36, 0x6C, 0xD8, 0xB1,
        0x63, 0xC7, 0x8F, 0x1E, 0x3C, 0x79, 0xF3, 0xE7, 0xCE, 0x9C, 0x39, 0x73, 0xE6, 0xCC, 0x98,
        0x31, 0x62, 0xC5, 0x8B, 0x16, 0x2D, 0x5A, 0xB4, 0x69, 0xD2, 0xA4, 0x48, 0x91, 0x22, 0x45,
        0x8A, 0x14, 0x29, 0x52, 0xA5, 0x4A, 0x95, 0x2A, 0x54, 0xA9, 0x53, 0xA7, 0x4E, 0x9D, 0x3B,
        0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xEC, 0xD9, 0xB3, 0x67, 0xCF, 0x9E, 0x3D, 0x7B, 0xF7, 0xEF,
        0xDF, 0xBF, 0x7E, 0xFD, 0xFA, 0xF4, 0xE9, 0xD3, 0xA6, 0x4C, 0x99, 0x33, 0x66, 0xCD, 0x9A,
        0x35, 0x6A, 0xD4, 0xA8, 0x51, 0xA3, 0x46, 0x8C, 0x18, 0x30, 0x60, 0xC1, 0x83, 0x07, 0x0E,
        0x1D, 0x3A, 0x75, 0xEA, 0xD5, 0xAA, 0x55, 0xAB, 0x57, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2,
        0xE5, 0xCA, 0x94, 0x28, 0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F, 0x9F, 0x3F, 0x7F,
    ],
    dtype=np.uint8,
)

# Stage captions: expected appearance + common artifacts
STAGE_NOTES = {
    "tx_payload": (
        "Expected: deterministic byte sequence i*37+11. Flat-looking stem plot; "
        "no structure beyond the arithmetic pattern."
    ),
    "tx_whitened": (
        "Expected: payload XOR whitening LFSR — looks pseudo-random, roughly uniform "
        "over 0..255. Artifact: if whitening is skipped, payload pattern remains visible."
    ),
    "tx_nibbles": (
        "Expected: 5 header nibbles, then whitened low/high nibbles, then 4 CRC nibbles. "
        "Header encodes length/CR/CRC flag + checksum."
    ),
    "tx_hamming": (
        "Expected: each nibble expands to a (4+CR)-bit codeword. Header uses CR=4 (8-bit). "
        "Artifact: wrong CR yields decode failures even with clean chips."
    ),
    "tx_interleave": (
        "Expected: diagonal interleaver spreads codeword bits across symbols. "
        "Header/LDRO blocks are reduced-rate (sf-2) with parity padding."
    ),
    "tx_chips": (
        "Expected: Gray-decoded chip indices in 0..2^SF-1. First 8 symbols are the "
        "header block. Artifact: off-by-one Gray map shifts all peaks by 1 chip."
    ),
    "tx_iq_time": (
        "Expected: constant envelope |x|≈amplitude across preamble/sync/SFD/data; "
        "silence gaps at 0. Artifact: amplitude droop, clipping, or DC offset."
    ),
    "tx_iq_spec": (
        "Expected: linear FM sweeps filling ±BW/2; preamble = stacked upchirps; "
        "SFD = downchirps (opposite slope). Artifact: missing SFD slope flip, "
        "or bandwidth narrower than configured."
    ),
    "tx_iq_const": (
        "Expected: unit-circle ring at radius≈amplitude (constant envelope CSS). "
        "Artifact: elliptical ring → I/Q imbalance; filled disk → noise/AGC; "
        "spiral → amplitude ramp."
    ),
    "tx_iq_psd": (
        "Expected: roughly flat PSD across the LoRa bandwidth, steep skirts outside. "
        "Artifact: strong DC spike, spurs, or energy outside ±BW/2."
    ),
    "rx_time": (
        "Expected: repeating frame envelopes matching TX pattern period. "
        "Digital: clean, high SNR. OTA: lower RMS, possible AGC settle on warmup. "
        "Artifact: clipping (>few %), dropouts, or missing frames."
    ),
    "rx_spec": (
        "Expected: same chirp waterfall as TX, possibly with CFO shift and rate skew. "
        "OTA: ~1k ppm rate offset typical on this platform; chirps slowly walk. "
        "Artifact: smeared chirps (timing), horizontal lines (CW interferers)."
    ),
    "rx_iq": (
        "Expected: digital ≈ tight ring; OTA = thicker noisy ring / cloud. "
        "Artifact: DC blob at origin, quadrant imbalance, clipping square."
    ),
    "rx_dechirp": (
        "Expected: after multiply by down-chirp ref, a tone at the chip frequency. "
        "Preamble → near-DC tone. Artifact: chirp residual (bad ref/BW), beating."
    ),
    "rx_fft": (
        "Expected: single sharp peak in folded chip domain at the symbol's chip index. "
        "Preamble peaks near 0 (modulo). Artifact: split peaks (timing), raised floor "
        "(noise), secondary images (OS fold incomplete)."
    ),
    "rx_preamble_traj": (
        "Expected: nearly flat or slowly drifting chip trajectory across preamble. "
        "Slope ≈ rate error in chips/symbol. Artifact: jumps at sync/SFD, outliers."
    ),
    "rx_sync": (
        "Expected: sync peaks at syncChip0/1 = ((nibble)<<3). For 0x2B → chips 128, 88 "
        "(SF-dependent shift). Artifact: wrong sync word, large CFO wrapping."
    ),
    "rx_sfd": (
        "Expected: strong peaks with downchirp reference; used with upchirp peaks to "
        "solve CFO=(u+d)/2 and timing. Artifact: weak SFD → bad CFO/timing."
    ),
    "rx_data_chips": (
        "Expected: RX chip decisions overlay TX chips; errors should be rare/isolated. "
        "Artifact: burst errors near DSI boundaries, systematic ±1 chip (ref drift)."
    ),
    "rx_decode": (
        "Expected: header valid, CRC OK, payload matches i*37+11. "
        "Artifact: header fail with good SNR → sync/timing; CRC fail → chip errors."
    ),
}


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.grid": True,
            "grid.alpha": 0.3,
            "grid.linestyle": ":",
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 7,
            "figure.dpi": 120,
            "savefig.dpi": 160,
            "savefig.bbox": "tight",
            "lines.linewidth": 1.0,
        }
    )


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 2 != 0:
        raise ValueError(f"{path}: odd float count {raw.size}")
    return raw[0::2] + 1j * raw[1::2]


def savefig(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    print(f"  wrote {path}", flush=True)


def annotate(ax: plt.Axes, note: str, y: float = -0.22) -> None:
    ax.text(
        0.0,
        y,
        textwrap.fill(note, 110),
        transform=ax.transAxes,
        fontsize=7,
        color="#333333",
        va="top",
        ha="left",
        wrap=True,
    )


# ---------------------------------------------------------------------------
# Coding chain (Python port of coding.cpp)
# ---------------------------------------------------------------------------
def crc16_byte(crc: int, byte: int) -> int:
    for _ in range(8):
        if ((crc & 0x8000) >> 8) ^ (byte & 0x80):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
        byte = (byte << 1) & 0xFF
    return crc


def payload_crc(payload: np.ndarray) -> int:
    crc = 0
    n = len(payload)
    if n < 2:
        for b in payload:
            crc = crc16_byte(crc, int(b))
        return crc
    for i in range(n - 2):
        crc = crc16_byte(crc, int(payload[i]))
    return (crc ^ int(payload[-1]) ^ (int(payload[-2]) << 8)) & 0xFFFF


def header_checksum(h: list[int]) -> int:
    c4 = ((h[0] & 8) >> 3) ^ ((h[0] & 4) >> 2) ^ ((h[0] & 2) >> 1) ^ (h[0] & 1)
    c3 = ((h[0] & 8) >> 3) ^ ((h[1] & 8) >> 3) ^ ((h[1] & 4) >> 2) ^ ((h[1] & 2) >> 1) ^ (h[2] & 1)
    c2 = ((h[0] & 4) >> 2) ^ ((h[1] & 8) >> 3) ^ (h[1] & 1) ^ ((h[2] & 8) >> 3) ^ ((h[2] & 2) >> 1)
    c1 = (
        ((h[0] & 2) >> 1)
        ^ ((h[1] & 4) >> 2)
        ^ (h[1] & 1)
        ^ ((h[2] & 4) >> 2)
        ^ ((h[2] & 2) >> 1)
        ^ (h[2] & 1)
    )
    c0 = (
        (h[0] & 1)
        ^ ((h[1] & 2) >> 1)
        ^ ((h[2] & 8) >> 3)
        ^ ((h[2] & 4) >> 2)
        ^ ((h[2] & 2) >> 1)
        ^ (h[2] & 1)
    )
    return (c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0


def build_header(payload_len: int, cr: int, has_crc: bool) -> list[int]:
    h = [payload_len >> 4, payload_len & 0xF, (cr << 1) | (1 if has_crc else 0), 0, 0]
    chk = header_checksum(h)
    h[3] = (chk >> 4) & 1
    h[4] = chk & 0xF
    return h


def hamming_encode(nibble: int, cr_app: int) -> int:
    b0, b1, b2, b3 = nibble & 1, (nibble >> 1) & 1, (nibble >> 2) & 1, (nibble >> 3) & 1
    if cr_app == 1:
        p4 = b0 ^ b1 ^ b2 ^ b3
        return (b0 << 4) | (b1 << 3) | (b2 << 2) | (b3 << 1) | p4
    p0 = b0 ^ b1 ^ b2
    p1 = b1 ^ b2 ^ b3
    p2 = b0 ^ b1 ^ b3
    p3 = b0 ^ b2 ^ b3
    full = (b0 << 7) | (b1 << 6) | (b2 << 5) | (b3 << 4) | (p0 << 3) | (p1 << 2) | (p2 << 1) | p3
    return full >> (4 - cr_app)


def gray_decode(g: int, bits: int) -> int:
    b = g
    for j in range(1, bits):
        b ^= g >> j
    return b & ((1 << bits) - 1)


def gray_encode(b: int) -> int:
    return b ^ (b >> 1)


def interleave_block(cw: list[int], sf: int, sf_app: int, cw_len: int, reduced: bool) -> list[int]:
    out = []
    for i in range(cw_len):
        sym = 0
        ones = 0
        for j in range(sf_app):
            src = ((i - j - 1) % sf_app + sf_app) % sf_app
            bit = (cw[src] >> (cw_len - 1 - i)) & 1
            sym = (sym << 1) | bit
            ones += bit
        if reduced:
            sym = (sym << 1) | (ones % 2)
            sym <<= sf - sf_app - 1
        out.append(sym)
    return out


@dataclass
class EncodeTrace:
    payload: np.ndarray
    whitened: np.ndarray
    header: list[int]
    nibbles: np.ndarray
    codewords: list[int]
    interleaved: list[int]
    chips: np.ndarray
    crc: int


def encode_frame_traced(
    payload: np.ndarray, sf: int = 11, cr: int = 1, has_crc: bool = True, ldro: bool = False
) -> EncodeTrace:
    n_chips = 1 << sf
    header = build_header(len(payload), cr, has_crc)
    nibbles: list[int] = list(header)
    whitened = np.empty(len(payload), dtype=np.uint8)
    for i, b in enumerate(payload):
        w = int(b) ^ int(K_WHITENING[i % len(K_WHITENING)])
        whitened[i] = w
        nibbles.append(w & 0xF)
        nibbles.append(w >> 4)
    crc = 0
    if has_crc:
        crc = payload_crc(payload)
        nibbles.extend([crc & 0xF, (crc >> 4) & 0xF, (crc >> 8) & 0xF, (crc >> 12) & 0xF])

    codewords: list[int] = []
    interleaved: list[int] = []
    chips: list[int] = []
    idx = 0
    first = True
    while idx < len(nibbles):
        reduced = first or ldro
        cr_app = 4 if first else cr
        cw_len = cr_app + 4
        sf_app = sf - 2 if reduced else sf
        cw = []
        for _ in range(sf_app):
            nib = nibbles[idx] if idx < len(nibbles) else 0
            cw.append(hamming_encode(nib, cr_app))
            idx += 1
        codewords.extend(cw)
        syms = interleave_block(cw, sf, sf_app, cw_len, reduced)
        interleaved.extend(syms)
        for s in syms:
            chips.append((gray_decode(s, sf) + 1) % n_chips)
        first = False

    return EncodeTrace(
        payload=payload.astype(np.uint8),
        whitened=whitened,
        header=header,
        nibbles=np.array(nibbles, dtype=np.uint8),
        codewords=codewords,
        interleaved=interleaved,
        chips=np.array(chips, dtype=np.uint16),
        crc=crc,
    )


# ---------------------------------------------------------------------------
# Modulator / demod helpers
# ---------------------------------------------------------------------------
def build_upchirp(chip_id: int, sf: int, os: int, amplitude: float = 0.8) -> np.ndarray:
    n_chips = float(1 << sf)
    total = int(n_chips) * os
    n_fold = total - chip_id * os
    out = np.empty(total, dtype=np.complex64)
    n = np.arange(total, dtype=np.float64)
    quad = n * n / (2.0 * n_chips) / (os * os)
    lin = np.where(n < n_fold, chip_id / n_chips - 0.5, chip_id / n_chips - 1.5)
    phase = 2.0 * np.pi * (quad + lin * n / os)
    out.real = amplitude * np.cos(phase)
    out.imag = amplitude * np.sin(phase)
    return out


def build_frame_iq(
    chips: np.ndarray, sf: int, os: int, preamble_len: int, sync_word: int, amplitude: float
) -> np.ndarray:
    sps = (1 << sf) * os
    up = build_upchirp(0, sf, os, amplitude)
    down = np.conj(up)
    sync0 = ((sync_word >> 4) & 0xF) << 3
    sync1 = (sync_word & 0xF) << 3
    parts = [up] * preamble_len
    parts.append(build_upchirp(sync0, sf, os, amplitude))
    parts.append(build_upchirp(sync1, sf, os, amplitude))
    parts.append(down)
    parts.append(down)
    parts.append(down[: sps // 4])
    for c in chips:
        parts.append(build_upchirp(int(c), sf, os, amplitude))
    return np.concatenate(parts)


def demod_symbol(window: np.ndarray, ref_down: np.ndarray, n_chips: int) -> tuple[float, int, np.ndarray, np.ndarray]:
    """Return (chip_frac, chip_int, dechirped, folded_power)."""
    work = window * ref_down
    spec = np.fft.fft(work)
    os = len(window) // n_chips
    folded = np.zeros(n_chips, dtype=np.float64)
    for im in range(os):
        folded += np.abs(spec[im * n_chips : (im + 1) * n_chips]) ** 2
    peak = int(np.argmax(folded))
    a = folded[(peak - 1) % n_chips]
    b = folded[peak]
    c = folded[(peak + 1) % n_chips]
    denom = a - 2 * b + c
    delta = 0.5 * (a - c) / denom if abs(denom) > 1e-20 else 0.0
    return peak + delta, peak, work, folded


def welch_psd(x: np.ndarray, fs: float, nperseg: int | None = None):
    nperseg = nperseg or min(4096, max(256, len(x) // 8))
    nperseg = min(nperseg, len(x))
    f, pxx = sp_signal.welch(x, fs=fs, nperseg=nperseg, return_onesided=False)
    f = np.fft.fftshift(f)
    pxx = np.fft.fftshift(pxx)
    return f, 10 * np.log10(pxx + 1e-20)


def find_frame_start(rx: np.ndarray, ref_down: np.ndarray, sps: int, n_chips: int, search_syms: int = 80) -> int:
    """Coarse search: find strongest contiguous preamble-like run."""
    limit = min(len(rx) - 8 * sps, search_syms * sps)
    if limit <= 0:
        return 0
    best_score = -1.0
    best_off = 0
    step = max(1, sps // 8)
    for off in range(0, limit, step):
        score = 0.0
        prev = None
        for s in range(6):
            pos = off + s * sps
            if pos + sps > len(rx):
                break
            chip, _, _, folded = demod_symbol(rx[pos : pos + sps], ref_down, n_chips)
            peak_p = folded.max()
            med = np.median(folded)
            snr = peak_p / (med + 1e-20)
            if snr > 8:
                score += snr
                if prev is not None and abs(chip - prev) < 4:
                    score += 5
            prev = chip
        if score > best_score:
            best_score = score
            best_off = off
    return best_off


# ---------------------------------------------------------------------------
# Plot jobs (each returns list of written paths; designed for process pool)
# ---------------------------------------------------------------------------
@dataclass
class JobCtx:
    dump_dir: str
    out_dir: str
    label: str


def _ctx_paths(ctx: JobCtx):
    return Path(ctx.dump_dir), Path(ctx.out_dir)


def job_tx_coding(ctx: JobCtx) -> list[str]:
    apply_style()
    dump_dir, out = _ctx_paths(ctx)
    with (dump_dir / "lora_manifest.json").open() as f:
        man = json.load(f)
    sf = int(man["spreading_factor"])
    payload_len = int(man["payload_len"])
    payload = np.array([(i * 37 + 11) & 0xFF for i in range(payload_len)], dtype=np.uint8)
    # Default params match LoraParams; LDRO from symbol time
    bw = float(man["bandwidth_hz"])
    ldro = ((1 << sf) / bw) >= 16.384e-3
    tr = encode_frame_traced(payload, sf=sf, cr=1, has_crc=True, ldro=ldro)
    written = []

    # Dashboard: coding stages
    fig = plt.figure(figsize=(16, 14))
    gs = GridSpec(3, 2, figure=fig, hspace=0.4, wspace=0.28)
    fig.suptitle(f"{ctx.label} — TX coding chain", fontsize=12, fontweight="bold")

    ax = fig.add_subplot(gs[0, 0])
    ax.stem(np.arange(len(tr.payload)), tr.payload, basefmt=" ")
    ax.set_title("Payload bytes")
    ax.set_xlabel("Byte index")
    ax.set_ylabel("Value")
    annotate(ax, STAGE_NOTES["tx_payload"])

    ax = fig.add_subplot(gs[0, 1])
    ax.stem(np.arange(len(tr.whitened)), tr.whitened, linefmt="C1-", markerfmt="C1o", basefmt=" ")
    ax.set_title("Whitened payload")
    ax.set_xlabel("Byte index")
    ax.set_ylabel("Value")
    annotate(ax, STAGE_NOTES["tx_whitened"])

    ax = fig.add_subplot(gs[1, 0])
    ax.stem(np.arange(len(tr.nibbles)), tr.nibbles, linefmt="C2-", markerfmt="C2o", basefmt=" ")
    ax.axvline(4.5, color="k", ls="--", lw=0.8, label="end header")
    ax.set_title(f"Nibble stream (header={tr.header}, CRC=0x{tr.crc:04X})")
    ax.set_xlabel("Nibble index")
    ax.set_ylabel("Nibble (0..15)")
    ax.legend(loc="upper right")
    annotate(ax, STAGE_NOTES["tx_nibbles"])

    ax = fig.add_subplot(gs[1, 1])
    ax.stem(np.arange(len(tr.codewords)), tr.codewords, linefmt="C3-", markerfmt="C3o", basefmt=" ")
    ax.set_title("Hamming codewords")
    ax.set_xlabel("Codeword index")
    ax.set_ylabel("Codeword value")
    annotate(ax, STAGE_NOTES["tx_hamming"])

    ax = fig.add_subplot(gs[2, 0])
    ax.stem(
        np.arange(len(tr.interleaved)), tr.interleaved, linefmt="C4-", markerfmt="C4o", basefmt=" "
    )
    ax.set_title("Interleaved symbols (pre-Gray)")
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Symbol value")
    annotate(ax, STAGE_NOTES["tx_interleave"])

    ax = fig.add_subplot(gs[2, 1])
    ax.stem(np.arange(len(tr.chips)), tr.chips, linefmt="C0-", markerfmt="C0o", basefmt=" ")
    ax.axvline(7.5, color="k", ls="--", lw=0.8, label="end header block")
    ax.set_title(f"TX chip indices ({len(tr.chips)} data symbols)")
    ax.set_xlabel("Data symbol index")
    ax.set_ylabel("Chip (0..2^SF-1)")
    ax.legend(loc="upper right")
    annotate(ax, STAGE_NOTES["tx_chips"])

    path = out / "01_tx_coding_dashboard.png"
    savefig(fig, path)
    written.append(str(path))

    # Individual coding plots
    for name, data, title, note_key, ylabel in [
        ("01a_tx_payload", tr.payload, "Payload bytes", "tx_payload", "Value"),
        ("01b_tx_whitened", tr.whitened, "Whitened payload", "tx_whitened", "Value"),
        ("01c_tx_nibbles", tr.nibbles, "Nibble stream", "tx_nibbles", "Nibble"),
        ("01d_tx_hamming", tr.codewords, "Hamming codewords", "tx_hamming", "Codeword"),
        ("01e_tx_interleaved", tr.interleaved, "Interleaved symbols", "tx_interleave", "Symbol"),
        ("01f_tx_chips", tr.chips, "TX chip indices", "tx_chips", "Chip"),
    ]:
        fig, ax = plt.subplots(figsize=(10, 3.8))
        ax.stem(np.arange(len(data)), data, basefmt=" ")
        ax.set_title(title)
        ax.set_xlabel("Index")
        ax.set_ylabel(ylabel)
        annotate(ax, STAGE_NOTES[note_key])
        fig.tight_layout()
        p = out / f"{name}.png"
        savefig(fig, p)
        written.append(str(p))

    # Persist chips for RX jobs
    np.save(out / "_tx_chips.npy", tr.chips)
    with (out / "_encode_meta.json").open("w") as f:
        json.dump(
            {
                "header": tr.header,
                "crc": tr.crc,
                "n_chips_data": int(len(tr.chips)),
                "ldro": ldro,
                "sf": sf,
                "payload_hex": payload.tobytes().hex(),
            },
            f,
            indent=2,
        )
    return written


def job_tx_iq(ctx: JobCtx) -> list[str]:
    apply_style()
    dump_dir, out = _ctx_paths(ctx)
    with (dump_dir / "lora_manifest.json").open() as f:
        man = json.load(f)
    tx = load_iq(dump_dir / "lora_tx_frame.iq")
    fs = float(man["sample_rate_hz"])
    bw = float(man["bandwidth_hz"])
    sps = int(man["sps"])
    pre = int(man["preamble_len"])
    n_data = int(man["data_symbols"])
    written = []

    # Region boundaries (samples)
    sync_start = pre * sps
    sfd_start = sync_start + 2 * sps
    data_start = sfd_start + 2 * sps + sps // 4
    regions = [
        (0, sync_start, "Preamble", "C0"),
        (sync_start, sfd_start, "Sync", "C1"),
        (sfd_start, data_start, "SFD", "C2"),
        (data_start, data_start + n_data * sps, "Data", "C3"),
    ]

    # Dashboard
    fig = plt.figure(figsize=(16, 14))
    gs = GridSpec(3, 2, figure=fig, hspace=0.38, wspace=0.28)
    fig.suptitle(f"{ctx.label} — TX modulated IQ", fontsize=12, fontweight="bold")

    ax = fig.add_subplot(gs[0, :])
    t_ms = np.arange(len(tx)) / fs * 1e3
    # downsample for display
    step = max(1, len(tx) // 8000)
    ax.plot(t_ms[::step], np.abs(tx[::step]), color="C0", lw=0.7)
    for a, b, name, c in regions:
        ax.axvspan(a / fs * 1e3, min(b, len(tx)) / fs * 1e3, alpha=0.12, color=c, label=name)
    ax.set_title(f"TX magnitude  ({len(tx)} samples, {len(tx)/fs*1e3:.1f} ms)")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("|x|")
    ax.legend(loc="upper right", ncol=4)
    annotate(ax, STAGE_NOTES["tx_iq_time"], y=-0.28)

    ax = fig.add_subplot(gs[1, 0])
    nfft = min(1024, sps)
    ax.specgram(tx, NFFT=nfft, Fs=fs, noverlap=nfft // 2, scale="dB", cmap="viridis")
    ax.set_title("TX spectrogram")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    ax.set_ylim(-fs / 2, fs / 2)
    annotate(ax, STAGE_NOTES["tx_iq_spec"])

    ax = fig.add_subplot(gs[1, 1])
    # constellation: subsample one symbol from each region
    for a, b, name, c in regions:
        seg = tx[a : min(a + sps, b, len(tx))]
        if len(seg) == 0:
            continue
        ss = max(1, len(seg) // 400)
        ax.plot(seg.real[::ss], seg.imag[::ss], ".", ms=1.5, alpha=0.5, color=c, label=name)
    ax.set_aspect("equal")
    ax.set_title("TX IQ constellation (by region)")
    ax.set_xlabel("In-phase")
    ax.set_ylabel("Quadrature")
    ax.legend(loc="upper right", markerscale=4)
    annotate(ax, STAGE_NOTES["tx_iq_const"])

    ax = fig.add_subplot(gs[2, 0])
    f, pdb = welch_psd(tx, fs, nperseg=min(4096, len(tx)))
    ax.plot(f / 1e3, pdb, color="C0")
    ax.axvline(-bw / 2e3, color="r", ls="--", lw=0.8)
    ax.axvline(bw / 2e3, color="r", ls="--", lw=0.8, label=f"±BW/2 ({bw/1e3:.0f} kHz)")
    ax.set_title("TX power spectral density (Welch)")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.legend(loc="upper right")
    annotate(ax, STAGE_NOTES["tx_iq_psd"])

    ax = fig.add_subplot(gs[2, 1])
    # Instantaneous frequency of first preamble symbol
    sym0 = tx[:sps]
    phase = np.unwrap(np.angle(sym0))
    inst_f = np.diff(phase) * fs / (2 * np.pi)
    t = np.arange(len(inst_f)) / fs * 1e3
    ax.plot(t, inst_f / 1e3, color="C0", lw=0.8)
    ax.set_title("Instantaneous frequency — preamble symbol 0")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Frequency (kHz)")
    ax.axhline(-bw / 2e3, color="r", ls=":", lw=0.8)
    ax.axhline(bw / 2e3, color="r", ls=":", lw=0.8)
    annotate(ax, "Expected: linear sweep from -BW/2 to +BW/2 over one symbol period.")

    path = out / "02_tx_iq_dashboard.png"
    savefig(fig, path)
    written.append(str(path))

    # Individual
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t_ms[::step], np.abs(tx[::step]), lw=0.7)
    for a, b, name, c in regions:
        ax.axvspan(a / fs * 1e3, min(b, len(tx)) / fs * 1e3, alpha=0.12, color=c, label=name)
    ax.set_title("TX magnitude vs time")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("|x|")
    ax.legend(ncol=4, loc="upper right")
    annotate(ax, STAGE_NOTES["tx_iq_time"])
    fig.tight_layout()
    p = out / "02a_tx_magnitude.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.specgram(tx, NFFT=nfft, Fs=fs, noverlap=nfft // 2, scale="dB", cmap="viridis")
    ax.set_title("TX spectrogram")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    annotate(ax, STAGE_NOTES["tx_iq_spec"])
    fig.tight_layout()
    p = out / "02b_tx_spectrogram.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(6, 6))
    ss = max(1, len(tx) // 5000)
    ax.plot(tx.real[::ss], tx.imag[::ss], ".", ms=1, alpha=0.35, color="C0")
    ax.set_aspect("equal")
    ax.set_title("TX IQ constellation")
    ax.set_xlabel("In-phase")
    ax.set_ylabel("Quadrature")
    annotate(ax, STAGE_NOTES["tx_iq_const"])
    fig.tight_layout()
    p = out / "02c_tx_constellation.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(f / 1e3, pdb)
    ax.axvline(-bw / 2e3, color="r", ls="--")
    ax.axvline(bw / 2e3, color="r", ls="--")
    ax.set_title("TX PSD")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("PSD (dB/Hz)")
    annotate(ax, STAGE_NOTES["tx_iq_psd"])
    fig.tight_layout()
    p = out / "02d_tx_psd.png"
    savefig(fig, p)
    written.append(str(p))

    # Per-region symbol detail: preamble0, sync0, sfd0, data0
    labels_sym = [
        ("preamble0", 0, False),
        ("sync0", sync_start, False),
        ("sfd0", sfd_start, True),
        ("data0", data_start, False),
    ]
    fig, axes = plt.subplots(4, 3, figsize=(14, 12))
    fig.suptitle(f"{ctx.label} — TX symbol detail (time / spectrum / IQ)", fontsize=11)
    for row, (name, start, is_down) in enumerate(labels_sym):
        seg = tx[start : start + sps]
        if len(seg) < sps:
            continue
        tt = np.arange(sps) / fs * 1e3
        axes[row, 0].plot(tt, seg.real, lw=0.6, label="I")
        axes[row, 0].plot(tt, seg.imag, lw=0.6, label="Q")
        axes[row, 0].set_ylabel(name)
        axes[row, 0].legend(loc="upper right", fontsize=6)
        if row == 0:
            axes[row, 0].set_title("Time domain (I/Q)")
        axes[row, 0].set_xlabel("Time (ms)")

        spec = np.fft.fftshift(np.fft.fft(seg))
        freqs = np.fft.fftshift(np.fft.fftfreq(sps, 1 / fs))
        axes[row, 1].plot(freqs / 1e3, 20 * np.log10(np.abs(spec) + 1e-12), lw=0.7)
        if row == 0:
            axes[row, 1].set_title("Magnitude spectrum")
        axes[row, 1].set_xlabel("Frequency (kHz)")
        axes[row, 1].set_ylabel("dB")

        ss = max(1, sps // 300)
        axes[row, 2].plot(seg.real[::ss], seg.imag[::ss], ".", ms=2, alpha=0.5)
        axes[row, 2].set_aspect("equal")
        if row == 0:
            axes[row, 2].set_title("IQ")
        axes[row, 2].set_xlabel("I")
        axes[row, 2].set_ylabel("Q")
    fig.tight_layout()
    p = out / "02e_tx_symbol_detail.png"
    savefig(fig, p)
    written.append(str(p))

    return written


def job_rx_capture(ctx: JobCtx) -> list[str]:
    apply_style()
    dump_dir, out = _ctx_paths(ctx)
    with (dump_dir / "lora_manifest.json").open() as f:
        man = json.load(f)
    rx = load_iq(dump_dir / "lora_rx_raw.iq")
    fs = float(man["sample_rate_hz"])
    bw = float(man["bandwidth_hz"])
    pattern = int(man["pattern_samples"])
    written = []

    mag = np.abs(rx)
    rms = float(np.sqrt(np.mean(mag**2)))
    peak = float(np.max(mag))
    clip = float(np.mean((np.abs(rx.real) > 0.95) | (np.abs(rx.imag) > 0.95)))
    rms_dbfs = 20 * math.log10(rms / 127.0 + 1e-12)  # Soapy int8 scale convention in stats.hpp may differ
    # Host float path uses amplitude ~0.8; report relative dB to peak float
    rms_db = 20 * math.log10(rms + 1e-12)

    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 2, figure=fig, hspace=0.38, wspace=0.28)
    fig.suptitle(
        f"{ctx.label} — RX capture  N={len(rx)}  RMS={rms_db:.1f} dB  peak={peak:.3f}  clip≈{clip*100:.2f}%",
        fontsize=12,
        fontweight="bold",
    )

    ax = fig.add_subplot(gs[0, :])
    step = max(1, len(rx) // 12000)
    t = np.arange(len(rx)) / fs
    ax.plot(t[::step], mag[::step], lw=0.5, color="C0")
    for k in range(0, len(rx), pattern):
        ax.axvline(k / fs, color="C3", ls=":", lw=0.6, alpha=0.5)
    ax.set_title("RX magnitude (dotted lines = TX pattern period)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|x|")
    annotate(ax, STAGE_NOTES["rx_time"], y=-0.28)

    # Spectrogram of first ~2 pattern periods (or 2s)
    ax = fig.add_subplot(gs[1, 0])
    n_show = min(len(rx), int(min(2.0 * fs, 2 * pattern)))
    nfft = 1024
    ax.specgram(
        rx[:n_show],
        NFFT=nfft,
        Fs=fs,
        noverlap=nfft // 2,
        cmap="viridis",
        scale="dB",
    )
    ax.set_title(f"RX spectrogram (first {n_show/fs:.2f} s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    annotate(ax, STAGE_NOTES["rx_spec"])

    ax = fig.add_subplot(gs[1, 1])
    # IQ of a mid-capture window
    mid = len(rx) // 2
    win = rx[mid : mid + min(int(0.05 * fs), len(rx) - mid)]
    ss = max(1, len(win) // 4000)
    ax.plot(win.real[::ss], win.imag[::ss], ".", ms=1, alpha=0.35)
    ax.set_aspect("equal")
    ax.set_title("RX IQ constellation (50 ms mid-capture)")
    ax.set_xlabel("In-phase")
    ax.set_ylabel("Quadrature")
    annotate(ax, STAGE_NOTES["rx_iq"])

    ax = fig.add_subplot(gs[2, 0])
    f, pdb = welch_psd(rx[: min(len(rx), int(2 * fs))], fs)
    ax.plot(f / 1e3, pdb)
    ax.axvline(-bw / 2e3, color="r", ls="--")
    ax.axvline(bw / 2e3, color="r", ls="--", label=f"±BW/2")
    ax.set_title("RX PSD (first 2 s)")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.legend()
    annotate(ax, STAGE_NOTES["tx_iq_psd"])

    ax = fig.add_subplot(gs[2, 1])
    # Frame envelope autocorrelation-ish: magnitude energy per pattern slot
    n_pat = max(1, len(rx) // pattern)
    env = []
    for i in range(min(n_pat, 40)):
        seg = mag[i * pattern : (i + 1) * pattern]
        env.append(float(np.mean(seg**2)))
    ax.bar(np.arange(len(env)), 10 * np.log10(np.array(env) + 1e-20), color="C0")
    ax.set_title("Mean power per TX pattern period")
    ax.set_xlabel("Pattern index")
    ax.set_ylabel("Power (dB)")
    annotate(ax, "Expected: stable power across periods. Dips indicate dropouts or TX gaps.")

    path = out / "03_rx_capture_dashboard.png"
    savefig(fig, path)
    written.append(str(path))

    # Individuals
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t[::step], mag[::step], lw=0.5)
    ax.set_title("RX magnitude vs time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|x|")
    annotate(ax, STAGE_NOTES["rx_time"])
    fig.tight_layout()
    p = out / "03a_rx_magnitude.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.specgram(rx[:n_show], NFFT=nfft, Fs=fs, noverlap=nfft // 2, scale="dB", cmap="viridis")
    ax.set_title("RX spectrogram")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    annotate(ax, STAGE_NOTES["rx_spec"])
    fig.tight_layout()
    p = out / "03b_rx_spectrogram.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(win.real[::ss], win.imag[::ss], ".", ms=1, alpha=0.35)
    ax.set_aspect("equal")
    ax.set_title("RX IQ constellation")
    ax.set_xlabel("I")
    ax.set_ylabel("Q")
    annotate(ax, STAGE_NOTES["rx_iq"])
    fig.tight_layout()
    p = out / "03c_rx_constellation.png"
    savefig(fig, p)
    written.append(str(p))

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(f / 1e3, pdb)
    ax.set_title("RX PSD")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("PSD (dB/Hz)")
    fig.tight_layout()
    p = out / "03d_rx_psd.png"
    savefig(fig, p)
    written.append(str(p))

    return written


def job_rx_demod(ctx: JobCtx) -> list[str]:
    apply_style()
    dump_dir, out = _ctx_paths(ctx)
    with (dump_dir / "lora_manifest.json").open() as f:
        man = json.load(f)
    frames = json.loads((dump_dir / "lora_frames.json").read_text())
    rx = load_iq(dump_dir / "lora_rx_raw.iq")
    fs = float(man["sample_rate_hz"])
    bw = float(man["bandwidth_hz"])
    sf = int(man["spreading_factor"])
    sps = int(man["sps"])
    pre = int(man["preamble_len"])
    n_chips = 1 << sf
    os_factor = sps // n_chips
    sync_word = 0x2B
    sync0 = ((sync_word >> 4) & 0xF) << 3
    sync1 = (sync_word & 0xF) << 3

    chips_path = out / "_tx_chips.npy"
    if chips_path.exists():
        tx_chips = np.load(chips_path)
    else:
        payload_len = int(man["payload_len"])
        payload = np.array([(i * 37 + 11) & 0xFF for i in range(payload_len)], dtype=np.uint8)
        ldro = ((1 << sf) / bw) >= 16.384e-3
        tx_chips = encode_frame_traced(payload, sf=sf, cr=1, has_crc=True, ldro=ldro).chips

    ref_down = np.conj(build_upchirp(0, sf, os_factor, 1.0))
    ref_up = build_upchirp(0, sf, os_factor, 1.0)

    # Prefer receiver-reported start; else search. Note: receiver start is after
    # 0.5s skip in the app, and absolute to the fed stream — dumps include skip.
    skip = int(0.5 * fs)
    mean_ppm = float(np.mean([f["ppm"] for f in frames])) if frames else 0.0
    # RX sample clock is fast by ~ppm → each nominal symbol spans more RX samples
    sps_rx = sps * (1.0 + mean_ppm / 1e6)

    def window_at(abs_f: float) -> np.ndarray | None:
        """Cubic-ish linear resample of sps nominal samples from fractional raw index."""
        idx = abs_f + np.arange(sps, dtype=np.float64)
        if idx[-1] >= len(rx) - 1 or idx[0] < 0:
            return None
        i0 = np.floor(idx).astype(np.int64)
        frac = idx - i0
        return ((1.0 - frac) * rx[i0] + frac * rx[i0 + 1]).astype(np.complex64)

    if frames:
        # start is relative to post-skip stream; dump includes everything
        align = int(frames[0]["start"]) + skip
        if align + int((pre + 4) * sps_rx) > len(rx):
            align = find_frame_start(rx[skip:], ref_down, sps, n_chips) + skip
    else:
        align = find_frame_start(rx[skip:], ref_down, sps, n_chips) + skip

    written = []

    # Demod preamble + sync + sfd + data for one frame (rate-corrected grid)
    n_data = int(man["data_symbols"])
    data_start_f = align + (pre + 2 + 2.25) * sps_rx

    traj = []
    mags = []
    for s in range(pre + 2 + 2):
        win = window_at(align + s * sps_rx)
        if win is None:
            break
        use_down_ref = s >= pre + 2  # SFD uses upchirp ref (= downchirp signal)
        ref = ref_up if use_down_ref else ref_down
        chip, chip_i, dechirped, folded = demod_symbol(win, ref, n_chips)
        traj.append(chip)
        mags.append(folded.max())

    # Data symbols: remove preamble CFO/ref then hard-decide (matches Receiver)
    rx_fracs = []
    data_foldeds = []
    if len(traj) >= 3:
        signed = [((c + n_chips / 2) % n_chips) - n_chips / 2 for c in traj[1 : min(12, len(traj))]]
        cfo_chips = float(np.median(signed))
    else:
        cfo_chips = 0.0
    ref_chip = cfo_chips  # for meta
    decided = []
    for s in range(min(n_data, 24)):
        win = window_at(data_start_f + s * sps_rx)
        if win is None:
            break
        chip, chip_i, _, folded = demod_symbol(win, ref_down, n_chips)
        # residual after CFO removal; light decision-directed track
        resid = ((chip - cfo_chips + n_chips / 2) % n_chips) - n_chips / 2
        hard = int(round(resid)) % n_chips
        cfo_chips += 0.15 * (((chip - hard + n_chips / 2) % n_chips) - n_chips / 2 - cfo_chips)
        rx_fracs.append(chip)
        decided.append(hard)
        if s < 8:
            data_foldeds.append(folded)
    decided = np.array(decided, dtype=np.int32)

    # --- Dashboard ---
    fig = plt.figure(figsize=(16, 16))
    gs = GridSpec(4, 2, figure=fig, hspace=0.4, wspace=0.28)
    fig.suptitle(
        f"{ctx.label} — RX demodulation  align={align}  frames={len(frames)}",
        fontsize=12,
        fontweight="bold",
    )

    # Preamble trajectory
    ax = fig.add_subplot(gs[0, 0])
    if traj:
        ax.plot(np.arange(len(traj)), traj, "o-", ms=3)
        ax.axvline(pre - 0.5, color="C1", ls="--", label="sync")
        ax.axvline(pre + 1.5, color="C2", ls="--", label="SFD")
        ax.axhline(0, color="gray", ls=":", lw=0.8)
    ax.set_title("Dechirped peak chip vs symbol (preamble/sync/SFD)")
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Chip")
    ax.legend(loc="best")
    annotate(ax, STAGE_NOTES["rx_preamble_traj"])

    ax = fig.add_subplot(gs[0, 1])
    if mags:
        ax.semilogy(np.arange(len(mags)), mags, "o-", ms=3, color="C3")
    ax.set_title("Peak magnitude vs symbol")
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("|peak|^2")
    annotate(ax, "Expected: strong, stable peaks through preamble; sync similar; SFD strong with downchirp ref.")

    # Waterfall of folded spectra for preamble
    ax = fig.add_subplot(gs[1, 0])
    pre_fold = []
    for s in range(min(pre, 16)):
        win = window_at(align + s * sps_rx)
        if win is None:
            break
        _, _, _, folded = demod_symbol(win, ref_down, n_chips)
        pre_fold.append(10 * np.log10(folded + 1e-20))
    if pre_fold:
        img = np.array(pre_fold)
        im = ax.imshow(
            img,
            aspect="auto",
            origin="lower",
            cmap="viridis",
            extent=[-0.5, n_chips - 0.5, -0.5, len(pre_fold) - 0.5],
        )
        plt.colorbar(im, ax=ax, fraction=0.046, label="dB")
    ax.set_title("Preamble folded-chip waterfall")
    ax.set_xlabel("Chip bin")
    ax.set_ylabel("Preamble symbol")
    annotate(ax, STAGE_NOTES["rx_fft"])

    # Example dechirped time + FFT for preamble0 and sync0
    ax = fig.add_subplot(gs[1, 1])
    for s, color, lab in [(0, "C0", "preamble0"), (pre, "C1", "sync0")]:
        win = window_at(align + s * sps_rx)
        if win is None:
            continue
        _, _, dechirped, folded = demod_symbol(win, ref_down, n_chips)
        ax.plot(np.arange(n_chips), 10 * np.log10(folded + 1e-20), color=color, lw=0.9, label=lab)
    ax.axvline(sync0, color="C1", ls=":", lw=0.8)
    ax.axvline(sync1, color="C3", ls=":", lw=0.8, label=f"sync chips {sync0},{sync1}")
    ax.set_title("Folded chip spectrum")
    ax.set_xlabel("Chip bin")
    ax.set_ylabel("Power (dB)")
    ax.legend(loc="upper right")
    annotate(ax, STAGE_NOTES["rx_fft"])

    # Dechirped time domain preamble0
    ax = fig.add_subplot(gs[2, 0])
    win0 = window_at(float(align))
    if win0 is not None:
        _, _, dechirped, _ = demod_symbol(win0, ref_down, n_chips)
        tt = np.arange(sps) / fs * 1e3
        step = max(1, sps // 2000)
        ax.plot(tt[::step], dechirped.real[::step], lw=0.6, label="I")
        ax.plot(tt[::step], dechirped.imag[::step], lw=0.6, label="Q")
        ax.legend(loc="upper right")
    ax.set_title("Dechirped preamble symbol 0 (time)")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    annotate(ax, STAGE_NOTES["rx_dechirp"])

    # IQ of dechirped preamble0
    ax = fig.add_subplot(gs[2, 1])
    if win0 is not None:
        _, _, dechirped, _ = demod_symbol(win0, ref_down, n_chips)
        ss = max(1, sps // 500)
        ax.plot(dechirped.real[::ss], dechirped.imag[::ss], ".", ms=2, alpha=0.4)
        ax.set_aspect("equal")
    ax.set_title("Dechirped preamble0 IQ (should be a tone arc/cluster)")
    ax.set_xlabel("I")
    ax.set_ylabel("Q")
    annotate(ax, "Expected: tight cluster or slow arc (residual CFO). Noise fills a cloud.")

    # Data chips TX vs RX
    ax = fig.add_subplot(gs[3, 0])
    n_cmp = min(len(tx_chips), len(decided))
    if n_cmp:
        ax.step(np.arange(n_cmp), tx_chips[:n_cmp], where="mid", label="TX chips", lw=1.2)
        ax.plot(np.arange(n_cmp), decided[:n_cmp], "o", ms=4, label="RX decided", alpha=0.8)
        err = []
        for i in range(n_cmp):
            d = abs(int(decided[i]) - int(tx_chips[i]))
            d = min(d, n_chips - d)
            err.append(d)
        bad = [i for i, e in enumerate(err) if e > 0]
        if bad:
            ax.plot(bad, decided[bad], "rx", ms=7, label=f"errors ({len(bad)})")
    ax.set_title(
        f"Data chips: TX vs RX (rate-corrected offline, {mean_ppm:+.0f} ppm)"
    )
    ax.set_xlabel("Data symbol")
    ax.set_ylabel("Chip")
    ax.legend(loc="best")
    annotate(ax, STAGE_NOTES["rx_data_chips"])

    ax = fig.add_subplot(gs[3, 1])
    if frames:
        ax.plot([f["ppm"] for f in frames], "o-", ms=3, label="rate ppm")
        ax2 = ax.twinx()
        ax2.plot([f["cfo_hz"] for f in frames], "s-", ms=3, color="C1", label="CFO Hz")
        ax.set_ylabel("Rate (ppm)")
        ax2.set_ylabel("CFO (Hz)")
        bad = [i for i, f in enumerate(frames) if not f.get("payload_ok")]
        if bad:
            ax.plot(bad, [frames[i]["ppm"] for i in bad], "rx", ms=8, label="payload fail")
        ax.set_title(
            f"Per-frame sync estimates  ok={sum(1 for f in frames if f.get('payload_ok'))}/{len(frames)}"
        )
        ax.set_xlabel("Frame index")
        ax.legend(loc="upper left")
    annotate(ax, STAGE_NOTES["rx_decode"], y=-0.28)

    path = out / "04_rx_demod_dashboard.png"
    savefig(fig, path)
    written.append(str(path))

    # Individual demod plots
    fig, ax = plt.subplots(figsize=(10, 4))
    if traj:
        ax.plot(traj, "o-", ms=4)
    ax.set_title("Preamble/sync/SFD chip trajectory")
    ax.set_xlabel("Symbol index")
    ax.set_ylabel("Chip")
    annotate(ax, STAGE_NOTES["rx_preamble_traj"])
    fig.tight_layout()
    p = out / "04a_rx_preamble_trajectory.png"
    savefig(fig, p)
    written.append(str(p))

    if pre_fold:
        fig, ax = plt.subplots(figsize=(10, 5))
        im = ax.imshow(
            np.array(pre_fold),
            aspect="auto",
            origin="lower",
            cmap="viridis",
            extent=[-0.5, n_chips - 0.5, -0.5, len(pre_fold) - 0.5],
        )
        plt.colorbar(im, ax=ax, label="dB")
        ax.set_title("Preamble folded-chip waterfall")
        ax.set_xlabel("Chip bin")
        ax.set_ylabel("Symbol")
        annotate(ax, STAGE_NOTES["rx_fft"])
        fig.tight_layout()
        p = out / "04b_rx_preamble_waterfall.png"
        savefig(fig, p)
        written.append(str(p))

    # Sync detail
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for idx_s, (s, exp, title) in enumerate(
        [(pre, sync0, "Sync symbol 0"), (pre + 1, sync1, "Sync symbol 1")]
    ):
        win = window_at(align + s * sps_rx)
        if win is None:
            continue
        _, peak_i, _, folded = demod_symbol(win, ref_down, n_chips)
        # expected sync peak is shifted by the same CFO/ref as preamble
        axes[idx_s].plot(10 * np.log10(folded + 1e-20), lw=0.8)
        axes[idx_s].axvline(exp, color="r", ls="--", label=f"nominal {exp}")
        axes[idx_s].axvline(peak_i, color="C2", ls=":", label=f"peak {peak_i}")
        axes[idx_s].set_title(title)
        axes[idx_s].set_xlabel("Chip bin")
        axes[idx_s].set_ylabel("Power (dB)")
        axes[idx_s].legend()
    fig.suptitle("Sync word chip peaks")
    annotate(axes[0], STAGE_NOTES["rx_sync"], y=-0.28)
    fig.tight_layout()
    p = out / "04c_rx_sync_peaks.png"
    savefig(fig, p)
    written.append(str(p))

    # SFD
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for i in range(2):
        win = window_at(align + (pre + 2 + i) * sps_rx)
        if win is None:
            continue
        _, peak_i, _, folded = demod_symbol(win, ref_up, n_chips)
        axes[i].plot(10 * np.log10(folded + 1e-20), lw=0.8)
        axes[i].axvline(peak_i, color="C2", ls="--", label=f"peak {peak_i}")
        axes[i].set_title(f"SFD downchirp symbol {i}")
        axes[i].set_xlabel("Chip bin")
        axes[i].set_ylabel("Power (dB)")
        axes[i].legend()
    annotate(axes[0], STAGE_NOTES["rx_sfd"], y=-0.28)
    fig.tight_layout()
    p = out / "04d_rx_sfd_peaks.png"
    savefig(fig, p)
    written.append(str(p))

    # Data chip comparison
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    n_cmp = min(len(tx_chips), len(decided))
    if n_cmp:
        axes[0].step(np.arange(n_cmp), tx_chips[:n_cmp], where="mid", label="TX")
        axes[0].plot(np.arange(n_cmp), decided[:n_cmp], "o", ms=4, label="RX")
        axes[0].set_ylabel("Chip")
        axes[0].legend()
        axes[0].set_title("Data symbol chips")
        err = np.array(
            [
                min(
                    abs(int(decided[i]) - int(tx_chips[i])),
                    n_chips - abs(int(decided[i]) - int(tx_chips[i])),
                )
                for i in range(n_cmp)
            ]
        )
        axes[1].bar(np.arange(n_cmp), err, color="C3")
        axes[1].set_ylabel("Circular chip error")
        axes[1].set_xlabel("Data symbol index")
        axes[1].set_title(f"Chip errors: {int(np.sum(err > 0))}/{n_cmp}")
    annotate(axes[1], STAGE_NOTES["rx_data_chips"], y=-0.35)
    fig.tight_layout()
    p = out / "04e_rx_data_chips.png"
    savefig(fig, p)
    written.append(str(p))

    # Dechirp time/freq/IQ for preamble0
    if win0 is not None:
        _, _, dechirped, folded = demod_symbol(win0, ref_down, n_chips)
        fig, axes = plt.subplots(1, 3, figsize=(14, 3.8))
        tt = np.arange(sps) / fs * 1e3
        step = max(1, sps // 2000)
        axes[0].plot(tt[::step], dechirped.real[::step], lw=0.6)
        axes[0].plot(tt[::step], dechirped.imag[::step], lw=0.6)
        axes[0].set_title("Dechirped time")
        axes[0].set_xlabel("Time (ms)")
        axes[1].plot(10 * np.log10(folded + 1e-20))
        axes[1].set_title("Folded chip spectrum")
        axes[1].set_xlabel("Chip")
        axes[1].set_ylabel("dB")
        ss = max(1, sps // 400)
        axes[2].plot(dechirped.real[::ss], dechirped.imag[::ss], ".", ms=2, alpha=0.4)
        axes[2].set_aspect("equal")
        axes[2].set_title("Dechirped IQ")
        fig.suptitle("Preamble symbol 0 — dechirp domains")
        fig.tight_layout()
        p = out / "04f_rx_dechirp_preamble0.png"
        savefig(fig, p)
        written.append(str(p))

    # Save align for summary
    with (out / "_demod_meta.json").open("w") as f:
        json.dump(
            {
                "align": align,
                "sps_rx": sps_rx,
                "mean_ppm": mean_ppm,
                "sync0_expected": sync0,
                "sync1_expected": sync1,
                "n_frames": len(frames),
                "payload_ok": sum(1 for fr in frames if fr.get("payload_ok")),
                "ref_chip_est": float(ref_chip),
                "offline_chip_errors": int(
                    sum(
                        min(
                            abs(int(decided[i]) - int(tx_chips[i])),
                            n_chips - abs(int(decided[i]) - int(tx_chips[i])),
                        )
                        > 0
                        for i in range(min(len(decided), len(tx_chips)))
                    )
                )
                if len(decided)
                else -1,
            },
            f,
            indent=2,
        )

    return written


def job_summary(ctx: JobCtx) -> list[str]:
    apply_style()
    dump_dir, out = _ctx_paths(ctx)
    with (dump_dir / "lora_manifest.json").open() as f:
        man = json.load(f)
    frames = json.loads((dump_dir / "lora_frames.json").read_text())
    run_log = ""
    log_path = dump_dir / "run.log"
    if log_path.exists():
        run_log = log_path.read_text()[-2000:]

    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)
    mode = "OTA RF" if man.get("ota") else "Digital FPGA"
    fig.suptitle(f"{ctx.label} — end-to-end summary ({mode})", fontsize=12, fontweight="bold")

    ax = fig.add_subplot(gs[0, 0])
    ax.axis("off")
    lines = [
        f"Mode:              {mode}",
        f"SF / BW / sps:     SF{man['spreading_factor']} / {man['bandwidth_hz']/1e3:.0f} kHz / {man['sps']}",
        f"Payload:           {man['payload_len']} bytes",
        f"Data symbols:      {man['data_symbols']}",
        f"Warmup symbols:    {man.get('warmup_syms', 0)}",
        f"Pattern samples:   {man['pattern_samples']}",
        f"Frames found:      {man.get('frames_found', len(frames))}",
        f"Payload OK:        {man.get('frames_payload_ok', sum(1 for f in frames if f.get('payload_ok')))}",
    ]
    if frames:
        ppms = [f["ppm"] for f in frames]
        cfos = [f["cfo_hz"] for f in frames]
        snrs = [f["snr_db"] for f in frames]
        lines += [
            f"Rate ppm:          {np.mean(ppms):+.1f} (min {min(ppms):+.1f}, max {max(ppms):+.1f})",
            f"CFO Hz:            {np.mean(cfos):+.1f} (min {min(cfos):+.1f}, max {max(cfos):+.1f})",
            f"SNR dB:            {np.mean(snrs):.1f} (min {min(snrs):.1f}, max {max(snrs):.1f})",
            f"Chip errs/frame:   mean {np.mean([f['chips_wrong'] for f in frames]):.2f}",
        ]
    ax.text(0.02, 0.98, "\n".join(lines), va="top", family="monospace", fontsize=9)
    ax.set_title("Run parameters & results")

    ax = fig.add_subplot(gs[0, 1])
    if frames:
        cats = ["synced", "hdr", "crc", "payload_ok"]
        vals = [sum(1 for f in frames if f.get(k)) for k in cats]
        ax.bar(cats, vals, color=["C0", "C1", "C2", "C3"])
        ax.axhline(len(frames), color="k", ls="--", lw=0.8, label=f"N={len(frames)}")
        ax.set_ylabel("Frame count")
        ax.legend()
    ax.set_title("Decode funnel")

    ax = fig.add_subplot(gs[1, 0])
    if frames:
        ax.plot([f["snr_db"] for f in frames], "o-", ms=3, label="SNR")
        ax.set_ylabel("SNR (dB)")
        ax2 = ax.twinx()
        ax2.plot([f["chips_wrong"] for f in frames], "s-", ms=3, color="C3", label="chip errs")
        ax2.set_ylabel("Chip errors")
        ax.set_xlabel("Frame index")
        ax.legend(loc="upper left")
        ax2.legend(loc="upper right")
    ax.set_title("SNR and chip errors per frame")

    ax = fig.add_subplot(gs[1, 1])
    ax.axis("off")
    guide = (
        "How to read this suite\n"
        "─────────────────────\n"
        "01_*  TX coding: bytes → whitening → nibbles → Hamming →\n"
        "      interleave → Gray → chip indices.\n"
        "02_*  TX IQ: constant-envelope chirps; spectrogram shows\n"
        "      upchirp sweeps and SFD downchirp slope flip.\n"
        "03_*  RX capture: time / spectrogram / IQ / PSD of the\n"
        "      raw loopback stream.\n"
        "04_*  RX demod: dechirp tone, folded-chip peaks, preamble\n"
        "      drift (rate), sync/SFD peaks, data chip decisions.\n"
        "05_*  This summary.\n\n"
        "Digital vs OTA\n"
        "• Digital: FPGA TX→RX BB loopback; high SNR, low CFO,\n"
        "  residual ~1k ppm rate from DSI clocking still present.\n"
        "• OTA: RF path; lower SNR, AGC warmup, multipath/coupling;\n"
        "  same rate offset plus possible larger CFO."
    )
    ax.text(0.02, 0.98, guide, va="top", family="monospace", fontsize=8)

    path = out / "05_summary_dashboard.png"
    savefig(fig, path)

    # Master overview collage note file
    notes_path = out / "README_plots.txt"
    notes_path.write_text(
        f"""LoRa signal-chain plots — {ctx.label}
{'=' * 60}

Dump directory: {dump_dir}
Mode: {'OTA' if man.get('ota') else 'digital'}
SF{man['spreading_factor']} BW={man['bandwidth_hz']} sps={man['sps']}
Frames: {man.get('frames_found')}  payload OK: {man.get('frames_payload_ok')}

Stage notes
-----------
"""
        + "\n\n".join(f"[{k}]\n{v}" for k, v in STAGE_NOTES.items())
        + "\n"
    )
    return [str(path), str(notes_path)]


JOBS = [
    ("tx_coding", job_tx_coding),
    ("tx_iq", job_tx_iq),
    ("rx_capture", job_rx_capture),
    ("rx_demod", job_rx_demod),
    ("summary", job_summary),
]


def _run_named_job(name: str, dump_dir: str, out_dir: str, label: str) -> tuple[str, list[str]]:
    """Top-level worker so ProcessPoolExecutor can pickle it."""
    fn = dict(JOBS)[name]
    ctx = JobCtx(dump_dir=dump_dir, out_dir=out_dir, label=label)
    print(f"[{label}] start {name}", flush=True)
    paths = fn(ctx)
    print(f"[{label}] done  {name} ({len(paths)} files)", flush=True)
    return name, paths


def run_all(dump_dir: Path, out_dir: Path, label: str, jobs: list[str] | None, workers: int) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    selected = [n for n, _ in JOBS if jobs is None or n in jobs]

    # tx_coding must finish before rx_demod (writes _tx_chips.npy)
    first = [n for n in selected if n == "tx_coding"]
    deferred = [n for n in selected if n == "rx_demod"]
    rest = [n for n in selected if n not in ("tx_coding", "rx_demod")]

    print(f"=== {label}: {dump_dir} → {out_dir} ===", flush=True)
    dump_s, out_s = str(dump_dir), str(out_dir)
    all_paths: list[str] = []

    for name in first:
        all_paths.extend(_run_named_job(name, dump_s, out_s, label)[1])

    if rest:
        with ProcessPoolExecutor(max_workers=max(1, workers)) as ex:
            futs = [
                ex.submit(_run_named_job, name, dump_s, out_s, label) for name in rest
            ]
            for fut in as_completed(futs):
                _, paths = fut.result()
                all_paths.extend(paths)

    for name in deferred:
        all_paths.extend(_run_named_job(name, dump_s, out_s, label)[1])

    print(f"=== {label}: wrote {len(all_paths)} artifacts ===", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", type=Path, required=True, help="lora_loopback dump directory")
    ap.add_argument("--out", type=Path, default=None, help="plot output directory")
    ap.add_argument("--label", type=str, default=None)
    ap.add_argument("--jobs", nargs="*", default=None, help="subset: tx_coding tx_iq rx_capture rx_demod summary")
    ap.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()
    out = args.out or (args.dir / "plots")
    label = args.label or args.dir.name
    run_all(args.dir, out, label, args.jobs, args.workers)


if __name__ == "__main__":
    main()

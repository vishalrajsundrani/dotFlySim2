#!/usr/bin/env python3
"""Derive Gazebo IMU noise parameters from a recorded rosbag2 (MCAP).

Reads the raw gyro/accelerometer topics out of a bag, finds a stationary
segment, and characterises the sensor: bias, white-noise density, and -- if
the segment is long enough -- bias instability and correlation time via the
overlapping Allan deviation.

The bags we care about are tens of gigabytes of camera data wrapped around a
few megabytes of IMU.  Reading them with a general-purpose deserialiser is
hopeless, so this module speaks MCAP directly: it walks the summary section
for the chunk index, decompresses only chunks that carry a channel we asked
for, and then uses the per-channel message index to slice out those messages
without decoding anything else in the chunk.

Stdlib only, apart from numpy.  No ROS, no mcap package: this has to run on a
laptop with a USB stick plugged in, not inside the simulation container.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("numpy is required: apt install python3-numpy")


# --------------------------------------------------------------------------
# decompression
# --------------------------------------------------------------------------

def _zstd_decompress(data: bytes, usize: int) -> bytes:
    try:
        from compression import zstd  # Python 3.14+
        return zstd.decompress(data)
    except ImportError:
        pass
    try:
        import zstandard
        return zstandard.ZstdDecompressor().decompress(data, max_output_size=usize)
    except ImportError:
        raise SystemExit("zstd chunks need Python 3.14+ or the 'zstandard' package")


def _lz4_decompress(data: bytes, usize: int) -> bytes:
    try:
        import lz4.frame
        return lz4.frame.decompress(data)
    except ImportError:
        raise SystemExit("lz4 chunks need the 'lz4' package")


def decompress(compression: str, data: bytes, usize: int) -> bytes:
    if compression == "":
        return data
    if compression == "zstd":
        return _zstd_decompress(data, usize)
    if compression == "lz4":
        return _lz4_decompress(data, usize)
    raise SystemExit(f"unsupported chunk compression {compression!r}")


# --------------------------------------------------------------------------
# MCAP
# --------------------------------------------------------------------------

OP_SCHEMA, OP_CHANNEL, OP_MESSAGE = 0x03, 0x04, 0x05
OP_CHUNK, OP_MESSAGE_INDEX, OP_CHUNK_INDEX = 0x06, 0x07, 0x08
MAGIC = b"\x89MCAP0\r\n"


def _str(buf: bytes, off: int) -> tuple[str, int]:
    n, = struct.unpack_from("<I", buf, off)
    off += 4
    return buf[off:off + n].decode("utf-8", "replace"), off + n


class McapFile:
    """Random-access reader over one .mcap file, driven by its summary section."""

    def __init__(self, path: Path):
        self.path = path
        self.f = path.open("rb")
        if self.f.read(8) != MAGIC:
            raise SystemExit(f"{path}: not an MCAP file")
        self.channels: dict[int, str] = {}
        self.schemas: dict[int, str] = {}
        self.chunks: list[dict] = []
        self._read_summary()

    def close(self):
        self.f.close()

    def _read_summary(self):
        f = self.f
        f.seek(0, 2)
        size = f.tell()
        # footer = opcode(1) + length(8) + content(20), then the trailing magic
        f.seek(size - 8 - 20 - 9)
        op, _ = struct.unpack("<BQ", f.read(9))
        summary_start, summary_offset_start, _ = struct.unpack("<QQI", f.read(20))
        if summary_start == 0:
            raise SystemExit(f"{self.path}: no summary section; bag was not finalised")
        end = summary_offset_start or (size - 8 - 20 - 9)

        f.seek(summary_start)
        while f.tell() < end:
            hdr = f.read(9)
            if len(hdr) < 9:
                break
            op, ln = struct.unpack("<BQ", hdr)
            body = f.read(ln)
            if op == OP_CHANNEL:
                cid, sid = struct.unpack_from("<HH", body, 0)
                topic, _ = _str(body, 4)
                self.channels[cid] = topic
            elif op == OP_SCHEMA:
                sid, = struct.unpack_from("<H", body, 0)
                name, _ = _str(body, 2)
                self.schemas[sid] = name
            elif op == OP_CHUNK_INDEX:
                o = 0
                mst, met, cso, clen = struct.unpack_from("<QQQQ", body, o)
                o += 32
                milen, = struct.unpack_from("<I", body, o)
                o += 4
                idx_end = o + milen
                mi: dict[int, int] = {}
                while o < idx_end:
                    cid, off = struct.unpack_from("<HQ", body, o)
                    o += 10
                    mi[cid] = off
                o = idx_end
                o += 8  # message_index_length
                comp, o = _str(body, o)
                csize, usize = struct.unpack_from("<QQ", body, o)
                self.chunks.append({
                    "start": cso, "len": clen, "comp": comp,
                    "csize": csize, "usize": usize, "index": mi,
                    "t0": mst, "t1": met,
                })

    def _message_offsets(self, mi_offset: int) -> list[tuple[int, int]]:
        """Read a MessageIndex record: [(log_time, offset_into_chunk), ...]."""
        f = self.f
        f.seek(mi_offset)
        op, ln = struct.unpack("<BQ", f.read(9))
        if op != OP_MESSAGE_INDEX:
            raise SystemExit(f"{self.path}: expected MessageIndex at {mi_offset}, got op {op}")
        body = f.read(ln)
        o = 2  # channel_id
        n, = struct.unpack_from("<I", body, o)
        o += 4
        out = []
        end = o + n
        while o < end:
            t, off = struct.unpack_from("<QQ", body, o)
            o += 16
            out.append((t, off))
        return out

    def read_topics(self, wanted: set[str], progress=None):
        """Yield (topic, log_time_ns, payload) for the wanted topics only."""
        want_ids = {cid for cid, t in self.channels.items() if t in wanted}
        if not want_ids:
            return
        for ci, chunk in enumerate(self.chunks):
            hits = {cid: off for cid, off in chunk["index"].items() if cid in want_ids}
            if not hits:
                continue  # chunk carries none of our channels; never decompress it
            # message indexes live outside the chunk, so read them before paying
            # for decompression
            per_channel = {cid: self._message_offsets(off) for cid, off in hits.items()}
            if not any(per_channel.values()):
                continue
            self.f.seek(chunk["start"])
            op, ln = struct.unpack("<BQ", self.f.read(9))
            if op != OP_CHUNK:
                raise SystemExit(f"{self.path}: expected Chunk at {chunk['start']}, got op {op}")
            head = self.f.read(8 + 8 + 8 + 4)
            comp, _ = _str(self.f.read(4 + len(chunk["comp"])), 0)
            self.f.seek(chunk["start"] + 9 + 28 + 4 + len(comp))
            rec_len, = struct.unpack("<Q", self.f.read(8))
            raw = self.f.read(rec_len)
            data = decompress(comp, raw, chunk["usize"])
            for cid, entries in per_channel.items():
                topic = self.channels[cid]
                for log_time, off in entries:
                    op2, ln2 = struct.unpack_from("<BQ", data, off)
                    if op2 != OP_MESSAGE:
                        continue
                    body = data[off + 9: off + 9 + ln2]
                    # channel_id(2) sequence(4) log_time(8) publish_time(8)
                    yield topic, log_time, body[22:]
            if progress:
                progress(ci + 1, len(self.chunks))


# --------------------------------------------------------------------------
# CDR
# --------------------------------------------------------------------------

class Cdr:
    """Minimal CDR reader. Alignment is relative to the end of the 4-byte
    encapsulation header, which is why `base` is subtracted everywhere."""

    def __init__(self, buf: bytes):
        self.buf = buf
        eh = buf[0:4]
        self.little = eh[1] in (1, 3)
        self.base = 4
        self.off = 4

    def _align(self, n: int):
        rel = self.off - self.base
        pad = (-rel) % n
        self.off += pad

    def u32(self) -> int:
        self._align(4)
        v, = struct.unpack_from("<I" if self.little else ">I", self.buf, self.off)
        self.off += 4
        return v

    def i32(self) -> int:
        self._align(4)
        v, = struct.unpack_from("<i" if self.little else ">i", self.buf, self.off)
        self.off += 4
        return v

    def f64(self) -> float:
        self._align(8)
        v, = struct.unpack_from("<d" if self.little else ">d", self.buf, self.off)
        self.off += 8
        return v

    def string(self) -> str:
        n = self.u32()
        s = self.buf[self.off:self.off + n]
        self.off += n
        return s.rstrip(b"\x00").decode("utf-8", "replace")

    def header(self) -> float:
        sec = self.i32()
        nsec = self.u32()
        self.string()  # frame_id
        return sec + nsec * 1e-9

    def vec3(self) -> tuple[float, float, float]:
        return self.f64(), self.f64(), self.f64()


def decode_vector3_stamped(buf: bytes):
    c = Cdr(buf)
    t = c.header()
    return t, c.vec3()


def decode_accel_stamped(buf: bytes):
    c = Cdr(buf)
    t = c.header()
    linear = c.vec3()
    return t, linear


def decode_imu(buf: bytes):
    c = Cdr(buf)
    t = c.header()
    for _ in range(4):
        c.f64()        # orientation
    for _ in range(9):
        c.f64()        # orientation_covariance
    gyro = c.vec3()
    for _ in range(9):
        c.f64()
    accel = c.vec3()
    return t, gyro, accel


DECODERS = {
    "geometry_msgs/msg/Vector3Stamped": decode_vector3_stamped,
    "geometry_msgs/msg/AccelStamped": decode_accel_stamped,
}


# --------------------------------------------------------------------------
# collection
# --------------------------------------------------------------------------

GYRO_TOPIC = "/wrapper/psdk_ros2/angular_rate_body_raw"
ACCEL_TOPIC = "/wrapper/psdk_ros2/acceleration_body_raw"
IMU_TOPIC = "/wrapper/psdk_ros2/imu"
COMPARE_TOPICS = {
    "/wrapper/psdk_ros2/acceleration_body_fused": "accel",
    "/wrapper/psdk_ros2/acceleration_ground_fused": "accel",
    "/wrapper/psdk_ros2/angular_rate_ground_fused": "v3",
}


def collect(bag: Path, extra: bool = True, verbose: bool = True) -> dict:
    """Pull the IMU series out of every .mcap in a bag directory."""
    if bag.is_dir():
        files = sorted(bag.glob("*.mcap"))
    else:
        files = [bag]
    if not files:
        raise SystemExit(f"{bag}: no .mcap files")

    # The simulation does not publish acceleration_body_raw at all: it is in
    # the bridge's SYNTHESIS_GAP, because nothing in PX4 corresponds to DJI's
    # raw accelerometer surface.  What it does publish is `imu`, carrying an
    # accelerometer and gyro in one sensor_msgs/Imu, so `imu` is read as well
    # and stands in for whichever raw topic is absent.  That is what lets one
    # tool measure both a real bag and a simulated one.
    #
    # The stand-in is sound for noise work, but NOT because the topics carry
    # the same messages -- they do not.  On the physical bag `imu` and the raw
    # topics have different counts (7988 vs 7989 vs 7981) and stamps a median
    # 5 ms apart, so they are separate samplings of the sensor, not copies.
    # What was checked is the thing this tool actually uses: over the same
    # stationary segment the noise densities agree to within 2% on all six
    # axes (gyro 7.84 vs 7.93e-5, accel z 1.314 vs 1.322e-3).  Densities are
    # interchangeable here; individual samples are not, so do not use this
    # substitution for anything time-aligned.
    kinds = {GYRO_TOPIC: "v3", ACCEL_TOPIC: "accel", IMU_TOPIC: "imu"}
    if extra:
        kinds.update(COMPARE_TOPICS)
    series: dict[str, list] = {t: [] for t in kinds}

    total_bytes = sum(f.stat().st_size for f in files)
    done_bytes = 0
    t_start = time.time()
    for f in files:
        mf = McapFile(f)
        if verbose:
            print(f"  {f.name}  ({f.stat().st_size/1e9:.2f} GB, {len(mf.chunks)} chunks)",
                  file=sys.stderr)

        def prog(i, n, _f=f, _done=done_bytes, _tot=total_bytes):
            if i % 200:
                return
            frac = (_done + _f.stat().st_size * i / n) / _tot
            el = time.time() - t_start
            eta = el / frac - el if frac > 0.01 else 0
            print(f"\r    {frac*100:5.1f}%  elapsed {el:5.0f}s  eta {eta:5.0f}s",
                  end="", file=sys.stderr, flush=True)

        for topic, log_time, payload in mf.read_topics(set(kinds), prog if verbose else None):
            kind = kinds[topic]
            if kind == "v3":
                t, v = decode_vector3_stamped(payload)
            elif kind == "accel":
                t, v = decode_accel_stamped(payload)
            else:
                t, gy, ac = decode_imu(payload)
                series[topic].append((t, log_time * 1e-9, (gy, ac)))
                continue
            series[topic].append((t, log_time * 1e-9, v))
        mf.close()
        done_bytes += f.stat().st_size
        if verbose:
            print(file=sys.stderr)

    out = {}
    imu_rows = series.pop(IMU_TOPIC, [])
    for topic, rows in series.items():
        if not rows:
            continue
        rows.sort(key=lambda r: r[1])
        out[topic] = {
            "stamp": np.array([r[0] for r in rows]),
            "log_time": np.array([r[1] for r in rows]),
            "xyz": np.array([r[2] for r in rows]),
        }
    if imu_rows:
        imu_rows.sort(key=lambda r: r[1])
        stamp = np.array([r[0] for r in imu_rows])
        log_t = np.array([r[1] for r in imu_rows])
        gyro = np.array([r[2][0] for r in imu_rows])
        accel = np.array([r[2][1] for r in imu_rows])
        out[IMU_TOPIC] = {"stamp": stamp, "log_time": log_t, "xyz": gyro,
                          "accel": accel}
        for topic, arr in ((GYRO_TOPIC, gyro), (ACCEL_TOPIC, accel)):
            if topic not in out:
                print(f"  {topic} absent; standing in from {IMU_TOPIC}",
                      file=sys.stderr)
                out[topic] = {"stamp": stamp, "log_time": log_t, "xyz": arr}
    return out


def save_cache(path: Path, data: dict) -> None:
    arrays = {}
    for topic, d in data.items():
        key = topic.rsplit("/", 1)[-1]
        arrays[f"{key}__stamp"] = d["stamp"]
        arrays[f"{key}__log_time"] = d["log_time"]
        arrays[f"{key}__xyz"] = d["xyz"]
        if "accel" in d:
            arrays[f"{key}__accel"] = d["accel"]
    np.savez_compressed(path, topics=np.array(sorted(data), dtype=object), **arrays)


def load_cache(path: Path) -> dict:
    z = np.load(path, allow_pickle=True)
    out = {}
    for topic in z["topics"]:
        key = str(topic).rsplit("/", 1)[-1]
        out[str(topic)] = {
            "stamp": z[f"{key}__stamp"],
            "log_time": z[f"{key}__log_time"],
            "xyz": z[f"{key}__xyz"],
        }
        if f"{key}__accel" in z:
            out[str(topic)]["accel"] = z[f"{key}__accel"]
    return out


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def sample_rate(t: np.ndarray) -> tuple[float, float]:
    """Publication rate, measured only over intervals the link did not drop.

    Three estimators disagree here and only one is right.  message_count over
    duration says 43 Hz, because this recording lost 23 s to three write
    stalls.  The median interval says 52 Hz, because the interval distribution
    is skewed -- a long right tail of late packets and a hard floor at the
    sensor period pulls the median below the mean.  The mean over intervals
    that are not dropouts says 49.8 Hz, which is the 50 Hz the aircraft
    actually runs at.  Gaps are excluded at 3x the rough period rather than a
    fixed threshold so this still holds for a differently-paced topic.
    """
    dt = np.diff(t)
    dt = dt[dt > 0]
    if dt.size == 0:
        return float("nan"), float("nan")
    rough = float(np.median(dt))
    keep = dt[dt < 3.0 * rough]
    if keep.size == 0:
        keep = dt
    return 1.0 / float(np.mean(keep)), float(np.std(keep))


def quiet_profile(gyro: np.ndarray, t: np.ndarray, rate: float,
                  probe_s: float = 1.0):
    """Per-axis gyro standard deviation in a short sliding probe window."""
    w = max(8, int(probe_s * rate))
    step = max(1, w // 2)
    idx, prof = [], []
    for start in range(0, max(1, gyro.shape[0] - w), step):
        idx.append(start)
        prof.append(float(np.sum(np.std(gyro[start:start + w], axis=0))))
    return np.array(idx), np.array(prof), w


def find_stationary(gyro: np.ndarray, accel: np.ndarray, t: np.ndarray,
                    min_window_s: float, probe_s: float = 1.0,
                    threshold_factor: float = 3.0) -> dict:
    """Locate the longest run during which the aircraft was genuinely still.

    The first version of this took a fixed-length window and returned the
    least-bad one.  On a bag whose only still moments are the 5 s before
    takeoff and the 3 s after landing, a 10 s window cannot fit in either, so
    it returned a window half full of climb-out and reported a gyro noise
    figure 70x too large with every sign of being wrong (lag-1 autocorrelation
    0.95) treated as just another number.  Returning a confidently wrong
    answer is worse than returning none.

    So the window is no longer assumed.  A short probe window sweeps the
    flight to build a noise profile, "still" is defined relative to the
    quietest probe seen, and the longest contiguous run of still probes
    becomes the segment.  If that run is shorter than the caller is willing
    to accept, this raises instead of improvising.
    """
    rate, _ = sample_rate(t)
    idx, prof, w = quiet_profile(gyro, t, rate, probe_s)
    if prof.size == 0:
        raise SystemExit("not enough samples to profile")
    floor = float(np.min(prof))
    threshold = floor * threshold_factor
    still = prof <= threshold

    runs, start = [], None
    for i, s_ in enumerate(still):
        if s_ and start is None:
            start = i
        elif not s_ and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(still) - 1))
    if not runs:
        raise SystemExit("no stationary period found anywhere in this bag")

    spans = []
    for a, b in runs:
        i0 = int(idx[a])
        i1 = int(idx[b]) + w
        i1 = min(i1, gyro.shape[0])
        spans.append((i0, i1, float(t[i1 - 1] - t[i0])))
    spans.sort(key=lambda r: -r[2])
    i0, i1, length = spans[0]

    if length < min_window_s:
        detail = ", ".join(f"{s[2]:.1f}s at t+{t[s[0]] - t[0]:.1f}s" for s in spans[:4])
        raise SystemExit(
            f"longest stationary run is {length:.1f}s but {min_window_s:.1f}s was "
            f"required.\n  stationary runs found: {detail}\n"
            f"  re-run with --window {math.floor(length)} to accept the "
            f"longest run, or --segment START:END to force one.")

    return {
        "start_index": i0,
        "window_samples": i1 - i0,
        "t_offset": float(t[i0] - t[0]),
        "gyro_var": float(np.sum(np.var(gyro[i0:i1], axis=0))),
        "accel_var": float(np.sum(np.var(accel[i0:i1], axis=0))),
        "threshold": threshold,
        "probe_floor": floor,
        "runs": [(float(t[a] - t[0]), float(t[b - 1] - t[0]), ln) for a, b, ln in spans],
        "scores": [(int(idx[i]), float(prof[i]), 0.0, float(t[idx[i]] - t[0]))
                   for i in range(len(idx))],
    }


def whiteness(x: np.ndarray) -> float:
    """Lag-1 autocorrelation of a detrended series.

    Near zero means successive samples are independent, so the per-sample
    standard deviation can legitimately be rescaled to another sample rate.
    Clearly positive means the signal was low-pass filtered before it reached
    us, and that rescaling would be a lie.
    """
    x = x - x.mean()
    if x.size < 3 or x.std() == 0:
        return float("nan")
    return float(np.dot(x[:-1], x[1:]) / np.dot(x, x))


def noise_density_psd(x: np.ndarray, rate: float,
                      band: tuple[float, float] = (0.35, 0.95)) -> dict:
    """Noise density from the flat top of the spectrum, not from total variance.

    Taking the standard deviation of a stationary segment assumes everything
    in it is sensor noise.  For the gyro that holds.  For the accelerometer it
    does not: a parked airframe sways on its legs in the wind, and on this bag
    two thirds of the x-axis power sits below 5 Hz.  That is real motion of a
    real aircraft, and folding it into the sensor noise figure would make the
    simulated accelerometer noisier than the part actually is.

    White noise is flat in frequency, so the density is read off the plateau
    between `band` fractions of Nyquist, where vehicle motion has died away.

    Returns the density and a redness ratio: how much the total-variance
    estimate exceeds the spectral one, i.e. how much of the segment was
    movement rather than noise.
    """
    n = x.size
    if n < 32:
        return {"density": float("nan"), "redness": float("nan"), "f": [], "psd": []}
    v = x - x.mean()
    w = np.hanning(n)
    f = np.fft.rfftfreq(n, 1.0 / rate)
    # one-sided PSD in units^2/Hz, corrected for the window's power loss
    psd = (2.0 / (rate * n)) * np.abs(np.fft.rfft(v * w)) ** 2 / np.mean(w ** 2)
    nyq = rate / 2.0
    sel = (f >= band[0] * nyq) & (f <= band[1] * nyq)
    if sel.sum() < 4:
        sel = f > 0
    level = float(np.median(psd[sel]))
    density = math.sqrt(max(level, 0.0))
    total_density = float(np.std(v, ddof=1)) / math.sqrt(rate)
    return {
        "density": density,
        "redness": (total_density / density) if density > 0 else float("nan"),
        "f": f.tolist(),
        "psd": psd.tolist(),
    }


def allan_deviation(x: np.ndarray, rate: float, n_tau: int = 40):
    """Overlapping Allan deviation of a rate signal."""
    n = x.size
    dt = 1.0 / rate
    theta = np.cumsum(x) * dt  # integrate rate -> angle
    max_m = (n - 1) // 3
    if max_m < 2:
        return np.array([]), np.array([])
    ms = np.unique(np.logspace(0, math.log10(max_m), n_tau).astype(int))
    taus, devs = [], []
    for m in ms:
        if n - 2 * m < 1:
            continue
        tau = m * dt
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        avar = float(np.sum(d * d)) / (2 * tau * tau * (n - 2 * m))
        if avar > 0:
            taus.append(tau)
            devs.append(math.sqrt(avar))
    return np.array(taus), np.array(devs)


def allan_fit(taus: np.ndarray, devs: np.ndarray) -> dict:
    """Pull the classic coefficients off an Allan curve.

    N (white noise) is read at tau=1s off the -1/2 slope; B (bias
    instability) is the floor, 0.664*min(sigma); the tau at that minimum is
    the correlation time.
    """
    out = {"N": float("nan"), "B": float("nan"), "tau_B": float("nan")}
    if taus.size < 4:
        return out
    lo = taus <= max(taus[0] * 4, 1.0)
    if lo.sum() >= 2:
        # sigma(tau) = N / sqrt(tau)  ->  N = sigma * sqrt(tau)
        out["N"] = float(np.median(devs[lo] * np.sqrt(taus[lo])))
    i = int(np.argmin(devs))
    out["B"] = float(devs[i] / 0.664)
    out["tau_B"] = float(taus[i])
    out["min_sigma"] = float(devs[i])
    out["at_edge"] = bool(i >= devs.size - 2)
    return out


# --------------------------------------------------------------------------
# the model file
# --------------------------------------------------------------------------

AXES = ("x", "y", "z")


def read_model_noise(sdf: Path) -> dict:
    """Scrape the current imu_sensor noise values so the report can show
    measured against in-use side by side."""
    import re
    text = sdf.read_text()
    m = re.search(r'<sensor name="imu_sensor".*?</sensor>', text, re.S)
    if not m:
        return {}
    block = m.group(0)
    out = {"update_rate": None}
    r = re.search(r"<update_rate>([\d.]+)</update_rate>", block)
    if r:
        out["update_rate"] = float(r.group(1))
    for group in ("angular_velocity", "linear_acceleration"):
        g = re.search(rf"<{group}>(.*?)</{group}>", block, re.S)
        if not g:
            continue
        for axis in AXES:
            a = re.search(rf"<{axis}>(.*?)</{axis}>", g.group(1), re.S)
            if not a:
                continue
            body = a.group(1)
            def grab(tag, default=0.0):
                mm = re.search(rf"<{tag}>([-\d.eE+]+)</{tag}>", body)
                return float(mm.group(1)) if mm else default
            out[f"{group}.{axis}"] = {
                "mean": grab("mean"),
                "stddev": grab("stddev"),
                "dynamic_bias_stddev": grab("dynamic_bias_stddev"),
                "dynamic_bias_correlation_time": grab("dynamic_bias_correlation_time"),
            }
    return out


def render_noise_block(stats: dict, indent: str = "          ") -> str:
    """Build the <angular_velocity>/<linear_acceleration> XML to paste back."""
    lines = []
    for group, key in (("angular_velocity", "gyro"), ("linear_acceleration", "accel")):
        lines.append(f"{indent}<{group}>")
        for axis in AXES:
            s = stats[key][axis]
            lines.append(f"{indent}  <{axis}>")
            lines.append(f'{indent}    <noise type="gaussian">')
            lines.append(f"{indent}      <mean>{s['mean']:.8g}</mean>")
            lines.append(f"{indent}      <stddev>{s['stddev_sim']:.8g}</stddev>")
            lines.append(f"{indent}      <dynamic_bias_stddev>{s['dynamic_bias_stddev']:.6g}</dynamic_bias_stddev>")
            lines.append(f"{indent}      <dynamic_bias_correlation_time>{s['dynamic_bias_correlation_time']:.6g}</dynamic_bias_correlation_time>")
            lines.append(f"{indent}    </noise>")
            lines.append(f"{indent}  </{axis}>")
        lines.append(f"{indent}</{group}>")
    return "\n".join(lines)


def patch_model(sdf: Path, stats: dict) -> None:
    """Replace the imu_sensor noise in place, keeping everything else."""
    import re
    text = sdf.read_text()
    m = re.search(r'(<sensor name="imu_sensor".*?<imu>)(.*?)(\s*</imu>)', text, re.S)
    if not m:
        raise SystemExit(f"{sdf}: could not find the imu_sensor <imu> block")
    new = "\n" + render_noise_block(stats) + "\n"
    out = text[:m.start(2)] + new + text[m.end(2):]
    sdf.write_text(out)


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------

def characterise(data: dict, window_s: float, sim_rate: float,
                 segment: str = "auto") -> dict:
    gyro_d = data[GYRO_TOPIC]
    accel_d = data[ACCEL_TOPIC]
    # the two topics are published independently; trim to a common span and
    # index them on their own stamps rather than assuming they interleave
    t_g, g = gyro_d["stamp"], gyro_d["xyz"]
    t_a, a = accel_d["stamp"], accel_d["xyz"]
    rate_g, jit_g = sample_rate(t_g)
    rate_a, jit_a = sample_rate(t_a)

    # resample accel onto the gyro clock so the stationary search sees one grid
    a_on_g = np.column_stack([np.interp(t_g, t_a, a[:, i]) for i in range(3)])

    if segment == "auto":
        st = find_stationary(g, a_on_g, t_g, window_s)
        i0, w = st["start_index"], st["window_samples"]
    else:
        lo, hi = (float(v) for v in segment.split(":"))
        i0 = int(np.searchsorted(t_g, t_g[0] + lo))
        w = int(np.searchsorted(t_g, t_g[0] + hi)) - i0
        st = {"start_index": i0, "window_samples": w,
              "t_offset": lo, "gyro_var": float(np.sum(np.var(g[i0:i0+w], axis=0))),
              "accel_var": float(np.sum(np.var(a_on_g[i0:i0+w], axis=0))), "scores": []}

    seg_t = t_g[i0:i0 + w]
    seg_g = g[i0:i0 + w]
    i0a = int(np.searchsorted(t_a, seg_t[0]))
    i1a = int(np.searchsorted(t_a, seg_t[-1]))
    seg_a = a[i0a:i1a]
    seg_ta = t_a[i0a:i1a]

    stats = {"gyro": {}, "accel": {}}
    for key, series, times, rate in (("gyro", seg_g, seg_t, rate_g),
                                     ("accel", seg_a, seg_ta, rate_a)):
        for i, axis in enumerate(AXES):
            x = series[:, i]
            sd = float(np.std(x, ddof=1))
            density_var = sd / math.sqrt(rate)
            spec = noise_density_psd(x, rate)
            # the spectral estimate is the one adopted; the variance-based one
            # is kept so the report can show what the difference cost
            density = spec["density"] if np.isfinite(spec["density"]) else density_var
            taus, devs = allan_deviation(x, rate)
            fit = allan_fit(taus, devs)
            stats[key][axis] = {
                "mean": float(np.mean(x)),
                "stddev_measured": sd,
                "rate_measured": rate,
                "density": density,
                "density_variance": density_var,
                "redness": spec["redness"],
                "psd_f": spec["f"],
                "psd": spec["psd"],
                "stddev_sim": density * math.sqrt(sim_rate),
                "lag1": whiteness(x),
                "allan_tau": taus.tolist(),
                "allan_dev": devs.tolist(),
                "allan_N": fit["N"],
                "allan_B": fit["B"],
                "allan_tau_B": fit["tau_B"],
                "allan_at_edge": fit.get("at_edge", False),
                "n": int(x.size),
            }
    return {
        "stationary": st,
        "stats": stats,
        "rate_gyro": rate_g, "rate_accel": rate_a,
        "jitter_gyro": jit_g, "jitter_accel": jit_a,
        "sim_rate": sim_rate,
        "duration": float(t_g[-1] - t_g[0]),
        "n_gyro": int(t_g.size), "n_accel": int(t_a.size),
        "segment_start_s": float(seg_t[0] - t_g[0]),
        "segment_len_s": float(seg_t[-1] - seg_t[0]),
    }


def calibrate_against(result: dict, sim_json: Path) -> list[str]:
    """Close the loop: correct the model for what the whole chain does to it.

    What goes into models/m4e/model.sdf is not what comes out of
    /wrapper/psdk_ros2/imu.  Gazebo generates noise at the sensor's 250 Hz,
    PX4 filters it into SensorCombined, and the bridge publishes at the
    aircraft's 50 Hz.  That last step is a decimation, and it is not
    anti-aliased: noise occupying 125 Hz of bandwidth folds into 25 Hz, so the
    density a consumer sees comes out close to sqrt(250/50) = 2.2x higher than
    the figure written into the SDF.  Measured on a 147 s simulated recording
    the factor is 1.85 to 1.94, uniform across all six axes -- mostly aliasing,
    partly undone by PX4's own low-pass.

    Guessing that factor from theory would be fragile, since it depends on
    PX4's filter settings and the bridge's rate cap.  So it is measured: run
    the simulation, characterise its published IMU with this same tool, and
    pass the result here.  Each axis is scaled by the ratio of what the
    aircraft publishes to what the simulation published, applied to the SDF
    value that was in force during that recording.
    """
    sim = json.loads(sim_json.read_text())
    notes = []
    worst = 0.0
    for key, group in (("gyro", "angular_velocity"), ("accel", "linear_acceleration")):
        for axis in AXES:
            tgt = result["stats"][key][axis]
            got = sim["result"]["stats"][key][axis]
            in_force = sim["current"].get(f"{group}.{axis}", {}).get("stddev")
            if not in_force or not got["density"]:
                tgt["loop_gain"] = float("nan")
                continue
            gain = tgt["density"] / got["density"]
            tgt["loop_gain"] = gain
            tgt["stddev_openloop"] = tgt["stddev_sim"]
            tgt["stddev_sim"] = in_force * gain
            tgt["sim_density_measured"] = got["density"]
            worst = max(worst, abs(math.log(gain)))
    notes.append(
        f"closed-loop correction applied from {sim_json.name}: the SDF values are "
        f"divided by what the Gazebo->PX4->bridge chain multiplies them by "
        f"(measured {1/math.exp(worst):.2f}x to {math.exp(worst):.2f}x), so the "
        f"PUBLISHED topic matches the aircraft rather than the SDF matching it")
    return notes


def apply_policy(result: dict, current: dict, accel_mean: str,
                 min_allan_s: float = 600.0) -> list[str]:
    """Decide which measured numbers we are entitled to use.

    A 3-minute bag can support bias and white noise honestly.  It cannot
    support bias instability or a correlation time of 100-300s, so those stay
    at their inherited values and we say so out loud rather than quietly
    shipping a fitted number that the data does not justify.
    """
    notes = []
    seg = result["segment_len_s"]
    allow_allan = seg >= min_allan_s
    if not allow_allan:
        notes.append(
            f"stationary segment is {seg:.0f}s; bias instability and correlation "
            f"time need >={min_allan_s:.0f}s, so those two stay at their inherited values")
    for key, group in (("gyro", "angular_velocity"), ("accel", "linear_acceleration")):
        for axis in AXES:
            s = result["stats"][key][axis]
            cur = current.get(f"{group}.{axis}", {})
            if allow_allan and not s["allan_at_edge"]:
                s["dynamic_bias_stddev"] = s["allan_B"]
                s["dynamic_bias_correlation_time"] = s["allan_tau_B"]
                s["bias_source"] = "measured"
            else:
                s["dynamic_bias_stddev"] = cur.get("dynamic_bias_stddev", 0.0)
                s["dynamic_bias_correlation_time"] = cur.get("dynamic_bias_correlation_time", 100.0)
                s["bias_source"] = "inherited"
            if key == "accel" and accel_mean == "zero":
                s["mean"] = 0.0
                s["mean_source"] = "forced to zero"
            else:
                s["mean_source"] = "measured"
            s["stddev_current"] = cur.get("stddev")
            s["mean_current"] = cur.get("mean")
            s["dbs_current"] = cur.get("dynamic_bias_stddev")
            s["dbct_current"] = cur.get("dynamic_bias_correlation_time")
    if accel_mean == "zero":
        notes.append(
            "accelerometer mean forced to zero: the bag's acceleration_body_raw is "
            "already gravity-compensated by the flight controller, so its mean mixes "
            "sensor bias with gravity-removal error and cannot be used as a bias")
    red = {f"{k}.{a}": result["stats"][k][a]["redness"]
           for k in ("gyro", "accel") for a in AXES}
    loud = {k: v for k, v in red.items() if v > 1.5}
    if loud:
        notes.append(
            "noise density is taken from the high-frequency plateau of the spectrum, "
            "not from the segment variance, because "
            + ", ".join(f"{k} carries {v:.1f}x more total power than white noise alone"
                        for k, v in sorted(loud.items(), key=lambda kv: -kv[1]))
            + " -- low-frequency content of a parked airframe is real motion, not sensor noise")
    # Whether rescaling to the simulation rate is legitimate is a question
    # about the top of the spectrum, not about lag-1 autocorrelation.  A
    # red-but-unfiltered signal (an airframe swaying) has high lag-1 and a
    # perfectly flat plateau; a filtered one rolls off.  Only the second case
    # invalidates the rescale, so test for roll-off directly.
    rolled = {}
    for k in ("gyro", "accel"):
        for a in AXES:
            s = result["stats"][k][a]
            f = np.asarray(s["psd_f"])
            p = np.asarray(s["psd"])
            if f.size < 8:
                continue
            nyq = f[-1]
            mid = p[(f >= 0.15 * nyq) & (f <= 0.35 * nyq)]
            top = p[(f >= 0.60 * nyq) & (f <= 0.95 * nyq)]
            if mid.size < 2 or top.size < 2:
                continue
            ratio = float(np.median(top) / np.median(mid))
            s["rolloff"] = ratio
            if ratio < 0.5:
                rolled[f"{k}.{a}"] = ratio
    if rolled:
        notes.append(
            "the spectrum rolls off before Nyquist on "
            + ", ".join(f"{k} (top/mid power {v:.2f})" for k, v in rolled.items())
            + ": that channel was low-pass filtered before recording, so its density "
              "is a lower bound and the simulation will be slightly quieter than the aircraft")
    return notes


def _fmt(v):
    return "   --    " if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:9.3g}"


def report(result: dict, current: dict, notes: list[str]) -> None:
    r = result
    print()
    print("=" * 78)
    print("  IMU characterisation")
    print("=" * 78)
    print(f"  bag duration         {r['duration']:.1f} s   "
          f"({r['n_gyro']} gyro / {r['n_accel']} accel messages)")
    print(f"  measured rate        gyro {r['rate_gyro']:.2f} Hz, accel {r['rate_accel']:.2f} Hz")
    print(f"  simulation rate      {r['sim_rate']:.0f} Hz "
          f"(noise rescaled by sqrt({r['sim_rate']:.0f}/{r['rate_gyro']:.1f}) "
          f"= {math.sqrt(r['sim_rate']/r['rate_gyro']):.2f})")
    print(f"  stationary segment   t+{r['segment_start_s']:.1f}s .. "
          f"t+{r['segment_start_s']+r['segment_len_s']:.1f}s  "
          f"({r['segment_len_s']:.1f} s, gyro variance {r['stationary']['gyro_var']:.3e})")
    print()
    hdr = f"  {'':26s} {'measured':>9s} {'density':>9s} {'-> sim':>9s} {'current':>9s} {'ratio':>7s} {'lag1':>6s} {'red':>6s} {'loop':>6s}"
    for key, label, unit in (("gyro", "angular_velocity", "rad/s"),
                             ("accel", "linear_acceleration", "m/s^2")):
        print(f"  {label}  [{unit}]")
        print(hdr)
        for axis in AXES:
            s = r["stats"][key][axis]
            cur = s["stddev_current"]
            ratio = (s["stddev_sim"] / cur) if cur else float("nan")
            print(f"    {axis}  stddev{'':14s}{_fmt(s['stddev_measured'])}"
                  f"{_fmt(s['density'])}{_fmt(s['stddev_sim'])}{_fmt(cur)}"
                  f"{'   --  ' if math.isnan(ratio) else f'{ratio:7.2f}'}"
                  f"{s['lag1']:6.2f}{s['redness']:7.2f}"
                  + (f"{s['loop_gain']:7.2f}" if "loop_gain" in s else ""))
        for axis in AXES:
            s = r["stats"][key][axis]
            print(f"    {axis}  mean{'':16s}{_fmt(s['mean'])}{'':9s}{'':9s}"
                  f"{_fmt(s['mean_current'])}   ({s['mean_source']})")
        for axis in AXES:
            s = r["stats"][key][axis]
            print(f"    {axis}  dyn_bias_stddev{'':5s}{_fmt(s['dynamic_bias_stddev'])}"
                  f"{'':9s}{'':9s}{_fmt(s['dbs_current'])}   ({s['bias_source']})")
        print()
    if notes:
        print("  caveats")
        for n in notes:
            print(f"    - {n}")
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path, help="rosbag2 directory or single .mcap")
    ap.add_argument("--model", type=Path,
                    default=Path(__file__).resolve().parent.parent / "models/m4e/model.sdf")
    ap.add_argument("--window", type=float, default=10.0,
                    help="length of the stationary window to search for, seconds")
    ap.add_argument("--segment", default="auto",
                    help="force a segment as START:END seconds instead of searching")
    ap.add_argument("--sim-rate", type=float, default=None,
                    help="simulation IMU rate; defaults to the model's update_rate")
    ap.add_argument("--accel-mean", choices=("zero", "measured"), default="zero")
    ap.add_argument("--min-allan", type=float, default=600.0,
                    help="stationary seconds required before trusting Allan results")
    ap.add_argument("--emit", action="store_true", help="write the result into --model")
    ap.add_argument("--json", type=Path, help="dump everything for the report builder")
    ap.add_argument("--calibrate", type=Path, metavar="SIM_JSON",
                    help="a --json from a SIMULATION recording; corrects the emitted "
                         "values for what the Gazebo/PX4/bridge chain does to them")
    ap.add_argument("--cache", type=Path,
                    help="decoded-series cache; written on first run, reused after")
    ap.add_argument("--refresh", action="store_true",
                    help="ignore an existing --cache and re-read the bag")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    current = read_model_noise(args.model) if args.model.exists() else {}
    sim_rate = args.sim_rate or current.get("update_rate") or 250.0

    # Reading the physical bag means pulling 15 GB off a USB stick and
    # decompressing every chunk, which is ~100 s.  The analysis gets iterated
    # on far more often than the bag changes, so the decoded series is cached
    # and reused; only the (small) IMU channels are ever in it.
    if args.cache and args.cache.exists() and not args.refresh:
        print(f"reusing cached series from {args.cache}", file=sys.stderr)
        data = load_cache(args.cache)
    else:
        print(f"reading {args.bag}", file=sys.stderr)
        data = collect(args.bag, verbose=not args.quiet)
        if args.cache:
            save_cache(args.cache, data)
            print(f"cached series to {args.cache}", file=sys.stderr)
    if GYRO_TOPIC not in data or ACCEL_TOPIC not in data:
        raise SystemExit("bag has no raw gyro/accelerometer topics")

    result = characterise(data, args.window, sim_rate, args.segment)
    notes = apply_policy(result, current, args.accel_mean, args.min_allan)
    if args.calibrate:
        notes = calibrate_against(result, args.calibrate) + notes
    report(result, current, notes)

    print("  SDF block")
    print()
    print(render_noise_block(result["stats"]))
    print()

    if args.emit:
        patch_model(args.model, result["stats"])
        print(f"  wrote {args.model}")

    if args.json:
        payload = {
            "bag": str(args.bag),
            "model": str(args.model),
            "notes": notes,
            "current": current,
            "result": {k: v for k, v in result.items() if k != "stationary"},
            "stationary": {k: v for k, v in result["stationary"].items() if k != "scores"},
            "scores": result["stationary"]["scores"],
            "raw": {
                topic: {
                    "t": (d["stamp"] - d["stamp"][0]).tolist(),
                    "xyz": d["xyz"].tolist(),
                }
                for topic, d in data.items()
            },
        }
        args.json.write_text(json.dumps(payload))
        print(f"  wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

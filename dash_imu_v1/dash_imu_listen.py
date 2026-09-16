#!/usr/bin/env python3
"""Subscribe to the dash_imu_v1 board and print what it sends.

    python3 dash_imu_listen.py           # one summary line per second
    python3 dash_imu_listen.py --raw     # every packet
    python3 dash_imu_listen.py --rpy     # live roll/pitch/yaw gauges
    python3 dash_imu_listen.py --rc      # live table of every RC channel
    python3 dash_imu_listen.py --rc --raw   # every RC packet, one per line

Keeps the subscription alive with a hello a few times a second, so the board
streams to this process's own port rather than to the compiled-in default.
That means several of these can run at once, and none of them needs the host to
be at a particular address.
"""

import argparse
import math
import socket
import struct
import sys
import time

MAGIC_DATA = 0xDA61
MAGIC_HELLO = 0xDA62
MAGIC_RC = 0xDA63
VERSION = 1
BOARD = ("192.168.0.113", 8006)

# Must match DashImuPacket exactly: <HBB I I H H 4f 3f 3f B = 57 bytes.
PACKET = struct.Struct("<HBBIIHH4f3f3fB")
HELLO = struct.Struct("<HBBB")
# Must match DashRcPacket exactly: <HBB I I B H H 16H B = 50 bytes.
RC_PACKET = struct.Struct("<HBBIIBHH16HB")

RC_FLAG_LOST_FRAME = 1 << 0
RC_FLAG_FAILSAFE = 1 << 1
RC_FLAG_CH17 = 1 << 2
RC_FLAG_CH18 = 1 << 3
RC_FLAG_NO_SIGNAL = 1 << 4
RC_FLAG_NAMES = ((RC_FLAG_LOST_FRAME, "lost"), (RC_FLAG_FAILSAFE, "failsafe"),
                 (RC_FLAG_CH17, "ch17"), (RC_FLAG_CH18, "ch18"),
                 (RC_FLAG_NO_SIGNAL, "no-signal"))

# FrSky SBUS endpoints for -100 / +100 %, and the lab's stick scaling.
SBUS_MIN, SBUS_MAX = 172, 1811

# What the lab's host parser (VN100UDPBridge.cpp) assumed each channel was.
# Set by the model's mixer on the Taranis X7, so unverified on Dash's radio.
LAB_RC_NAMES = {
    0: "left stick [1]",
    1: "right stick [0]",
    2: "right stick [1]",
    3: "left stick [0]",
    4: "left lower-left switch",
    5: "left lower-right switch",
    6: "left upper switch",
    7: "right lower-left switch",
    8: "right lower-right switch",
    9: "right upper switch",
    10: "knob [0]",
    11: "knob [1]",
}

HELLO_PERIOD_S = 0.2
RPY_REDRAW_HZ = 20.0

# The VN-100 is bolted to Dash's torso upside down, so the axes it reports are
# the torso's axes turned over. Each entry is that fixed mounting rotation as a
# body->sensor quaternion (x, y, z, w, scalar LAST); everything is reported
# back in body frame as q_body = q_mount^-1 (x) q_imu.
#
# flip-x and flip-y both put +z back down and are indistinguishable from
# gravity alone -- they differ by 180 degrees of yaw, i.e. by which way the
# torso calls "forward". See the README for how to tell them apart.
MOUNTS = {
    "none": (0.0, 0.0, 0.0, 1.0),
    "flip-x": (1.0, 0.0, 0.0, 0.0),   # 180 deg about body +x
    "flip-y": (0.0, 1.0, 0.0, 0.0),   # 180 deg about body +y
}
DEFAULT_MOUNT = "flip-x"


def crc8(data):
    """CRC-8, Dallas/Maxim poly 0x31 -- dash_imu_crc8() in the firmware."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def make_hello():
    body = HELLO.pack(MAGIC_HELLO, VERSION, 0, 0)[:-1]
    return body + bytes([crc8(body)])


def quat_conj(q):
    """Inverse of a unit quaternion (x, y, z, w)."""
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    """Hamilton product, so that R(a (x) b) == R(a) R(b)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    """Apply the rotation R(q) to a 3-vector."""
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def quat_to_rpy(q):
    """(x, y, z, w) -- scalar LAST, as the VN-100 sends it -- to degrees.

    ZYX intrinsic (yaw, then pitch, then roll), which is the convention the
    VN-100's own YawPitchRoll output uses, so these match what you would read
    off VectorNav's Control Center.
    """
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clamp: rounding can push this a hair past 1 near straight up/down.
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def bar(value, limit, width=41):
    """Centered gauge: '|' is zero, '#' is the value, clipped at +/- limit."""
    cells = ["-"] * width
    half = width // 2
    cells[half] = "|"
    cells[max(0, min(width - 1, half + int(round(value / limit * half))))] = "#"
    return "".join(cells)


def compass(yaw_deg, width=48):
    """0..360 strip with N/E/S/W ticks and '#' at the current heading."""
    cells = ["-"] * width
    for deg, letter in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        cells[int(deg / 360.0 * width)] = letter
    cells[int(yaw_deg % 360.0 / 360.0 * width) % width] = "#"
    return "".join(cells)


def parse(data):
    if len(data) != PACKET.size:
        return None
    if crc8(data[:-1]) != data[-1]:
        return None
    f = PACKET.unpack(data)
    if f[0] != MAGIC_DATA or f[1] != VERSION:
        return None
    # f = (magic, version, status, seq, teensy_us, bad_crc, resyncs,
    #      qx, qy, qz, qw, gx, gy, gz, ax, ay, az, crc)
    return {
        "status": f[2],
        "seq": f[3],
        "teensy_us": f[4],
        "bad_crc": f[5],
        "resyncs": f[6],
        "quat": f[7:11],   # x, y, z, w -- scalar LAST
        "gyro": f[11:14],
        "accel": f[14:17],
    }


def parse_rc(data):
    if len(data) != RC_PACKET.size:
        return None
    if crc8(data[:-1]) != data[-1]:
        return None
    f = RC_PACKET.unpack(data)
    if f[0] != MAGIC_RC or f[1] != VERSION:
        return None
    # f = (magic, version, status, seq, teensy_us, flags, lost_frames, age_ms,
    #      ch0 .. ch15, crc)
    return {
        "status": f[2],
        "seq": f[3],
        "teensy_us": f[4],
        "flags": f[5],
        "lost_frames": f[6],
        "age_ms": f[7],
        "ch": f[8:24],
    }


def rc_percent(raw):
    """Raw SBUS to -100..+100 %, the lab parser's scale_joystick() times 100."""
    return ((raw - SBUS_MIN) * 2.0 / (SBUS_MAX - SBUS_MIN) - 1.0) * 100.0


def rc_flag_names(flags):
    return ",".join(name for bit, name in RC_FLAG_NAMES if flags & bit) or "-"


def rc_status(rc):
    """What a controller should make of this packet -- dash_rc_usable()."""
    if rc["flags"] & RC_FLAG_FAILSAFE:
        return "FAILSAFE"
    if rc["flags"] & RC_FLAG_NO_SIGNAL:
        age = "never" if rc["age_ms"] == 0xFFFF else f"{rc['age_ms']} ms ago"
        return f"NO SIGNAL (last frame {age})"
    return "OK"


def format_rc_line(rc):
    return (f"{rc['seq']:8d}  {rc_status(rc):<9s}  flags {rc_flag_names(rc['flags']):<9s}  "
            f"lost {rc['lost_frames']:5d}  ch " +
            " ".join(f"{c:4d}" for c in rc["ch"]))


class RcView:
    """Live table of every RC channel under a status line saying whether to
    trust them. Redrawn in place, so it wants a terminal, not a pipe."""

    STALE_S = 0.5

    def __init__(self):
        self.rc = None
        self.last_rx = 0.0
        self.last_seq = None
        self.dropped = 0
        self.lo = [None] * 16
        self.hi = [None] * 16
        self.window_start = time.time()
        self.frames = 0
        self.hz = 0.0
        self.drawn = False

    def update(self, rc, now):
        if self.last_seq is not None and rc["seq"] != self.last_seq + 1:
            self.dropped += (rc["seq"] - self.last_seq - 1) & 0xFFFFFFFF
        self.last_seq = rc["seq"]
        self.rc = rc
        self.last_rx = now
        # Heartbeats repeat the last frame's channels; only real frames count
        # towards the rate and the min/max.
        if not rc["flags"] & RC_FLAG_NO_SIGNAL:
            self.frames += 1
            for i, v in enumerate(rc["ch"]):
                self.lo[i] = v if self.lo[i] is None else min(self.lo[i], v)
                self.hi[i] = v if self.hi[i] is None else max(self.hi[i], v)

    def draw(self, now):
        if now - self.window_start >= 1.0:
            self.hz = self.frames / (now - self.window_start)
            self.frames = 0
            self.window_start = now

        rc = self.rc
        if rc is None or now - self.last_rx > self.STALE_S:
            status = "NO RC PACKETS (board not streaming, or firmware without RC)"
        else:
            status = rc_status(rc)
        lost = rc["lost_frames"] if rc else 0
        flags = rc_flag_names(rc["flags"]) if rc else "-"
        lines = [
            f"RC {status}",
            f"   {self.hz:5.1f} frames/s   lost_frames {lost}   "
            f"dropped {self.dropped}   flags {flags}",
            f"ch    raw  -100%{'':11s}0{'':11s}+100%    pct   min   max  "
            f"lab assignment",
        ]
        for i in range(16):
            raw = rc["ch"][i] if rc else 0
            pct = rc_percent(raw) if rc else 0.0
            lo = "" if self.lo[i] is None else self.lo[i]
            hi = "" if self.hi[i] is None else self.hi[i]
            lines.append(f"ch{i:<2d} {raw:5d}  [{bar(pct, 100.0, 29)}] {pct:+5.0f}%"
                         f" {lo:>5} {hi:>5}  {LAB_RC_NAMES.get(i, '')}")

        if self.drawn:
            sys.stdout.write(f"\033[{len(lines)}A")
        sys.stdout.write("".join(line + "\033[K\n" for line in lines))
        sys.stdout.flush()
        self.drawn = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", action="store_true", help="print every packet")
    ap.add_argument("--rpy", action="store_true",
                    help="live roll/pitch/yaw gauges")
    ap.add_argument("--mount", default=DEFAULT_MOUNT, choices=sorted(MOUNTS),
                    help=f"how the VN-100 sits on the torso "
                         f"(default {DEFAULT_MOUNT}); 'none' reports the "
                         f"sensor frame exactly as it comes off the wire")
    ap.add_argument("--rc", action="store_true",
                    help="show the RC receiver instead of the IMU: a live "
                         "table of all 16 channels, or with --raw every "
                         "RC packet")
    ap.add_argument("--board", default=BOARD[0])
    ap.add_argument("--port", type=int, default=BOARD[1])
    args = ap.parse_args()
    if args.rc and args.rpy:
        ap.error("--rc and --rpy are separate views; pick one")
    rc_view = RcView() if args.rc and not args.raw else None

    # One inverse, once: every sample is reported in body frame from here on.
    q_mount_inv = quat_conj(MOUNTS[args.mount])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.settimeout(0.5)
    hello = make_hello()
    print(f"subscribing to {args.board}:{args.port} from port "
          f"{sock.getsockname()[1]}, mount {args.mount}", file=sys.stderr)

    last_hello = 0.0
    window_start = time.time()
    count = 0
    last_seq = None
    dropped = 0
    last_redraw = 0.0
    drawn = False

    while True:
        now = time.time()
        if now - last_hello > HELLO_PERIOD_S:
            sock.sendto(hello, (args.board, args.port))
            last_hello = now

        # Drawn on a clock rather than per packet: frames arrive at ~110 Hz,
        # and when none do at all, the table still has to say so.
        if rc_view is not None and now - last_redraw >= 1.0 / RPY_REDRAW_HZ:
            rc_view.draw(now)
            last_redraw = now

        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue

        # RC receiver packets share the stream with the IMU's.
        if data[:2] == struct.pack("<H", MAGIC_RC):
            if args.rc:
                rc = parse_rc(data)
                if rc is None:
                    print("bad RC packet", file=sys.stderr)
                elif rc_view is not None:
                    rc_view.update(rc, time.time())
                else:
                    print(format_rc_line(rc))
            continue
        if args.rc:
            continue

        pkt = parse(data)
        if pkt is None:
            print("bad packet", file=sys.stderr)
            continue

        if last_seq is not None and pkt["seq"] != last_seq + 1:
            # Gaps are real information: the board counts every sample it
            # parsed, so a gap here is a datagram that never arrived.
            dropped += pkt["seq"] - last_seq - 1
        last_seq = pkt["seq"]
        count += 1

        q = quat_mul(q_mount_inv, pkt["quat"])
        g = quat_rotate(q_mount_inv, pkt["gyro"])
        a = quat_rotate(q_mount_inv, pkt["accel"])
        roll, pitch, yaw = quat_to_rpy(q)

        if args.raw:
            print(f"{pkt['seq']:8d}  q {q[0]:+.4f} {q[1]:+.4f} {q[2]:+.4f} "
                  f"{q[3]:+.4f}  w {g[0]:+7.3f} {g[1]:+7.3f} {g[2]:+7.3f}  "
                  f"a {a[0]:+7.3f} {a[1]:+7.3f} {a[2]:+7.3f}  "
                  f"rpy {roll:+7.1f} {pitch:+7.1f} {yaw:+7.1f}")
        elif args.rpy:
            # The board streams at 200 Hz; redrawing that fast just melts the
            # terminal and is far past what an eye can follow.
            if now - last_redraw >= 1.0 / RPY_REDRAW_HZ:
                if drawn:
                    sys.stdout.write("\033[3A")
                sys.stdout.write(
                    f"roll  {roll:+7.1f} deg  [{bar(roll, 90.0)}]\033[K\n"
                    f"pitch {pitch:+7.1f} deg  [{bar(pitch, 90.0)}]\033[K\n"
                    f"yaw   {yaw:+7.1f} deg  [{compass(yaw)}]\033[K\n")
                sys.stdout.flush()
                last_redraw = now
                drawn = True
        elif now - window_start >= 1.0:
            flags = pkt["status"]
            print(f"{count / (now - window_start):6.1f} Hz  "
                  f"rpy {roll:+6.1f} {pitch:+6.1f} {yaw:+6.1f}  "
                  f"dropped {dropped:5d}  bad_crc {pkt['bad_crc']:5d}  "
                  f"resyncs {pkt['resyncs']:5d}  "
                  f"|a| {(a[0]**2 + a[1]**2 + a[2]**2) ** 0.5:5.2f} m/s^2  "
                  f"link {'y' if flags & 1 else 'n'}  "
                  f"sub {'y' if flags & 2 else 'n'}"
                  f"{'  BAD FORMAT' if flags & 4 else ''}")
            window_start = now
            count = 0


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

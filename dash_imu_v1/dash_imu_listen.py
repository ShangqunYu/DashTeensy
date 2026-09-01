#!/usr/bin/env python3
"""Subscribe to the dash_imu_v1 board and print what it sends.

    python3 dash_imu_listen.py           # one summary line per second
    python3 dash_imu_listen.py --raw     # every packet
    python3 dash_imu_listen.py --rpy     # live roll/pitch/yaw gauges

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
VERSION = 1
BOARD = ("192.168.0.113", 8006)

# Must match DashImuPacket exactly: <HBB I I H H 4f 3f 3f B = 57 bytes.
PACKET = struct.Struct("<HBBIIHH4f3f3fB")
HELLO = struct.Struct("<HBBB")

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", action="store_true", help="print every packet")
    ap.add_argument("--rpy", action="store_true",
                    help="live roll/pitch/yaw gauges")
    ap.add_argument("--mount", default=DEFAULT_MOUNT, choices=sorted(MOUNTS),
                    help=f"how the VN-100 sits on the torso "
                         f"(default {DEFAULT_MOUNT}); 'none' reports the "
                         f"sensor frame exactly as it comes off the wire")
    ap.add_argument("--board", default=BOARD[0])
    ap.add_argument("--port", type=int, default=BOARD[1])
    args = ap.parse_args()

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

        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
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

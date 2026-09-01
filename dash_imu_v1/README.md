# dash_imu_v1 — VN-100 IMU bridge

The torso's third Teensy 4.1. It reads the VN-100's binary output on **Serial2**
and forwards each sample to the host over UDP. No CAN, no motors, no control —
if this board resets, the arms do not notice.

Adapted from the lab's `vn100_imu_and_rc_teensy` sketch
(`~/Repositories/VN100-Teensy-Bridge`). The IMU half of that sketch was
reusable as-is; the RC/SBUS half is gone, and the transport was rewritten to
match the conventions of the two motor relay boards. See
[What changed](#what-changed-from-the-lab-sketch).

## Network

| Board | IP | Listens on | Sketch |
|---|---|---|---|
| left relay | 192.168.0.111 | 8004 | `dash_v1` (`DASH_BOARD_ID 1`) |
| right relay | 192.168.0.112 | 8004 | `dash_v1` (`DASH_BOARD_ID 2`) |
| **IMU** | **192.168.0.113** | **8006** | **`dash_imu_v1`** |
| host (upxtreme) | 192.168.0.110 | — | — |

There is only one of these boards, so unlike `dash_v1` there is no
`DASH_BOARD_ID` to set before flashing.

## Wiring

| | Teensy 4.1 pin |
|---|---|
| IMU TX → Teensy **RX2** | 7 |
| Teensy **TX2** → IMU RX | 8 |
| GND | any GND |

TX is only needed to *configure* the IMU (see below); streaming uses RX alone.

## Configuring the VN-100

**This must be done once, per IMU, before the bridge produces anything.** A
factory-default VN-100 emits ASCII `$VNYMR` sentences, which this sketch will
not parse — you get "no valid samples" on the console and a fast-blinking LED.

Set `DASH_IMU_CONFIG_PASSTHROUGH` to `1` at the top of the sketch, flash it, and
open the Arduino serial monitor at 115200 with **line ending = newline**. The
board is then a plain USB↔Serial2 wire and you are typing at the IMU. The `*XX`
on each line is VectorNav's "skip the checksum" wildcard.

```
$VNASY,0*XX                  stop async output while we reconfigure
$VNWRG,06,0*XX               turn off the ASCII async message
$VNWRG,75,2,4,01,0130*XX     binary output 1: port 2, 800/4 = 200 Hz,
                             Common group, quat + angular rate + accel
$VNCMD*XX                    enter command mode
system save                  commit to flash, so it survives a power cycle
exit
$VNASY,1*XX                  resume async output
```

Then set `DASH_IMU_CONFIG_PASSTHROUGH` back to `0` and reflash.

**`2` in register 75 is the IMU's serial port number, not the Teensy's.** If the
harness lands on the VN-100's port 1, that field must be `1` or the IMU will
stream out of a connector that is not plugged into anything.

**`system save` is not optional.** Without it the settings live in RAM and the
next power cycle silently returns a factory IMU emitting ASCII.

### Raising the baud rate — recommended

At the factory 115200, a 46-byte frame occupies **4.0 ms of the 5.0 ms sample
period** — 80% of the line. There is no room there for a retimed byte, and the
symptom of running out is a rising `resyncs` count rather than an error.

```
$VNWRG,05,460800*XX          then system save as above
```

and change `kImuBaud` in the sketch to match. That brings a frame down to
1.0 ms in 5.0. The IMU starts answering at the new rate immediately, so the
serial monitor goes quiet until you reopen it at 460800.

## The message

Register 75 as configured above produces, repeatedly:

```
0xFA | groups=0x01 | fields=0x0130 | quat[4] | gyro[3] | accel[3] | crc16
 1         1              2            16        12        12        2
```

`fields = 0x0130` is Quaternion (`0x0010`) + AngularRate (`0x0020`) +
Accel (`0x0100`) from the Common group. **The payload order is the order of the
field bits, low to high** — that, not the order they appear in the register
write, is what puts quat before gyro before accel.

`kVnGroups`, `kVnFields`, `kVnPayloadBytes` and `kVnFrameBytes` in the sketch
all describe this one message and must be changed together with the register.
The sketch checks the group/field bytes on every frame, so a mismatch is
reported on the console instead of being decoded as noise — the CRC alone will
not catch it, because a differently-configured IMU emits frames that pass their
*own* checksum perfectly well.

## Receiving it

The board does not stream into the void. A host announces itself with a 5-byte
`DashImuHello` and the Teensy streams `DashImuPacket` to that datagram's source
address **and source port**, until the hellos stop for 2 s. Same rule
`dash_v1` uses for its state packets, and for the same reason: several
processes can subscribe on their own ports with nothing hard-coded.

`DASH_IMU_AUTOSTREAM` (on by default) also sends to 192.168.0.110:8006 when
nobody has subscribed, so a bare listener works with no handshake at all. A
hello always takes precedence.

```
python3 dash_imu_listen.py            # subscribes, prints rate and values
python3 dash_imu_listen.py --raw      # every packet
python3 dash_imu_listen.py --rpy      # live roll/pitch/yaw gauges
python3 dash_imu_listen.py --mount none   # sensor frame, uncorrected
```

All three modes also print roll/pitch/yaw in degrees, derived on the host from
the quaternion by `quat_to_rpy()` (ZYX intrinsic, matching VectorNav's own
YawPitchRoll output). The board does not send Euler angles -- the quaternion is
the thing on the wire, because it has no gimbal lock and interpolates cleanly.

Two things to keep in mind when reading them:

- **Roll and pitch are gravity-referenced and trustworthy; yaw is not.** Yaw
  comes from the magnetometer, which sits inside a torso with ten BLDC motors
  in it. Verify it against a known heading before depending on it.
- **The angles describe the sensor's axes, not the torso's.** See below --
  the VN-100 is mounted upside down, and the listener corrects for it.

## Mounting rotation

The VN-100 is bolted to the torso **upside down**, so the frame it reports is
Dash's body frame turned over: with the torso upright and level it reads roll
180 deg and accel `z = +9.5` rather than roll 0 and `z = -9.5`.

That offset is corrected on the host, not in the IMU. `MOUNTS` in
`dash_imu_listen.py` holds the fixed body->sensor rotation and every sample is
reported as `q_body = q_mount^-1 (x) q_imu`, with gyro and accel rotated by the
same inverse so all three stay in one frame. Keeping it here rather than in the
VN-100's reference-frame-rotation register means the transform is greppable and
survives a sensor swap.

Correcting it on the host also keeps roll away from the +/-180 wrap, where the
angle jumps the full range on the slightest lean.

`--mount none` reports the sensor frame exactly as it comes off the wire, which
is what you want when debugging the link rather than reading the robot.

**flip-x vs flip-y.** Both put `+z` back down, and gravity alone cannot tell
them apart -- they differ by 180 deg of yaw, i.e. by which way the torso calls
forward. The default is `flip-x`. To confirm it against the hardware, tilt the
torso and check the sign:

| motion | expected |
|---|---|
| lean forward | pitch goes **negative** |
| lean right | roll goes **positive** |

If both come out backwards, it is `flip-y`. If exactly one is backwards, the
mounting is not a simple 180 deg flip and needs a rotation measured properly.

If you pipe the output anywhere, use `python3 -u` -- stdout is block-buffered
on a pipe, and a killed process discards the buffer.

`DashImuPacket` is 57 bytes, little-endian, CRC-8 sealed:

| field | | |
|---|---|---|
| `magic` | u16 | `0xDA61` |
| `version` | u8 | `1` |
| `status` | u8 | bit0 link, bit1 subscribed, bit2 bad format (sticky) |
| `seq` | u32 | one per *parsed* sample; gaps are drops |
| `teensy_us` | u32 | `micros()` at parse time |
| `bad_crc` | u16 | cumulative VN-100 frames failing CRC (saturates at 65535) |
| `resyncs` | u16 | cumulative times the parser had to hunt for the sync byte |
| `quat` | 4×f32 | **x, y, z, w — scalar last** |
| `gyro` | 3×f32 | rad/s, IMU body frame |
| `accel` | 3×f32 | m/s², IMU body frame |
| `crc` | u8 | CRC-8 over the preceding 56 bytes |

`bad_crc` and `resyncs` are the two numbers to watch: both should stay at 0 on a
healthy link, and both climbing together means the baud rate is too tight or the
cable is picking up noise from the motor harness.

**`quat` is scalar-last.** `Eigen::Quaternionf` takes `(w, x, y, z)`, so a
straight `memcpy` into one is wrong and produces a rotation that looks almost
plausible. Nothing here rotates the IMU frame into the robot frame either;
that transform belongs on the host, once the IMU's mounting is measured.

## LED

| | |
|---|---|
| solid | parsing and streaming |
| slow blink (1 Hz) | IMU fine, nothing subscribed (only with autostream off) |
| fast blink (5 Hz) | no valid samples — wiring, baud, or an unconfigured IMU |

## What changed from the lab sketch

- **SBUS/RC removed.** Dash has no RC receiver; `Serial7` and the `sbus` library
  dependency are gone with it.
- **Network moved** from the old lab `10.0.0.x` onto Dash's `192.168.0.x`.
- **ASCII → binary.** The original `sprintf("%f,%f,…")` cost ~130 bytes per
  sample against 57, could not distinguish a truncated datagram from a short
  one, and made the host re-parse floats it had just been handed.
- **Destination is no longer hard-coded** — see [Receiving it](#receiving-it).
- **The group/field header is checked**, so an unconfigured IMU is reported
  rather than misparsed.
- **Framing waits for a whole frame** to be buffered before consuming the sync
  byte, so `readBytes()` is a memcpy out of the ring buffer and can never block
  on the wire. The original called `readBytes(in, 45)` as soon as it saw the
  sync byte, which at 115200 can stall the loop for most of a sample period —
  with a 1 s default timeout if the frame never completes.
- **RX buffer enlarged** from the 64-byte default to 512. A frame plus sync is
  46 bytes, so the default left 18 bytes of slack, and an overrun shows up as a
  CRC failure rather than as anything that names the real cause.
- **Watchdog added**, matching the relay boards.
- The `delay(1)` in the original loop is gone; the frame-complete check does the
  same job without adding jitter.

Unchanged and deliberately so: the VN-100 CRC-16 routine and the payload offsets
(3 / 19 / 31), which are copied verbatim from a sketch known to work on the
previous robot.

## Host side

There is no DARoS-Core reader yet — `dash_imu_listen.py` is the only client.
When one is written, copy `DashImuProtocol.h` into
`DARoS-Core/Systems/DashSystem/RobotHardware/`, and **that copy becomes the
source of truth**, with this one kept byte-identical the way `DashUdpProtocol.h`
already is:

```
cp ~/Repositories/DARoS-Core/Systems/DashSystem/RobotHardware/DashImuProtocol.h \
   ~/Repositories/DashTeensy/dash_imu_v1/
```

Bump `kDashImuProtoVersion` on any layout change; both sides reject a mismatched
version, so a stale binary fails loudly instead of misparsing.

## Dependencies

- [QNEthernet](https://github.com/ssilverman/QNEthernet)
- Watchdog_t4 (bundled with Teensyduino)

Board: Teensy 4.1, 600 MHz. The `sbus` library the original needed is **not**
required.

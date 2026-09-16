# dash_imu_v1 — VN-100 IMU and RC receiver bridge

The torso's third Teensy 4.1. It reads the VN-100's binary output on **Serial2**
and the RC receiver's SBUS on **Serial7**, and forwards both to the host over
UDP. No CAN, no motors, no control — if this board resets, the arms do not
notice.

Adapted from the lab's `vn100_imu_and_rc_teensy` sketch
(`~/Repositories/VN100-Teensy-Bridge`). The IMU and SBUS parsing were reusable
as-is; the transport was rewritten to match the conventions of the two motor
relay boards. See [What changed](#what-changed-from-the-lab-sketch).

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
| X8R **SBUS** → Teensy **RX7** | 28 |
| GND (IMU and receiver) | any GND |

TX2 is only needed to *configure* the IMU (see below); streaming uses RX alone.

The receiver is a FrSky X8R bound to a Taranis X7 (ACCST D16). Use the X8R's
**SBUS** connector, not the numbered channel pins. SBUS is inverted serial;
the `sbus` library inverts it inside the Teensy UART, so no external inverter.
The X8R wants 4–10 V of power, and **Teensy 4.1 pins are not 5 V tolerant** —
check the SBUS signal level before connecting a receiver you have not used on a
Teensy before.

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

### Raising the baud rate — optional

At the factory 115200, a 46-byte frame occupies 4.0 ms of the 5.0 ms sample
period. That has not caused errors by itself — 0 in 12 000 samples, measured
with RC running — and the baud rate does not change how much buffer a stalled
`loop()` uses up, because the IMU delivers 9200 bytes/s at any baud. It is
headroom worth having if the sample rate goes up.

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
python3 dash_imu_listen.py --rc       # live table of every RC channel
python3 dash_imu_listen.py --rc --raw # every RC packet, one per line
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
healthy link. Neither counts until the first good frame, so joining the stream
mid-frame at power-up does not register. Both climbing together means bytes were
lost — a stalled `loop()` overflowing the Serial2 buffer, or noise on the
cable. See [IMU before RC](#imu-before-rc).

**`quat` is scalar-last.** `Eigen::Quaternionf` takes `(w, x, y, z)`, so a
straight `memcpy` into one is wrong and produces a rotation that looks almost
plausible. Nothing here rotates the IMU frame into the robot frame either;
that transform belongs on the host, once the IMU's mounting is measured.

## RC receiver

Each decoded SBUS frame goes out as a `DashRcPacket` to the same subscriber as
the IMU samples, so one hello gets both; a reader tells them apart by magic and
size. Once no frame has arrived for 100 ms, a heartbeat with `NoSignal` set goes
out every 100 ms instead, carrying the last channels received.

**Check `failsafe` and `NoSignal` both** before trusting a channel —
`dash_rc_usable()` does. On losing the radio the X8R either keeps sending
frames with `failsafe` set or stops sending altogether, depending on the
failsafe mode set for it on the X7; the second shows up as `NoSignal`.
`lost_frame` on its own is not a fault: the receiver repeats the last good
values for a missed radio frame, and the odd one is normal. `lost_frames`
counting up steadily is the number that means a poor link.

`DashRcPacket` is 50 bytes, little-endian, CRC-8 sealed:

| field | | |
|---|---|---|
| `magic` | u16 | `0xDA63` |
| `version` | u8 | `1` |
| `status` | u8 | same bits as `DashImuPacket` |
| `seq` | u32 | one per RC packet, heartbeats included; gaps are drops |
| `teensy_us` | u32 | `micros()` when the frame was decoded |
| `flags` | u8 | bit0 lost frame, bit1 failsafe, bit2 ch17, bit3 ch18, bit4 no signal |
| `lost_frames` | u16 | cumulative frames with lost frame set (saturates at 65535) |
| `age_ms` | u16 | since the last SBUS frame; 0 on a frame, `0xFFFF` if none ever |
| `ch` | 16×u16 | raw SBUS, 0..2047 |
| `crc` | u8 | CRC-8 over the preceding 49 bytes |

FrSky sends **172 / 992 / 1811** for −100 / 0 / +100 %. Which control is on
which channel is set by the model's mixer on the X7, not by anything here. The
lab's host parser (`VN100UDPBridge.cpp`) assumed the layout below; it has not
been checked against Dash's radio yet.

| ch | lab assignment |
|---|---|
| 3, 0 | left stick [0], [1] |
| 1, 2 | right stick [0], [1] |
| 4, 5, 6 | left lower-left, lower-right, upper switch |
| 7, 8, 9 | right lower-left, lower-right, upper switch |
| 10, 11 | knobs |

`python3 dash_imu_listen.py --rc` shows every channel live — raw value, a
−100..+100 % bar, the min and max seen since start, and the lab's name for it —
under a status line that reads `OK`, `FAILSAFE`, `NO SIGNAL` or
`NO RC PACKETS`. Move one control at a time and watch which row changes; sweep
each to both ends and check min/max land on 172 and 1811. The serial console
prints a one-line `[RC]` summary a second as well.

The board streams to one subscriber at a time, so while `--rc` runs, any other
reader of the IMU board (`DashImuBoard`, a second listener) takes turns with it.

## IMU before RC

Whenever the two compete, the IMU wins and RC waits:

- **Interrupts.** Serial2 (IMU) runs at NVIC priority 32, Serial7 (RC) at 96;
  both default to 64.
- **`loop()`.** RC is serviced only when no complete IMU frame is waiting. An
  SBUS frame left in Serial7's buffer costs nothing — the parser keeps the
  newest — while an IMU frame left waiting is a late sample.
- **Console output never blocks.** A line is written only if a terminal has the
  port open and the whole line fits in USB buffers that are already free;
  otherwise it is dropped.
- **Serial2's buffer is 4096 bytes**, 445 ms of `loop()` stall before a byte is
  lost.

The last two fix the only IMU errors actually seen. Measured 2026-09-14: 0
errors in 60 s with RC running, then exactly one `bad_crc` + `resyncs` about
5 s after each of four closes of the USB serial port. With nothing reading the
port the USB buffers fill, the next `Serial.printf()` waits its full 120 ms
timeout, and at 9200 bytes/s that overran the old 512-byte buffer. RC was not
involved.

## LED

| | |
|---|---|
| solid | parsing and streaming |
| slow blink (1 Hz) | IMU fine, nothing subscribed (only with autostream off) |
| fast blink (5 Hz) | no valid samples — wiring, baud, or an unconfigured IMU |

## What changed from the lab sketch

- **SBUS kept on `Serial7`, new transport.** The original sent each frame as an
  ASCII line to a second port (8001); here it is a CRC-sealed `DashRcPacket` on
  the same stream as the IMU. The lab's host parser ignored the failsafe flag;
  it is passed through. The original also sent nothing while the receiver was
  silent, which a host could not tell from sticks held still; now a `NoSignal`
  heartbeat goes out every 100 ms. (SBUS was removed from this sketch at first
  and restored once the X8R was wired.)
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
- **RX buffer enlarged** from the 64-byte default to 4096 (445 ms of stall). A
  frame plus sync is 46 bytes, so the default left 18 bytes of slack, and an
  overrun shows up as a CRC failure rather than as anything that names the real
  cause. 512 turned out to be too small; see [IMU before RC](#imu-before-rc).
- **Watchdog added**, matching the relay boards.
- The `delay(1)` in the original loop is gone; the frame-complete check does the
  same job without adding jitter.

Unchanged and deliberately so: the VN-100 CRC-16 routine and the payload offsets
(3 / 19 / 31), which are copied verbatim from a sketch known to work on the
previous robot.

## Host side

`DashImuBoard` (`DARoS-Core/Systems/DashSystem/RobotHardware/`) is the C++
reader and `dash_imu_listen.py` the quick one. Neither decodes `DashRcPacket`
yet; both skip it by magic. The DARoS-Core copy of `DashImuProtocol.h` is
**the source of truth**, with this one kept byte-identical the way
`DashUdpProtocol.h` already is:

```
cp ~/Repositories/DARoS-Core/Systems/DashSystem/RobotHardware/DashImuProtocol.h \
   ~/Repositories/DashTeensy/dash_imu_v1/
```

Bump `kDashImuProtoVersion` on any layout change; both sides reject a mismatched
version, so a stale binary fails loudly instead of misparsing.

## Dependencies

- [QNEthernet](https://github.com/ssilverman/QNEthernet)
- Watchdog_t4 (bundled with Teensyduino)
- [Bolder Flight Systems SBUS](https://github.com/bolderflight/sbus) 8.1.4 —
  Library Manager, "Bolder Flight Systems SBUS". Its channel decoding warns
  "suggest parentheses" on nearly every line; the values are correct regardless
  (checked against a bit-by-bit decoder over a million random frames).

Board: Teensy 4.1, 600 MHz.

# dash_v1 — Dash Teensy relay board, bring-up firmware

One sketch, flashed to **both** boards. Each Teensy 4.1 has all three CAN
channels available and carries up to `kDashMaxMotors` (12) motors.

Upper-torso wiring as of the ten-motor build:

| Board | `DASH_BOARD_ID` | IP | CAN1 | CAN2 | CAN3 |
|---|---|---|---|---|---|
| left  | 1 | 192.168.0.111 | lShoulderPitch, lHipYaw | lShoulderRoll, lShoulderYaw, lElbowPitch | — |
| right | 2 | 192.168.0.112 | rShoulderPitch, rHipYaw | rShoulderRoll, rShoulderYaw, rElbowPitch | — |

The board is a dumb relay: it copies raw 8-byte payloads between UDP and CAN.
It does not know about MIT scaling, motor limits or controller modes — those
live on the host in `DARoS-Core/Systems/DashSystem/RobotHardware/`, so you can
change them without reflashing.

**It does not know the wiring either.** Every command packet carries, per motor,
which channel to use and which id to address. The host's `kDashBoards` table in
`DashHardwareConfig.h` is the single source of truth, so moving a motor between
channels is a host rebuild — never a reflash.

## Which board am I? — `DASH_BOARD_ID`

The one thing a board knows about itself is its IP address, set by
`#define DASH_BOARD_ID` at the top of the sketch. **Set it before every flash.**
1 → `.111` (left), 2 → `.112` (right); anything else is a compile error.

Flashing both boards with the same id does not fail cleanly. Two devices end up
sharing an address, ARP resolves to whichever answered most recently, replies
interleave from both boards, and on the host you see motors flicking online and
offline with no obvious cause. The sketch prints its identity in the boot banner
and echoes the host's motor list on the first command received — check both
after a flash.

## Two boards, one host

Nothing on the board coordinates with the other one; they never talk to each
other and their CAN buses are entirely separate. The host runs one
`DashTeensyBoard` per Teensy, each with its own UDP socket on its own local port
(`host_port` in `kDashBoards`).

That works because this sketch replies to `udp.remoteIP()` **and**
`udp.remotePort()` — the source address and port of the command it just
received — rather than to a fixed host endpoint. Both boards listen on
`kDashUdpPort`; only the host's source ports differ. If you ever change the
reply path to use a hard-coded port, both boards' state packets will land in one
socket and the two streams will be indistinguishable.

## Cycle

One UDP command in → one CAN frame per motor, **queued across all channels
before waiting for any reply** → collect replies (1.5 ms timeout) → one UDP
state packet out. The loop rate is therefore exactly the host's command rate.

Because the channels are all loaded before the wait, they arbitrate in parallel
and a cycle costs the airtime of the *busiest* channel, not the sum. An extended
8-byte frame is ~131 µs at 1 Mbit/s and each motor needs two (command +
reply), so:

| Motors on busiest channel | CAN airtime per cycle | Practical max rate |
|---|---|---|
| 1 | ~262 µs | comfortably 1 kHz |
| 2 | ~524 µs | comfortably 1 kHz |
| 3 | ~786 µs | ~1 kHz, marginal |
| 4 | ~1048 µs | below 1 kHz |

Three on CAN1 plus three on CAN2 costs the same ~786 µs as three on one channel.
**This is the constraint that should drive motor-to-channel assignment: keep it
to ≤2 per channel if you want comfortable 1 kHz headroom.**

The boards are independent, so this is per board and does not add up across
them. Both current boards are 2 on CAN1 and 3 on CAN2, i.e. ~786 µs each, set
by the three-motor channel. CAN3 is free on both — moving one motor off CAN2
onto CAN3 would bring both boards to ~524 µs and buy real 1 kHz headroom.

If the host goes quiet for 200 ms the board stops transmitting on CAN entirely;
the motors' own `CFG_CAN_TIMEOUT` (250 ms) then faults them to Menu mode, so
there is no need for the Teensy to command a stop itself.

## Keeping the protocol in sync

`DashUdpProtocol.h` in this folder **must stay byte-identical** to
`DARoS-Core/Systems/DashSystem/RobotHardware/DashUdpProtocol.h`. After editing
the DARoS-Core copy:

```
cp ~/Repositories/DARoS-Core/Systems/DashSystem/RobotHardware/DashUdpProtocol.h \
   ~/Repositories/DashTeensy/dash_v1/
```

Bump `kDashProtoVersion` on any layout change — both sides reject a mismatched
version, so a stale binary fails loudly instead of misparsing. **Protocol v2
added the routing fields, so a v1 host and a v2 board will not talk to each
other**; flash this sketch and rebuild the host together.

## Dependencies

- [QNEthernet](https://github.com/ssilverman/QNEthernet)
- FlexCAN_T4 (bundled with Teensyduino)
- Watchdog_t4 (bundled with Teensyduino)

Board: Teensy 4.1. Set CPU speed to 600 MHz.

## Pinout

| Channel | `can_channel` on host | TX pin | RX pin |
|---|---|---|---|
| CAN1 | 0 | 22 | 23 |
| CAN2 | 1 | 1 | 0 |
| CAN3 | 2 | 31 | 30 |

All three are brought up in `setup()` regardless of what is cabled; a channel
with no motors assigned to it is simply never written to.

## Gotchas

**Extended CAN ids.** The Dash motor controller uses 29-bit ids
(`IDE = CAN_ID_EXT`). The receive mailboxes are explicitly configured with
`setMB(mb, RX, EXT)` — a mailbox left in the default standard-id mode silently
never matches, which looks exactly like a dead bus.

**Reply demultiplexing is keyed on (channel, id).** Every motor replies with
`ExtId = CFG_CAN_MASTER` (0) — the *same id for all of them* — so replies are
matched on payload byte 0, which the firmware sets to the sender's
`CFG_CAN_ID`. Ids only need to be unique *within* a channel, so id 1 exists on
both CAN1 and CAN2 on the current bench. That is why the channel a frame arrived
on is part of the demux key, and why there is one `onReceive` callback per
channel rather than one shared handler.

Consequence worth knowing: the host can detect a wrong *id* (reply byte 0
disagrees with the configured id) but **not** a wrong *channel* when the same id
exists on two channels — a motor cabled to the wrong channel shows up as
OFFLINE instead.

**`events()` is deliberately never called.** FlexCAN_T4 dispatches `onReceive`
callbacks straight from the ISR as long as `events()` has never been called
(`isEventsUsed` stays 0 in `struct2queueRx`). Once you call it, callbacks are
instead queued and dispatched **one frame per `events()` call** from the main
loop, which adds latency and needs an explicit drain loop. Do not add an
`events()` call to this sketch without re-testing round-trip timing.

**Callbacks run in interrupt context**, so `g_reply`, `g_reply_valid`, `g_bus`,
`g_can_id` and `g_n_motors` are all `volatile`. The routing arrays are only
written in `receiveUdp()`, which runs before `runCanCycle()` while CAN is idle
(the motors only ever answer a frame we sent).

**`SETUP_ONE_CAN` is a macro on purpose.** The three `FlexCAN_T4<CANn, ...>`
objects are unrelated types and `setMB()` is not virtual on the shared base, so
a common helper would have to be a template — and a file-scope template does not
survive the Arduino build. The builder auto-generates a forward prototype for
every function definition in a `.ino` and emits it *without* the
`template<...>` header, giving `error: variable or field 'setupOneCan' declared
void`. Note this means a plain `g++ -x c++` check on the sketch will pass code
the real toolchain rejects; only the Arduino/Teensy build exercises the
prototype injection.

**Mode frames produce no reply.** The firmware's mode / param branches return
without calling `pack_reply()`, so during a mode change `online_mask` is 0 for a
few cycles. That is expected, not a fault.

**Param replies are ignored.** A param read answers with
`[id][idx][float BE]`, whose byte 0 is a valid motor id — it would be
misread as a state frame. This sketch never sends param reads; if you add them,
tag the reply so `canRxCommon()` can tell them apart.

## Changing the wiring

Edit `kDashBoards` in the host's
`DARoS-Core/Systems/DashSystem/RobotHardware/DashHardwareConfig.h` and rebuild
the host. Nothing in this sketch needs to change and no reflash is needed —
`can_channel` and `can_id` travel with every command packet. The only reason to
reflash is a change of `DASH_BOARD_ID`.

## Checking it from the host

```
./build/bin/dash_monitor          # both boards, read-only, never energises
./build/bin/dash_monitor right    # one board
```

`dash_monitor` holds every motor in Menu mode and streams zero commands, which
is enough to get position, velocity, torque, bus voltage and temperature back
without the phases ever being driven. Use it to confirm each joint's name sits
on the right motor — grab a joint, watch which row moves — before running
`dash_teensy_test`, which does energise.

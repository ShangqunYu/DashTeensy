/*
 * Dash IMU Teensy 4.1 -- VN-100 serial to UDP bridge.
 *
 * Third board on the torso, alongside the two dash_v1 motor relays.  It reads
 * the VN-100's binary output on Serial2 and the RC receiver's SBUS on Serial7,
 * and forwards both to the host over ethernet.  No CAN, no motors, no control.
 *
 * Adapted from the lab's vn100_imu_and_rc_teensy sketch (Flight Dynamics and
 * Control Lab, MIT licence -- see the notice at the bottom).  What changed:
 *
 *   - SBUS still arrives on Serial7, but leaves as a CRC-sealed DashRcPacket on
 *     the IMU's own stream rather than an ASCII line to a second port.  The
 *     failsafe flag, which the lab's host parser ignored, is passed through,
 *     and a heartbeat marks a receiver that has gone silent -- the original
 *     sent nothing then, which looked exactly like sticks held still.
 *   - Static IP moved from the old 10.0.0.x lab net onto Dash's 192.168.0.x,
 *     next to the two relay boards.
 *   - ASCII "%f,%f,..." replaced by a packed binary struct with a magic
 *     number, a version, a sequence counter and a CRC-8, matching the style of
 *     DashUdpProtocol.h.  The old format could not tell a truncated datagram
 *     from a short one, cost ~130 bytes per sample instead of 57, and made the
 *     host reconstruct floats it had just been sent.
 *   - The destination is no longer hard-coded: the host subscribes with a
 *     DashImuHello and we stream to its source address and port.
 *   - The VN-100 group/field header is now checked, so an IMU that was never
 *     configured (or was configured with different fields) is reported instead
 *     of being parsed as garbage that happens to pass a CRC over the wrong
 *     length.
 *   - Framing waits for a whole frame to be buffered before consuming the sync
 *     byte, so readBytes() can never block mid-frame.  At 115200 baud a
 *     45-byte frame is 3.9 ms on the wire -- long enough for the original's
 *     blocking read to stall the loop for most of a sample period.
 *   - Watchdog added, to match the relay boards.
 *
 * The IMU must be configured before this is useful -- see README.md.
 */

#include <QNEthernet.h>

#include "DashImuProtocol.h"
#include "Watchdog_t4.h"
#include "sbus.h"

// ===========================================================================
//   Configuration
// ===========================================================================

// This board's address.  .111 and .112 are the left and right motor relays,
// .110 is the host; the IMU board is .113.  Unlike dash_v1 there is only ever
// one of these, so there is no DASH_BOARD_ID to get wrong at flash time.
static constexpr uint8_t kTeensyIp[4] = {192, 168, 0, 113};
static constexpr uint8_t kTeensyGateway[4] = {192, 168, 0, 1};
static constexpr uint8_t kTeensySubnet[4] = {255, 255, 255, 0};

// Where to stream when nobody has said hello.  Set DASH_IMU_AUTOSTREAM to 0 to
// send nothing at all until a host subscribes; leave it at 1 and you can flash
// the board and watch packets arrive with a three-line python script.
#define DASH_IMU_AUTOSTREAM 1
static constexpr bool kAutostream = (DASH_IMU_AUTOSTREAM != 0);
static constexpr uint8_t kDefaultHostIp[4] = {192, 168, 0, 110};
static constexpr uint16_t kDefaultHostPort = kDashImuUdpPort;

// A subscription lapses this long after the last hello.  Long enough that a
// host resending at 5 Hz keeps it alive through a few dropped datagrams.
static constexpr uint32_t kSubscriberTimeoutMs = 2000;

// Serial2 = pin 7 (RX2) / 8 (TX2) on a Teensy 4.1.  Only RX is needed to
// stream; TX is wired so the IMU can be configured over the same cable using
// the passthrough below.
//
// 115200 is the VN-100 factory default and is what the lab sketch used.  A
// 46-byte frame costs 4.0 ms of the 5.0 ms sample period at 200 Hz; that has
// measured error-free, but leaves little room to raise the rate.  To go faster,
// set the IMU to 460800 (README has the command) and change this to match.
static constexpr uint32_t kImuBaud = 115200;

// Enlarges Serial2's 64-byte default RX buffer.  The IMU delivers 9200 bytes/s
// (46 x 200 Hz) whatever the baud rate, so this is how long loop() may stall
// before bytes are lost: 4096 bytes = 445 ms.  The UART interrupt drops bytes
// silently when the buffer is full, and an overrun only ever shows up as
// bad_crc + resyncs.  512 bytes (55 ms) was measurably not enough: a USB serial
// print blocks for 120 ms when the host stops reading, and every such stall
// cost exactly one frame.
static constexpr size_t kImuRxBufferBytes = 4096;

// Bridges USB serial <-> Serial2 so the IMU can be configured from the Arduino
// serial monitor without unplugging anything.  Leave it off for normal running:
// while it is on, no samples are parsed or forwarded.
#define DASH_IMU_CONFIG_PASSTHROUGH 0

#define DEBUG_MODE

#ifdef DEBUG_MODE
#include <stdarg.h>
// Never blocks.  Serial.printf() waits up to 120 ms whenever the USB buffers
// are full -- which they are a few seconds after a serial monitor closes, since
// nothing drains them -- and that stall costs IMU bytes.  So a line goes out
// only if a terminal has the port open and the whole line fits in buffers that
// are already free; otherwise it is dropped.  A lost console line is fine; a
// lost sample is not.
static void debug_print(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));
static void debug_print(const char *fmt, ...) {
  if (!Serial) return;
  char line[256];
  va_list args;
  va_start(args, fmt);
  const int n = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  if (n <= 0) return;
  const size_t len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
  if (Serial.availableForWrite() < (int)len) return;
  Serial.write((const uint8_t *)line, len);
}
#else
#define debug_print(...) \
  do {                   \
  } while (0)
#endif

// ---- VN-100 binary frame layout ------------------------------------------
//
// What register 75 is configured to emit (see README.md):
//
//   0xFA | groups=0x01 | fields=0x0130 | quat[4] | gyro[3] | accel[3] | crc16
//    1        1              2             16        12        12        2
//
// fields 0x0130 = Quaternion (0x0010) + AngularRate (0x0020) + Accel (0x0100),
// all from the Common group.  The order of the payload is the order of the
// field bits, low to high -- that is what fixes quat before gyro before accel.
//
// Change the register and you must change all four of these together; the
// header check below is what stops a mismatch from being parsed as noise.
static constexpr uint8_t kVnSync = 0xFA;
static constexpr uint8_t kVnGroups = 0x01;
static constexpr uint16_t kVnFields = 0x0130;
static constexpr int kVnPayloadBytes = 40;
// Everything after the sync byte: groups + fields + payload + crc16.
static constexpr int kVnFrameBytes = 1 + 2 + kVnPayloadBytes + 2;

// If no valid sample arrives for this long the IMU is considered missing, which
// changes the LED and prints a warning.
static constexpr uint32_t kImuTimeoutMs = 200;

// ---- RC receiver -----------------------------------------------------------
//
// FrSky X8R SBUS port -> pin 28 (RX7); TX7 is not used.  SBUS is inverted
// serial at 100000 baud 8E2, and bfs::SbusRx inverts inside the Teensy's UART,
// so no external inverter.  Teensy 4.1 pins are NOT 5 V tolerant.
//
// Enlarges Serial7's 64-byte RX buffer.  An SBUS frame is 25 bytes, so the
// default holds two -- one slow pass through loop() (an ethernet send that
// takes its time) and the parser starts on a frame whose head was overwritten.
static constexpr size_t kRcRxBufferBytes = 256;

// While no SBUS frame arrives, a NoSignal heartbeat goes out this often.
static constexpr uint32_t kRcHeartbeatMs = 100;

// ===========================================================================

using namespace qindesign::network;

WDT_T4<WDT1> wdt;
EthernetUDP udp;

static uint8_t g_rx_buffer[kImuRxBufferBytes];
static uint8_t g_frame[kVnFrameBytes];

static IPAddress g_host_ip(kDefaultHostIp[0], kDefaultHostIp[1],
                           kDefaultHostIp[2], kDefaultHostIp[3]);
static uint16_t g_host_port = kDefaultHostPort;
static uint32_t g_last_hello_ms = 0;
static bool g_have_subscriber = false;
static bool g_link_up = false;

static uint32_t g_seq = 0;
static uint32_t g_last_sample_ms = 0;
static uint16_t g_bad_crc = 0;
static uint16_t g_resyncs = 0;
static bool g_bad_format = false;
static uint32_t g_send_fail = 0;

bfs::SbusRx g_sbus(&Serial7);
static uint8_t g_rc_rx_buffer[kRcRxBufferBytes];
static uint16_t g_rc_ch[kDashRcNumChannels];  /* last frame; zeros until one */
static uint32_t g_rc_seq = 0;
static uint32_t g_rc_frames = 0;
static uint32_t g_last_rc_ms = 0;
static bool g_have_rc = false;
static uint16_t g_rc_lost_frames = 0;
static uint32_t g_last_rc_heartbeat_ms = 0;
static bool g_rc_failsafe = false;

// Saturating, because these are diagnostics: a counter that wrapped to 3 looks
// like a healthy link, and 65535 does not.
static inline void bumpSaturating(uint16_t *counter) {
  if (*counter != 0xFFFF) ++(*counter);
}

// VN-100's 16-bit CRC (CCITT), over everything between the sync byte and the
// checksum.  Unchanged from the lab sketch.
static uint16_t vnCrc16(const uint8_t data[], unsigned int length) {
  uint16_t crc = 0;
  for (unsigned int i = 0; i < length; i++) {
    crc = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

void setupEthernet() {
  uint8_t mac[6];
  Ethernet.macAddress(mac);
  debug_print("MAC = %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2],
              mac[3], mac[4], mac[5]);

  IPAddress ip(kTeensyIp[0], kTeensyIp[1], kTeensyIp[2], kTeensyIp[3]);
  IPAddress gateway(kTeensyGateway[0], kTeensyGateway[1], kTeensyGateway[2],
                    kTeensyGateway[3]);
  IPAddress subnet(kTeensySubnet[0], kTeensySubnet[1], kTeensySubnet[2],
                   kTeensySubnet[3]);

  Ethernet.onLinkState([](bool state) {
    g_link_up = state;
    debug_print("[Ethernet] Link %s\n", state ? "ON" : "OFF");
  });

  if (!Ethernet.begin(mac, ip, gateway, gateway, subnet)) {
    debug_print("[Ethernet] failed to start\n");
  } else {
    debug_print("[Ethernet] up at %u.%u.%u.%u\n", kTeensyIp[0], kTeensyIp[1],
                kTeensyIp[2], kTeensyIp[3]);
  }
}

void wdCallback() { debug_print("FEED THE DOG SOON, OR RESET!\n"); }

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
  }
#endif
  debug_print("\n=====================================================\n");
  debug_print("[dash_imu_v1] VN-100 bridge at 192.168.0.%u\n", kTeensyIp[3]);
  debug_print("[dash_imu_v1] protocol v%u, packet %u bytes\n",
              kDashImuProtoVersion, (unsigned)sizeof(DashImuPacket));
  debug_print("[dash_imu_v1] IMU on Serial2 at %lu baud\n",
              (unsigned long)kImuBaud);
  debug_print("[dash_imu_v1] RC SBUS on Serial7 (pin 28), packet %u bytes\n",
              (unsigned)sizeof(DashRcPacket));
  debug_print("=====================================================\n");

  pinMode(LED_BUILTIN, OUTPUT);

  Serial2.begin(kImuBaud);
  Serial2.addMemoryForRead(g_rx_buffer, sizeof(g_rx_buffer));
  // Backstop only.  readOneSample() never calls readBytes() until a whole frame
  // is already buffered, so this should never fire -- but the default is 1000 ms
  // and a single stall that long would take the watchdog with it.
  Serial2.setTimeout(5);

  g_sbus.Begin();
  Serial7.addMemoryForRead(g_rc_rx_buffer, sizeof(g_rc_rx_buffer));

  // The IMU outranks the RC receiver.  Both UARTs default to priority 64, so an
  // RC byte's interrupt could hold off an IMU byte while it sits in the UART's
  // small hardware FIFO.  0 is highest; the Cortex-M7 honours steps of 16.
  // After begin(), which is what sets the default.
  NVIC_SET_PRIORITY(IRQ_LPUART4, 32);  // Serial2, IMU
  NVIC_SET_PRIORITY(IRQ_LPUART7, 96);  // Serial7, RC

#if !DASH_IMU_CONFIG_PASSTHROUGH
  WDT_timings_t config;
  config.trigger = 2; /* seconds */
  config.timeout = 5; /* seconds */
  config.callback = wdCallback;
  wdt.begin(config);

  setupEthernet();
  if (udp.begin(kDashImuUdpPort)) {
    debug_print("[UDP] listening on %u\n", kDashImuUdpPort);
  } else {
    debug_print("[UDP] failed to start\n");
  }
#if DASH_IMU_AUTOSTREAM
  debug_print("[UDP] autostreaming to %u.%u.%u.%u:%u until a host says hello\n",
              kDefaultHostIp[0], kDefaultHostIp[1], kDefaultHostIp[2],
              kDefaultHostIp[3], kDefaultHostPort);
#endif
#else
  debug_print("[dash_imu_v1] CONFIG PASSTHROUGH -- USB <-> Serial2, no bridging\n");
#endif
}

// Accepts hellos and retargets the stream.  Same rule as dash_v1: the reply
// goes to the source address AND source port of what we received, so two
// processes on the host can subscribe independently.
void receiveHello() {
  while (true) {
    const int size = udp.parsePacket();
    if (size <= 0) break;
    if (size != (int)sizeof(DashImuHello)) continue;
    DashImuHello hello;
    memcpy(&hello, udp.data(), sizeof(hello));
    if (!dash_imu_check_hello(&hello, size)) continue;

    const IPAddress from = udp.remoteIP();
    const uint16_t port = udp.remotePort();
    if (!g_have_subscriber || !(from == g_host_ip) || port != g_host_port) {
      debug_print("[UDP] subscriber %u.%u.%u.%u:%u\n", from[0], from[1], from[2],
                  from[3], port);
    }
    g_host_ip = from;
    g_host_port = port;
    g_have_subscriber = true;
    g_last_hello_ms = millis();
  }

  if (g_have_subscriber && (millis() - g_last_hello_ms) > kSubscriberTimeoutMs) {
    debug_print("[UDP] subscriber timed out\n");
    g_have_subscriber = false;
    g_host_ip = IPAddress(kDefaultHostIp[0], kDefaultHostIp[1],
                          kDefaultHostIp[2], kDefaultHostIp[3]);
    g_host_port = kDefaultHostPort;
  }
}

static uint8_t currentStatus() {
  return (uint8_t)((g_link_up ? kDashImuStatusLink : 0) |
                   (g_have_subscriber ? kDashImuStatusSubscribed : 0) |
                   (g_bad_format ? kDashImuStatusBadFormat : 0));
}

void sendDatagram(const uint8_t *data, size_t size) {
  if (!udp.send(g_host_ip, g_host_port, data, size)) {
    // Rate-limited on purpose.  This fires once per packet, and autostreaming
    // at 200 Hz to a host that has nothing bound to the port makes every one of
    // them fail -- which at one printf each is 200 lines a second of USB serial
    // for a condition that is not even an error.
    ++g_send_fail;
    static uint32_t last_fail_ms = 0;
    if ((millis() - last_fail_ms) > 1000) {
      last_fail_ms = millis();
      debug_print("[UDP] %lu send(s) failed -- is anything listening on "
                  "%u.%u.%u.%u:%u?\n",
                  (unsigned long)g_send_fail, g_host_ip[0], g_host_ip[1],
                  g_host_ip[2], g_host_ip[3], g_host_port);
      g_send_fail = 0;
    }
  }
}

void sendSample(const float *quat, const float *gyro, const float *accel) {
  if (!g_have_subscriber && !kAutostream) return;

  DashImuPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = kDashImuMagicData;
  pkt.version = kDashImuProtoVersion;
  pkt.status = currentStatus();
  pkt.seq = g_seq;
  pkt.teensy_us = micros();
  pkt.bad_crc = g_bad_crc;
  pkt.resyncs = g_resyncs;
  memcpy(pkt.quat, quat, sizeof(pkt.quat));
  memcpy(pkt.gyro, gyro, sizeof(pkt.gyro));
  memcpy(pkt.accel, accel, sizeof(pkt.accel));
  dash_imu_seal(&pkt);
  sendDatagram((const uint8_t *)&pkt, sizeof(pkt));
}

void sendRc(uint8_t flags, uint16_t age_ms) {
  if (!g_have_subscriber && !kAutostream) return;

  DashRcPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = kDashRcMagicData;
  pkt.version = kDashImuProtoVersion;
  pkt.status = currentStatus();
  pkt.seq = ++g_rc_seq;
  pkt.teensy_us = micros();
  pkt.flags = flags;
  pkt.lost_frames = g_rc_lost_frames;
  pkt.age_ms = age_ms;
  memcpy(pkt.ch, g_rc_ch, sizeof(pkt.ch));
  dash_rc_seal(&pkt);
  sendDatagram((const uint8_t *)&pkt, sizeof(pkt));
}

// Forwards the newest SBUS frame if one has arrived, or a NoSignal heartbeat
// once none has for kDashRcNoSignalMs.
//
// SbusRx::Read() drains every buffered byte and keeps only the last complete
// frame, so this never blocks, and a backlog after a stall costs stale frames
// rather than time.  It also means lost_frames counts the frames we saw, not
// every frame the receiver sent.
void pollRc() {
  const uint32_t now = millis();
  if (g_sbus.Read()) {
    const bfs::SbusData d = g_sbus.data();
    for (int i = 0; i < kDashRcNumChannels; ++i) g_rc_ch[i] = (uint16_t)d.ch[i];
    uint8_t flags = 0;
    if (d.lost_frame) {
      flags |= kDashRcFlagLostFrame;
      bumpSaturating(&g_rc_lost_frames);
    }
    if (d.failsafe) flags |= kDashRcFlagFailsafe;
    if (d.ch17) flags |= kDashRcFlagCh17;
    if (d.ch18) flags |= kDashRcFlagCh18;
    g_rc_failsafe = d.failsafe;
    g_have_rc = true;
    g_last_rc_ms = now;
    ++g_rc_frames;
    sendRc(flags, 0);
    return;
  }

  if (g_have_rc && (now - g_last_rc_ms) < kDashRcNoSignalMs) return;
  if ((now - g_last_rc_heartbeat_ms) < kRcHeartbeatMs) return;
  g_last_rc_heartbeat_ms = now;
  const uint32_t age = g_have_rc ? (now - g_last_rc_ms) : 0xFFFF;
  sendRc(kDashRcFlagNoSignal, (uint16_t)(age < 0xFFFF ? age : 0xFFFF));
}

// Consumes one buffered VN-100 frame if a whole one is present.  Returns true
// if a sample was parsed and forwarded.
//
// The wait for a COMPLETE frame before touching the sync byte is what keeps
// this non-blocking: once available() clears the bar, readBytes() is a memcpy
// out of the ring buffer and cannot stall waiting on the wire.
// False until the first frame passes its CRC.  The board almost always starts
// listening partway through a frame, and hunting past that fragment -- or
// tripping over an 0xFA inside it -- is not a data error, so it is not counted.
static bool g_imu_locked = false;

bool readOneSample() {
  if (Serial2.available() < kVnFrameBytes + 1) return false;

  // Hunt for the sync byte.  On a healthy stream the very next byte is it, so
  // a nonzero resync count means bytes are being lost or the IMU is emitting
  // something other than the configured message.
  bool discarded = false;
  while (Serial2.available() > 0 && Serial2.peek() != kVnSync) {
    Serial2.read();
    discarded = true;
  }
  if (discarded && g_imu_locked) bumpSaturating(&g_resyncs);
  if (Serial2.available() < kVnFrameBytes + 1) return false;
  Serial2.read();  // the sync byte itself

  Serial2.readBytes(g_frame, kVnFrameBytes);

  const uint16_t checksum =
      (uint16_t)((g_frame[kVnFrameBytes - 2] << 8) | g_frame[kVnFrameBytes - 1]);
  if (vnCrc16(g_frame, kVnFrameBytes - 2) != checksum) {
    if (g_imu_locked) bumpSaturating(&g_bad_crc);
    return false;
  }
  g_imu_locked = true;

  // CRC passed, so these bytes really are what the IMU meant to send.  If the
  // header is not what we compiled for, the payload after it is a different
  // length and a different set of fields -- refuse it rather than reinterpret
  // it.  This is the check the original sketch did not have.
  const uint16_t fields = (uint16_t)(g_frame[1] | (g_frame[2] << 8));
  if (g_frame[0] != kVnGroups || fields != kVnFields) {
    if (!g_bad_format) {
      debug_print("[IMU] unexpected output format: groups 0x%02X fields 0x%04X "
                  "(expected 0x%02X / 0x%04X) -- reconfigure register 75, see "
                  "README\n",
                  g_frame[0], fields, kVnGroups, kVnFields);
    }
    g_bad_format = true;
    return false;
  }

  // VN-100 sends little-endian IEEE-754 floats and the Teensy is little-endian,
  // so the payload copies straight across.
  float quat[4], gyro[3], accel[3];
  memcpy(quat, &g_frame[3], sizeof(quat));
  memcpy(gyro, &g_frame[3 + 16], sizeof(gyro));
  memcpy(accel, &g_frame[3 + 16 + 12], sizeof(accel));

  ++g_seq;
  g_last_sample_ms = millis();
  sendSample(quat, gyro, accel);
  return true;
}

// Solid = streaming, slow blink = IMU fine but nobody subscribed, fast blink =
// no IMU data.  Readable from across the lab, which a serial console is not.
void updateLed(bool imu_alive) {
  const uint32_t now = millis();
  bool on;
  if (!imu_alive) {
    on = (now % 200) < 100;   /* fast blink: nothing coming in on Serial2 */
  } else if (!g_have_subscriber && !kAutostream) {
    on = (now % 1000) < 500;  /* slow blink: parsing, but sending nowhere */
  } else {
    on = true;                /* solid: streaming */
  }
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void loop() {
#if DASH_IMU_CONFIG_PASSTHROUGH
  // Straight USB <-> Serial2 bridge for talking to the IMU by hand.
  while (Serial.available() > 0) Serial2.write(Serial.read());
  while (Serial2.available() > 0) Serial.write(Serial2.read());
  return;
#else
  wdt.feed();
  receiveHello();

  // Drain whatever the IMU has buffered.  Normally one frame per pass; the cap
  // stops a burst after a stall from starving the hello handler and watchdog.
  for (int i = 0; i < 8; ++i) {
    if (!readOneSample()) break;
  }

  // RC only once no complete IMU frame is waiting.  An SBUS frame left in
  // Serial7's buffer costs nothing -- SbusRx keeps the newest -- while an IMU
  // frame left waiting is a late sample.
  if (Serial2.available() < kVnFrameBytes + 1) pollRc();

  const bool imu_alive =
      g_last_sample_ms != 0 && (millis() - g_last_sample_ms) < kImuTimeoutMs;
  updateLed(imu_alive);

#ifdef DEBUG_MODE
  static uint32_t last_report_ms = 0;
  if ((millis() - last_report_ms) > 1000) {
    last_report_ms = millis();
    if (!imu_alive) {
      debug_print("[IMU] no valid samples -- check Serial2 wiring, baud (%lu) "
                  "and that the IMU is configured\n",
                  (unsigned long)kImuBaud);
    } else {
      static uint32_t last_seq = 0;
      debug_print("[IMU] %lu Hz  bad_crc %u  resyncs %u  -> %u.%u.%u.%u:%u%s\n",
                  (unsigned long)(g_seq - last_seq), g_bad_crc, g_resyncs,
                  g_host_ip[0], g_host_ip[1], g_host_ip[2], g_host_ip[3],
                  g_host_port, g_have_subscriber ? "" : " (default)");
      last_seq = g_seq;
    }

    static uint32_t last_rc_frames = 0;
    if (!g_have_rc || (millis() - g_last_rc_ms) >= kDashRcNoSignalMs) {
      debug_print("[RC] no SBUS frames -- check the receiver on pin 28 (RX7), "
                  "its power, and that it is bound\n");
    } else {
      // Grouped as the lab's parser read them: sticks | switches | knobs.
      debug_print("[RC] %lu Hz  lost_frames %u%s  ch %4u %4u %4u %4u | %4u %4u "
                  "%4u %4u %4u %4u | %4u %4u\n",
                  (unsigned long)(g_rc_frames - last_rc_frames),
                  g_rc_lost_frames, g_rc_failsafe ? "  FAILSAFE" : "",
                  g_rc_ch[0], g_rc_ch[1], g_rc_ch[2], g_rc_ch[3], g_rc_ch[4],
                  g_rc_ch[5], g_rc_ch[6], g_rc_ch[7], g_rc_ch[8], g_rc_ch[9],
                  g_rc_ch[10], g_rc_ch[11]);
    }
    last_rc_frames = g_rc_frames;
  }
#endif
#endif
}

// ---------------------------------------------------------------------------
// Derived from vn100_imu_and_rc_teensy.ino:
//
// Copyright (c) 2018 Flight Dynamics and Control Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ---------------------------------------------------------------------------

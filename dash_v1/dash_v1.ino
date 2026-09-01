/*
 * Dash Teensy 4.1 relay board -- multi-channel bring-up.
 *
 * One sketch, flashed to every board.  Set DASH_BOARD_ID below to say which
 * board this copy is; that picks the IP address and nothing else.
 *
 * The board is deliberately dumb: it copies raw 8-byte payloads between UDP and
 * CAN and knows nothing about MIT scaling, motor limits or controller modes.
 * All of that lives on the host in DashMotorProtocol.h, where it can be changed
 * without reflashing.
 *
 * It does not even know the wiring.  Every command packet carries, per motor,
 * which CAN channel to use and which id to address, so the host's kDashMotors
 * table in DashHardwareConfig.h is the single source of truth.  Moving a motor
 * to a different channel is a host rebuild, not a reflash.
 *
 * Cycle: one UDP command in -> one CAN command per motor -> collect replies ->
 * one UDP state packet out.  The loop rate is therefore exactly the host's
 * command rate.
 *
 * All three channels are transmitted before we wait for any reply, so the
 * channels arbitrate in parallel and the cycle costs the airtime of the
 * *busiest* channel rather than the sum of all of them.  Three motors on one
 * channel is ~780 us; three on CAN1 plus three on CAN2 is also ~780 us.
 *
 * NOTE on CAN ids: the Dash motor controller uses 29-bit *extended* ids.  We
 * transmit to the motor's own CFG_CAN_ID, but every motor replies with
 * ExtId = CFG_CAN_MASTER (0), the same for all of them, so replies are
 * demultiplexed on payload byte 0 (which the firmware sets to the sender's id)
 * rather than on the CAN id.  Ids are only unique within a channel, so the
 * demux key is the pair (channel, byte 0).
 *
 * DashUdpProtocol.h in this folder must stay byte-identical to the copy in
 * DARoS-Core/Systems/DashSystem/RobotHardware/.
 */

#include <FlexCAN_T4.h>
#include <QNEthernet.h>

#include "DashUdpProtocol.h"
#include "Watchdog_t4.h"

// ===========================================================================
//   Configuration -- edit here when the wiring changes
// ===========================================================================

// ***** SET THIS BEFORE EVERY FLASH *****
//
// The only thing a board knows about itself.  Everything else -- which motors,
// which channels, which ids -- arrives in each command packet, so this is the
// one value that differs between the two boards and the one value that has to
// be right at flash time.
//
//   1 -> 192.168.0.111   left side
//   2 -> 192.168.0.112   right side
//
// Flashing both boards with the same id gives two devices one address.  That
// does not fail cleanly: ARP resolves to whichever answered last, replies
// interleave from both boards, and the host sees motors flicking online and
// offline for no visible reason.  The IP is printed in the boot banner and
// again on the first command received -- check it after every flash.
#define DASH_BOARD_ID 2

#if DASH_BOARD_ID == 1
static constexpr uint8_t kBoardIpLastOctet = 111;
static constexpr const char *kBoardName = "left";
#elif DASH_BOARD_ID == 2
static constexpr uint8_t kBoardIpLastOctet = 112;
static constexpr const char *kBoardName = "right";
#else
#error "DASH_BOARD_ID must be 1 (left, .111) or 2 (right, .112)"
#endif

// All three channels are brought up unconditionally; which ones actually carry
// traffic is decided by the host's table.  A channel with no transceiver simply
// never gets written to.  Both boards currently use CAN1 and CAN2 only.
//   CAN1 = pin 22 (TX) / 23 (RX)
//   CAN2 = pin  1 (TX) /  0 (RX)
//   CAN3 = pin 31 (TX) / 30 (RX)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

static constexpr uint8_t kTeensyIp[4] = {192, 168, 0, kBoardIpLastOctet};
static constexpr uint8_t kTeensyGateway[4] = {192, 168, 0, 1};
static constexpr uint8_t kTeensySubnet[4] = {255, 255, 255, 0};
// Fallback only: we normally reply to whoever sent the command -- including its
// source *port*, which is what lets the host run one socket per board.
static constexpr uint8_t kHostIp[4] = {192, 168, 0, 110};

static constexpr uint32_t kCanBaud = 1000000;

// How long to wait for every motor to answer before giving up on the cycle and
// reporting the missing ones offline.  Must comfortably exceed the airtime of
// the busiest channel: an extended 8-byte frame is ~131 us at 1 Mbps, and a
// channel with N motors needs 2*N frames, so three motors is ~790 us.  1500 us
// leaves margin for bit stuffing and a retransmission without ever reporting a
// healthy motor as offline.
static constexpr uint32_t kCanResponseTimeoutUs = 1500;

// Optional pacing between CAN transmits.  0 is correct: the frames are queued
// into separate mailboxes and the controller arbitrates them itself, so any
// delay here is pure added latency.
static constexpr uint32_t kCanSendDelayUs = 0;

// If the host goes quiet for this long we stop transmitting on CAN entirely.
// We deliberately do NOT command Menu mode ourselves: the motor's own
// CFG_CAN_TIMEOUT (10000 FOC cycles at 40 kHz = 250 ms) already faults it to
// Menu, and staying silent avoids fighting the host when it reconnects.
static constexpr uint32_t kHostTimeoutMs = 200;

#define DEBUG_MODE

#ifdef DEBUG_MODE
#define debug_print(...) Serial.printf(__VA_ARGS__)
#else
#define debug_print(...) \
  do {                   \
  } while (0)
#endif

// ===========================================================================

using namespace qindesign::network;

WDT_T4<WDT1> wdt;
EthernetUDP udp;

// Routing for the current cycle, copied out of the newest command packet.  Read
// by the CAN receive ISRs, so it must not change while a cycle is in flight --
// it is only written in receiveUdp(), which runs before runCanCycle().
static volatile uint8_t g_n_motors = 0;
static volatile uint8_t g_bus[kDashMaxMotors];
static volatile uint8_t g_can_id[kDashMaxMotors];

// Written by the CAN receive ISRs, read by the main loop.
static volatile bool g_reply_valid[kDashMaxMotors];
static volatile uint8_t g_reply[kDashMaxMotors][8];

static DashCmdPacket g_cmd;
static bool g_have_cmd = false;
static uint32_t g_last_cmd_ms = 0;

static IPAddress g_host_ip(kHostIp[0], kHostIp[1], kHostIp[2], kHostIp[3]);
static uint16_t g_host_port = kDashUdpPort;

// Payload byte 0 is the replying motor's CFG_CAN_ID.  Ids repeat across
// channels, so the channel the frame arrived on is part of the key.
static inline int slotFor(uint8_t bus, uint8_t can_id) {
  for (uint8_t i = 0; i < g_n_motors; ++i) {
    if (g_bus[i] == bus && g_can_id[i] == can_id) return i;
  }
  return -1;
}

// Runs in CAN interrupt context.  FlexCAN_T4 dispatches onReceive callbacks
// straight from the ISR as long as events() is never called, which is why this
// sketch does not call it: a reply is recorded the moment its mailbox
// interrupt fires, with no polling granularity in the way.
static inline void canRxCommon(uint8_t bus, const CAN_message_t &msg) {
  if (msg.len < 8) return;
  const int slot = slotFor(bus, msg.buf[0]);
  if (slot < 0) return;  // not one of ours, or a param reply -- ignore
  for (int b = 0; b < 8; ++b) g_reply[slot][b] = msg.buf[b];
  g_reply_valid[slot] = true;
}

// One trivial callback per channel.  CAN_message_t does carry a `bus` field,
// but binding the channel at registration time removes any doubt about its
// numbering.
void canRx1(const CAN_message_t &msg) { canRxCommon(0, msg); }
void canRx2(const CAN_message_t &msg) { canRxCommon(1, msg); }
void canRx3(const CAN_message_t &msg) { canRxCommon(2, msg); }

// Dispatch helpers.  The three FlexCAN_T4 instances are distinct types, so a
// switch is both simpler and cheaper than routing through the virtual base.
static inline void canWrite(uint8_t bus, const CAN_message_t &msg) {
  switch (bus) {
    case 0: Can1.write(msg); break;
    case 1: Can2.write(msg); break;
    case 2: Can3.write(msg); break;
    default: break;
  }
}

void wdCallback() { debug_print("FEED THE DOG SOON, OR RESET!\n"); }

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

  Ethernet.onLinkState(
      [](bool state) { debug_print("[Ethernet] Link %s\n", state ? "ON" : "OFF"); });

  if (!Ethernet.begin(mac, ip, gateway, gateway, subnet)) {
    debug_print("[Ethernet] failed to start\n");
  } else {
    debug_print("[Ethernet] up at %u.%u.%u.%u\n", kTeensyIp[0], kTeensyIp[1],
                kTeensyIp[2], kTeensyIp[3]);
  }
}

// The mailbox layout is identical on every channel: the motors reply with
// extended ids, so the receive mailboxes must be configured for EXT -- a
// mailbox left in the default standard-id mode silently never matches.
//
// This has to be a macro, not a function.  FlexCAN_T4<CAN1,...> and
// FlexCAN_T4<CAN2,...> are unrelated types, and the shared FlexCAN_T4_Base is
// no help because setMB() is not virtual -- so a shared helper would have to be
// a template.  A file-scope template does not survive the Arduino build: the
// preprocessor auto-generates a prototype for every function definition in a
// .ino and emits it without the `template<...>` header, which fails to compile
// with "variable or field declared void".  Do not "clean this up" into a
// template unless this file becomes a real .cpp.
#define SETUP_ONE_CAN(Can, handler)                                 \
  do {                                                              \
    Can.begin();                                                    \
    Can.setBaudRate(kCanBaud);                                      \
    Can.setMaxMB(16);                                               \
    for (int i = 0; i < 8; ++i) Can.setMB((FLEXCAN_MAILBOX)i, RX, EXT); \
    for (int i = 8; i < 16; ++i) Can.setMB((FLEXCAN_MAILBOX)i, TX);  \
    Can.setMBFilter(ACCEPT_ALL);                                    \
    Can.enableMBInterrupts();                                       \
    Can.onReceive(handler);                                         \
    /* Deliberately no Can.events() anywhere in this sketch -- see  \
     * the comment on canRxCommon(). */                             \
  } while (0)

void setupCan() {
  SETUP_ONE_CAN(Can1, canRx1);
  SETUP_ONE_CAN(Can2, canRx2);
  SETUP_ONE_CAN(Can3, canRx3);
  debug_print("[CAN] CAN1/CAN2/CAN3 up at %lu baud\n", (unsigned long)kCanBaud);
}

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
  }
#endif
  // Board identity first and on its own lines: a board flashed with the wrong
  // DASH_BOARD_ID is the one failure here that looks like a network problem
  // rather than a build problem, so it should be impossible to miss.
  debug_print("\n=====================================================\n");
  debug_print("[dash_v1] board id %d = \"%s\" at 192.168.0.%u\n", DASH_BOARD_ID,
              kBoardName, kBoardIpLastOctet);
  debug_print("[dash_v1] protocol v%u\n", kDashProtoVersion);
  debug_print("=====================================================\n");

  WDT_timings_t config;
  config.trigger = 2; /* seconds */
  config.timeout = 5; /* seconds */
  config.callback = wdCallback;
  wdt.begin(config);

  setupEthernet();
  if (udp.begin(kDashUdpPort)) {
    debug_print("[UDP] listening on %u\n", kDashUdpPort);
  } else {
    debug_print("[UDP] failed to start\n");
  }
  setupCan();
}

// Drains the socket and keeps only the newest valid command, so a backlog
// cannot make us lag behind the host by a growing number of cycles.
bool receiveUdp() {
  bool got_new = false;
  DashCmdPacket pkt;
  while (true) {
    const int size = udp.parsePacket();
    if (size <= 0) break;
    if (size != (int)sizeof(DashCmdPacket)) continue;
    memcpy(&pkt, udp.data(), sizeof(pkt));
    // Validates magic, version, length, CRC *and* that every routing entry
    // names a real channel and a nonzero id.
    if (!dash_check_cmd(&pkt, size)) continue;

    g_cmd = pkt;
    got_new = true;
    g_host_ip = udp.remoteIP();
    g_host_port = udp.remotePort();
  }

  if (got_new) {
    if (!g_have_cmd) {
      // The host's view of this board, echoed back.  If these motors are not
      // the ones cabled to this Teensy, DASH_BOARD_ID and kDashBoards disagree
      // about which board is which.
      const IPAddress from = udp.remoteIP();
      debug_print("[UDP] first command from %u.%u.%u.%u:%u -- %u motor(s) for "
                  "\"%s\"\n",
                  from[0], from[1], from[2], from[3], udp.remotePort(),
                  g_cmd.n_motors, kBoardName);
      for (uint8_t i = 0; i < g_cmd.n_motors; ++i) {
        debug_print("[UDP]   slot %u -> CAN%u id %u\n", i, g_cmd.bus[i] + 1,
                    g_cmd.can_id[i]);
      }
    }
    // Publish the routing before any reply can arrive for it.  CAN is idle at
    // this point: the motors only ever answer a frame we sent.
    for (uint8_t i = 0; i < g_cmd.n_motors; ++i) {
      g_bus[i] = g_cmd.bus[i];
      g_can_id[i] = g_cmd.can_id[i];
    }
    g_n_motors = g_cmd.n_motors;

    g_have_cmd = true;
    g_last_cmd_ms = millis();
  }
  return got_new;
}

// One command burst across all channels, then a bounded wait for every reply.
uint16_t runCanCycle() {
  const uint8_t n = g_n_motors;
  for (uint8_t i = 0; i < n; ++i) g_reply_valid[i] = false;

  // Queue every channel before waiting on any of them, so the channels are busy
  // at the same time instead of one after another.
  CAN_message_t msg;
  msg.len = 8;
  msg.flags.extended = 1;  // Dash controllers use 29-bit ids
  for (uint8_t i = 0; i < n; ++i) {
    msg.id = g_can_id[i];
    memcpy(msg.buf, g_cmd.frame[i], 8);
    canWrite(g_bus[i], msg);
    if (kCanSendDelayUs > 0) delayMicroseconds(kCanSendDelayUs);
  }

  // Pure spin: the onReceive callbacks run in interrupt context, so the flags
  // become true the instant each reply lands.  Nothing to poll or drain.
  const uint32_t start = micros();
  while ((micros() - start) < kCanResponseTimeoutUs) {
    bool all = true;
    for (uint8_t i = 0; i < n; ++i) {
      if (!g_reply_valid[i]) {
        all = false;
        break;
      }
    }
    if (all) break;
  }

  uint16_t mask = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (g_reply_valid[i]) mask |= (uint16_t)(1u << i);
  }
  return mask;
}

void sendUdpState(uint16_t online_mask) {
  DashStatePacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = kDashProtoMagicState;
  pkt.version = kDashProtoVersion;
  pkt.n_motors = g_n_motors;
  pkt.seq = g_cmd.seq;
  pkt.online_mask = online_mask;
  pkt.teensy_us = micros();
  for (uint8_t i = 0; i < g_n_motors; ++i) {
    for (int b = 0; b < 8; ++b) pkt.frame[i][b] = g_reply[i][b];
  }
  dash_seal_state(&pkt);

  if (!udp.send(g_host_ip, g_host_port, (const uint8_t *)&pkt, sizeof(pkt))) {
    debug_print("[UDP] send failed\n");
  }
}

void loop() {
  wdt.feed();

  const bool got_new = receiveUdp();

  if (g_have_cmd && (millis() - g_last_cmd_ms) > kHostTimeoutMs) {
    debug_print("[UDP] host timeout -- going quiet on CAN\n");
    g_have_cmd = false;
  }
  if (!g_have_cmd || !got_new) return;

  const uint16_t online_mask = runCanCycle();
  sendUdpState(online_mask);

#ifdef DEBUG_MODE
  // Report missing motors at most once a second so a wiring fault is obvious
  // without drowning the console.
  static uint32_t last_warn_ms = 0;
  const uint16_t all_online = (uint16_t)((1u << g_n_motors) - 1);
  if (online_mask != all_online && (millis() - last_warn_ms) > 1000) {
    last_warn_ms = millis();
    debug_print("[CAN] no reply from:");
    for (uint8_t i = 0; i < g_n_motors; ++i) {
      if (!((online_mask >> i) & 1u)) {
        debug_print(" CAN%u/id%u", g_bus[i] + 1, g_can_id[i]);
      }
    }
    debug_print("\n");
  }
#endif
}

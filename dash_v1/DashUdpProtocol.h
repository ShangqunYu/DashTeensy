#ifndef DASH_UDP_PROTOCOL_H
#define DASH_UDP_PROTOCOL_H

/*
 * Wire protocol between the robot computer (upxtreme) and a Dash Teensy 4.1
 * relay board.
 *
 * This file must be byte-identical on both sides.  Edit it here, then copy it
 * into the Teensy sketch folder -- see DashTeensy/dash_v1/README.md.
 *
 * The Teensy is deliberately dumb: it forwards raw 8-byte CAN payloads in both
 * directions and knows nothing about MIT scaling, motor limits or modes.  All
 * of that lives on the host in DashMotorProtocol.h, where it can be debugged
 * without reflashing.
 *
 * That extends to the wiring: every command carries its own routing (which CAN
 * channel, which motor id), so the Teensy holds no motor table of its own and
 * needs no reflash when motors move between channels.  The host's
 * kDashBoards in DashHardwareConfig.h is the single source of truth.
 *
 * Bump kDashProtoVersion on any layout change so a stale binary on either end
 * is rejected loudly instead of silently misparsed.
 */

#include <stdint.h>
#include <string.h>

static constexpr uint16_t kDashProtoMagicCmd = 0xDA51;
static constexpr uint16_t kDashProtoMagicState = 0xDA52;
/* v2: added per-motor bus/can_id routing to DashCmdPacket. */
static constexpr uint8_t kDashProtoVersion = 2;

/* Both packets are a compile-time constant size regardless of how many motors
 * are actually wired, so a truncated or corrupt datagram is caught by a single
 * length comparison.  n_motors says how many leading slots are meaningful. */
static constexpr uint8_t kDashMaxMotors = 12;

/* FlexCAN peripherals on a Teensy 4.1: 0=CAN1, 1=CAN2, 2=CAN3. */
static constexpr uint8_t kDashNumCanBuses = 3;

static constexpr uint16_t kDashUdpPort = 8004;

#pragma pack(push, 1)

/* Host -> Teensy.  One raw CAN payload per motor, in motor-index order. */
struct DashCmdPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t n_motors;
  uint32_t seq;
  /* Routing for slot i.  bus is a FlexCAN index (0=CAN1, 1=CAN2, 2=CAN3) and
   * can_id is the motor's flashed CFG_CAN_ID, sent as a 29-bit extended id.
   * Motor ids are only unique *within* a bus -- (bus, can_id) together are what
   * identify a motor, which is why both travel with every command. */
  uint8_t bus[kDashMaxMotors];
  uint8_t can_id[kDashMaxMotors];
  uint8_t frame[kDashMaxMotors][8];
  uint8_t crc; /* CRC-8 over every preceding byte */
};

/* Teensy -> Host.  One raw CAN reply per motor, in the same index order. */
struct DashStatePacket {
  uint16_t magic;
  uint8_t version;
  uint8_t n_motors;
  uint32_t seq;         /* seq of the command this state answers */
  uint16_t online_mask; /* bit i set = motor i replied on CAN this cycle */
  uint32_t teensy_us;   /* micros() on the Teensy when the packet was built */
  uint8_t frame[kDashMaxMotors][8];
  uint8_t crc;
};

#pragma pack(pop)

/* CRC-8, Dallas/Maxim polynomial -- same one the Prestoe boards use. */
static inline uint8_t dash_crc8(const uint8_t *data, uint32_t length) {
  uint8_t crc = 0;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline void dash_seal_cmd(DashCmdPacket *pkt) {
  pkt->crc = dash_crc8(reinterpret_cast<const uint8_t *>(pkt),
                       sizeof(DashCmdPacket) - 1);
}

static inline void dash_seal_state(DashStatePacket *pkt) {
  pkt->crc = dash_crc8(reinterpret_cast<const uint8_t *>(pkt),
                       sizeof(DashStatePacket) - 1);
}

static inline bool dash_check_cmd(const DashCmdPacket *pkt, int nbytes) {
  if (nbytes != (int)sizeof(DashCmdPacket)) return false;
  if (pkt->magic != kDashProtoMagicCmd) return false;
  if (pkt->version != kDashProtoVersion) return false;
  if (pkt->n_motors > kDashMaxMotors) return false;
  if (pkt->crc != dash_crc8(reinterpret_cast<const uint8_t *>(pkt),
                            sizeof(DashCmdPacket) - 1)) {
    return false;
  }
  /* Routing sanity, checked only once the packet is known intact.  can_id 0 is
   * CFG_CAN_MASTER -- the id the motors *reply* on -- so it is never a valid
   * destination, and transmitting to it would look like a reply to every board
   * on the bus. */
  for (uint8_t i = 0; i < pkt->n_motors; ++i) {
    if (pkt->bus[i] >= kDashNumCanBuses) return false;
    if (pkt->can_id[i] == 0) return false;
  }
  return true;
}

static inline bool dash_check_state(const DashStatePacket *pkt, int nbytes) {
  return nbytes == (int)sizeof(DashStatePacket) &&
         pkt->magic == kDashProtoMagicState &&
         pkt->version == kDashProtoVersion && pkt->n_motors <= kDashMaxMotors &&
         pkt->crc == dash_crc8(reinterpret_cast<const uint8_t *>(pkt),
                               sizeof(DashStatePacket) - 1);
}

#endif /* DASH_UDP_PROTOCOL_H */

#ifndef DASH_IMU_PROTOCOL_H
#define DASH_IMU_PROTOCOL_H

/*
 * Wire protocol between the robot computer (upxtreme) and the Dash IMU Teensy.
 *
 * This file must be byte-identical on both sides.  The DARoS-Core copy under
 * Systems/DashSystem/RobotHardware/ is the source of truth and the one in
 * DashTeensy/dash_imu_v1/ is the copy -- same arrangement as
 * DashUdpProtocol.h.  See README.md.
 *
 * Unlike the motor relay boards, this one is not polled.  The VN-100 pushes
 * samples at its own configured rate and the Teensy forwards each one as it
 * arrives, so there is no command packet and no request/response cycle: the
 * host just listens.
 *
 * The same board also carries the RC receiver (FrSky X8R, SBUS on Serial7).
 * Its DashRcPacket travels on the same stream as DashImuPacket -- one hello
 * subscribes to both -- and a reader tells them apart by magic and size.
 *
 * The board still has to be told WHERE to send.  A host announces itself with
 * a DashImuHello and the Teensy streams to that datagram's source address and
 * port until the hello stops arriving -- the same "reply to whoever asked"
 * rule dash_v1 uses, and for the same reason: it lets several processes on
 * different ports subscribe without anything being hard-coded.  A compiled-in
 * default host keeps a bare `flash it and look` workflow working; see
 * DASH_IMU_AUTOSTREAM in the sketch.
 *
 * Bump kDashImuProtoVersion on any layout change so a stale binary on either
 * end is rejected loudly instead of silently misparsed.
 */

#include <stdint.h>

static constexpr uint16_t kDashImuMagicData = 0xDA61;
static constexpr uint16_t kDashImuMagicHello = 0xDA62;
static constexpr uint16_t kDashRcMagicData = 0xDA63;
static constexpr uint8_t kDashImuProtoVersion = 1;

/* One past the two relay boards (8004, and 8005 as their host-side port for
 * board index 1 -- but those are HOST ports, not board ports, so there is no
 * clash on the Teensy side).  Keeping it adjacent makes the whole robot's UDP
 * traffic one contiguous range in tcpdump. */
static constexpr uint16_t kDashImuUdpPort = 8006;

/* status bits in DashImuPacket and DashRcPacket. */
static constexpr uint8_t kDashImuStatusLink = 1u << 0;       /* ethernet link up */
static constexpr uint8_t kDashImuStatusSubscribed = 1u << 1; /* a host said hello */
/* Sticky: at least one VN-100 frame arrived whose group/field header was not
 * the one this firmware expects.  Almost always means register 75 was never
 * written, or was written with different fields -- see README.md. */
static constexpr uint8_t kDashImuStatusBadFormat = 1u << 2;

/* flags bits in DashRcPacket.  The first four are the SBUS frame's own flag
 * byte; NoSignal is the Teensy's. */
static constexpr uint8_t kDashRcFlagLostFrame = 1u << 0; /* receiver missed a radio frame */
static constexpr uint8_t kDashRcFlagFailsafe = 1u << 1;  /* receiver has lost the radio */
static constexpr uint8_t kDashRcFlagCh17 = 1u << 2;      /* SBUS digital channel 17 */
static constexpr uint8_t kDashRcFlagCh18 = 1u << 3;      /* SBUS digital channel 18 */
/* No SBUS frame for kDashRcNoSignalMs.  The packet is a heartbeat and ch[]
 * holds the last frame received, or zeros if there never was one. */
static constexpr uint8_t kDashRcFlagNoSignal = 1u << 4;
static constexpr uint16_t kDashRcNoSignalMs = 100;

static constexpr int kDashRcNumChannels = 16;

#pragma pack(push, 1)

/* Teensy -> host.  One packet per VN-100 sample that passed its own CRC. */
struct DashImuPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t status;
  uint32_t seq;      /* increments once per forwarded sample; gaps = drops */
  uint32_t teensy_us;/* micros() when the sample finished parsing */
  uint16_t bad_crc;  /* cumulative VN-100 frames failing CRC (saturates) */
  uint16_t resyncs;  /* cumulative times the parser had to hunt for sync */
  /* VN-100 register 75 payload, in the order the IMU sends it.
   *
   * quat is (x, y, z, w) -- the SCALAR IS LAST.  Eigen::Quaternionf takes
   * (w, x, y, z) in its constructor, so a straight memcpy into one is wrong.
   * gyro is rad/s and accel is m/s^2, both in the IMU body frame; neither has
   * been rotated into the robot frame here. */
  float quat[4];
  float gyro[3];
  float accel[3];
  uint8_t crc; /* CRC-8 over every preceding byte */
};

/* Teensy -> host.  One packet per SBUS frame decoded, plus a heartbeat every
 * 100 ms while none arrive, so a silent receiver reads differently from a
 * board that has gone away.
 *
 * Depending on the failsafe mode set on the radio, a receiver that loses the
 * link either keeps sending frames with Failsafe set or stops sending, which
 * shows up here as NoSignal.  Check both -- dash_rc_usable() does. */
struct DashRcPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t status;       /* kDashImuStatus* bits, as in DashImuPacket */
  uint32_t seq;         /* increments per RC packet, heartbeats included */
  uint32_t teensy_us;   /* micros() when the frame was decoded */
  uint8_t flags;        /* kDashRcFlag* */
  uint16_t lost_frames; /* cumulative frames with LostFrame set (saturates) */
  uint16_t age_ms;      /* since the last SBUS frame; 0xFFFF if none ever */
  /* Raw 11-bit SBUS values, 0..2047.  FrSky sends 172 / 992 / 1811 for
   * -100 / 0 / +100 %.  Which stick or switch is on which channel is set by
   * the model's mixer on the radio, not by anything here. */
  uint16_t ch[kDashRcNumChannels];
  uint8_t crc; /* CRC-8 over every preceding byte */
};

/* Host -> Teensy.  Says "send here"; resend it a few times a second. */
struct DashImuHello {
  uint16_t magic;
  uint8_t version;
  uint8_t flags; /* reserved, send 0 */
  uint8_t crc;
};

#pragma pack(pop)

/* CRC-8, Dallas/Maxim polynomial.  Deliberately a separate copy of the routine
 * in DashUdpProtocol.h rather than an include: an IMU board has no business
 * carrying the motor relay's protocol header, and this keeps the sketch folder
 * to one file to copy.  The polynomial must stay 0x31 in both. */
static inline uint8_t dash_imu_crc8(const uint8_t *data, uint32_t length) {
  uint8_t crc = 0;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline void dash_imu_seal(DashImuPacket *pkt) {
  pkt->crc = dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                           sizeof(DashImuPacket) - 1);
}

static inline void dash_rc_seal(DashRcPacket *pkt) {
  pkt->crc = dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                           sizeof(DashRcPacket) - 1);
}

static inline void dash_imu_seal_hello(DashImuHello *pkt) {
  pkt->crc = dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                           sizeof(DashImuHello) - 1);
}

static inline bool dash_imu_check(const DashImuPacket *pkt, int nbytes) {
  return nbytes == (int)sizeof(DashImuPacket) &&
         pkt->magic == kDashImuMagicData &&
         pkt->version == kDashImuProtoVersion &&
         pkt->crc == dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                                   sizeof(DashImuPacket) - 1);
}

static inline bool dash_rc_check(const DashRcPacket *pkt, int nbytes) {
  return nbytes == (int)sizeof(DashRcPacket) &&
         pkt->magic == kDashRcMagicData &&
         pkt->version == kDashImuProtoVersion &&
         pkt->crc == dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                                   sizeof(DashRcPacket) - 1);
}

/* Whether ch[] reflects what the pilot is doing now.  LostFrame alone does not
 * disqualify a packet: the receiver repeats the last good values for a missed
 * radio frame, and the odd one is normal. */
static inline bool dash_rc_usable(const DashRcPacket *pkt) {
  return (pkt->flags & (kDashRcFlagFailsafe | kDashRcFlagNoSignal)) == 0;
}

static inline bool dash_imu_check_hello(const DashImuHello *pkt, int nbytes) {
  return nbytes == (int)sizeof(DashImuHello) &&
         pkt->magic == kDashImuMagicHello &&
         pkt->version == kDashImuProtoVersion &&
         pkt->crc == dash_imu_crc8(reinterpret_cast<const uint8_t *>(pkt),
                                   sizeof(DashImuHello) - 1);
}

#endif /* DASH_IMU_PROTOCOL_H */

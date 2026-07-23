#include "TeensyCANInterface.h"
#include <QNEthernet.h>
#include <string>
#include <queue>
#include <array>
#include <algorithm>
#include <lwip/ip_addr.h>
#include <variant>
#include <numeric>
#include <cmath>
#include "Watchdog_t4.h"

// #define DEBUG_RESPONSE
// #define DEBUG_COMMAND
#define DEBUG_MODE
// #define  LATENCY_TEST
// #define PRINT_CAN_TX   // enable Serial prints of outgoing CAN frames
#define DISABLE_LATENCY_PROFILING  // cycle latency globals/stats/printf; does not affect LATENCY_TEST

// Serial.printf wrapper: DEBUG_MODE (full debug), LATENCY_TEST / PRINT_CAN_TX, or cycle profiling build.
#if defined(DEBUG_MODE) || defined(LATENCY_TEST) || defined(PRINT_CAN_TX) || !defined(DISABLE_LATENCY_PROFILING)
#define debug_print(...) Serial.printf(__VA_ARGS__)
#else
#define debug_print(...) do {} while (0)
#endif

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define RESPONSE_TIMEOUT 1000
#define MAX_NODES 2  // SET:: maximum number of nodes per CAN channel

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

WDT_T4<WDT1> wdt;
EthernetUDP udp;
TeensyCAN* CanArray[3]; 

//======================================================================
//            Ethernet Settings
//======================================================================
constexpr uint16_t kPort = 8009;  // udp port
const char * HOST_IP = "192.168.0.110";
constexpr uint8_t TEENSY_IP[4] = { 192, 168, 0, 114 };
constexpr uint8_t TEENSY_DNS[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_GATEWAY[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_SUBNET[4] = { 255, 255, 255, 0 };

//================================-======================================
//            CAN Settings
//======================================================================
// Your setup:
// - CAN1 (first CAN channel): 2 actuators (IDs 1..2)
// - CAN3 (third CAN channel): 2 actuators (IDs 1..2)
constexpr uint8_t NUM_BUSSES = 2;
uint8_t NUM_NODES[NUM_BUSSES] = {2, 2};
// BUS_IDS entries are FlexCAN instances: 0->CAN1, 1->CAN2, 2->CAN3
uint8_t BUS_IDS[NUM_BUSSES] = {0, 2};
// Map FlexCAN bus index (0:CAN1,1:CAN2,2:CAN3) -> compact bus index (0..NUM_BUSSES-1) or -1 unused.
int8_t can_bus_idx[3] = {0, -1, 1};
long send_delay_us = 5;
//========================================================================

uint8_t TOTAL_NODES = std::accumulate(NUM_NODES, NUM_NODES + NUM_BUSSES, 0); 
volatile bool responseFlag[NUM_BUSSES][MAX_NODES] = {false};
volatile bool responseLastActFlag[NUM_BUSSES] = {false};

uint8_t can_data[NUM_BUSSES][MAX_NODES][8];     // CAN data buffer for each node
uint8_t can_command[NUM_BUSSES][MAX_NODES][8];  // CAN command buffer for each node

#ifndef DISABLE_LATENCY_PROFILING
// Cycle latency measurement (command burst start -> last expected response or timeout)
volatile unsigned long g_cycle_latency_us = 0;
volatile bool g_cycle_timed_out = false;
// Per-bus bitmask of missing node responses at end of cycle.
// Bit i corresponds to node i (0-based). Only bits < NUM_NODES[bus] are meaningful.
volatile uint8_t g_cycle_missing_mask[NUM_BUSSES] = {0};
#endif

unsigned long last_packet_time_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned long total_latency_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned int packet_count_bus[NUM_BUSSES][MAX_NODES] = { 0 };
constexpr int MAX_NUM_SAMPLES = 5000;
const char* CAN_BUS_NAMES[] = { "CAN0", "CAN1", "CAN2" };

#ifndef DISABLE_LATENCY_PROFILING
//======================================================================
//            Cycle latency stats (print every MAX_NUM_SAMPLES cycles)
//======================================================================
static uint32_t g_lat_samples[MAX_NUM_SAMPLES];
static uint32_t g_lat_count = 0;
static uint32_t g_lat_timeouts = 0;
static double g_lat_mean = 0.0;
static double g_lat_M2 = 0.0;  // sum of squares of differences from the current mean (Welford)
static uint32_t g_lat_min = 0;
static uint32_t g_lat_max = 0;

static void latency_stats_reset() {
  g_lat_count = 0;
  g_lat_timeouts = 0;
  g_lat_mean = 0.0;
  g_lat_M2 = 0.0;
  g_lat_min = 0;
  g_lat_max = 0;
}

static void latency_stats_add(uint32_t x_us, bool timed_out) {
  if (g_lat_count == 0) {
    g_lat_min = x_us;
    g_lat_max = x_us;
  } else {
    if (x_us < g_lat_min) g_lat_min = x_us;
    if (x_us > g_lat_max) g_lat_max = x_us;
  }

  // Welford update
  const double x = static_cast<double>(x_us);
  const double n1 = static_cast<double>(g_lat_count);
  const double n = n1 + 1.0;
  const double delta = x - g_lat_mean;
  g_lat_mean += delta / n;
  const double delta2 = x - g_lat_mean;
  g_lat_M2 += delta * delta2;

  if (timed_out) g_lat_timeouts++;

  if (g_lat_count < MAX_NUM_SAMPLES) {
    g_lat_samples[g_lat_count] = x_us;
    g_lat_count++;
  }
}

static uint32_t percentile_from_sorted(const uint32_t *sorted, uint32_t n, double p) {
  if (n == 0) return 0;
  if (p <= 0.0) return sorted[0];
  if (p >= 100.0) return sorted[n - 1];
  const double idx = (p / 100.0) * (static_cast<double>(n - 1));
  const uint32_t i = static_cast<uint32_t>(idx);
  return sorted[i];
}
#endif

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

// Track the last UDP sender so we can reply to it (avoid hard-coded HOST_IP).
IPAddress g_last_remote_ip;
uint16_t g_last_remote_port = 0;
bool g_have_remote = false;

// Calculate CRC-8 checksum
// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;
uint8_t calculate_crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void canReceive_i(const CAN_message_t &msg, int bus_id) {
  int node_id = msg.buf[0] - 1;

  if(node_id>=0 && node_id <NUM_NODES[can_bus_idx[bus_id]]){
      memcpy(can_data[can_bus_idx[bus_id]][node_id], msg.buf, 8);
      responseFlag[can_bus_idx[bus_id]][node_id] = true;

      if (node_id == (NUM_NODES[can_bus_idx[bus_id]] - 1)) {
            responseLastActFlag[can_bus_idx[bus_id]] = true;
        }
  }else{
  }
  #ifdef LATENCY_TEST
    if (node_id >= 0 && node_id < NUM_NODES[can_bus_idx[bus_id]]) {
      unsigned long current_time = micros();
      unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[bus_id]][node_id];
      total_latency_bus[can_bus_idx[bus_id]][node_id] += latency;
      packet_count_bus[can_bus_idx[bus_id]][node_id]++;
  
      if (packet_count_bus[can_bus_idx[bus_id]][node_id] == MAX_NUM_SAMPLES) {
        unsigned long average_latency = total_latency_bus[can_bus_idx[bus_id]][node_id] / MAX_NUM_SAMPLES;
        debug_print("Average latency of %d CAN receives for node %d on bus %d: %lu us\n", MAX_NUM_SAMPLES, node_id + 1,BUS_IDS[bus_id],average_latency);
        total_latency_bus[can_bus_idx[bus_id]][node_id] = 0;
        packet_count_bus[can_bus_idx[bus_id]][node_id] = 0;
      }

      last_packet_time_bus[can_bus_idx[bus_id]][node_id] = current_time;
    }
  #endif
}

void CanReceive1(const CAN_message_t &msg) { canReceive_i(msg, 0); }
void CanReceive2(const CAN_message_t &msg) { canReceive_i(msg, 1); }
void CanReceive3(const CAN_message_t &msg) { canReceive_i(msg, 2); }

void (*canReceiveFuncs[])(const CAN_message_t &) = {CanReceive1, CanReceive2, CanReceive3};

void wdCallback() {
#ifdef DEBUG_MODE
  debug_print("FEED THE DOG SOON, OR RESET!\n");
#endif
}

void setup() {
  debug_print("Starting setup\n");
  #if defined(DEBUG_MODE) || defined(LATENCY_TEST) || defined(PRINT_CAN_TX) || !defined(DISABLE_LATENCY_PROFILING)
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for Serial
  }
  #endif

  // watchdog timer setting
  WDT_timings_t config;
  config.trigger = 2; /* in seconds, 0->128 */
  config.timeout = 5; /* in seconds, 0->128 */
  config.callback = wdCallback;
  wdt.begin(config);

  CanArray[0] = new FlexCANWrapper<CAN1, RX_SIZE_256, TX_SIZE_16>(&Can0);
  CanArray[1] = new FlexCANWrapper<CAN2, RX_SIZE_256, TX_SIZE_16>(&Can1);
  CanArray[2] = new FlexCANWrapper<CAN3, RX_SIZE_256, TX_SIZE_16>(&Can2);

#ifdef DEBUG_MODE
  debug_print("Starting...\r\n");
  debug_print("Total Nodes: %d \n", TOTAL_NODES);
#endif
  debug_print("Setting up Ethernet\n");
  setupEthernet();
  debug_print("Setting up UDP\n");
  setupUDP();
  debug_print("Setting up CAN\n");
  setupCAN();

  #ifndef DISABLE_LATENCY_PROFILING
  latency_stats_reset();
  #endif
}


void loop() {
  
  wdt.feed();
  static int _iter=0;
  receiveUDPPacket();
  if (!first_packet_recv){
    return;
  }
  
  sendCANCMD();
  sendUDPPacket();
  
  #ifndef DISABLE_LATENCY_PROFILING
  // Accumulate cycle latency stats and print a summary every MAX_NUM_SAMPLES cycles.
  latency_stats_add(static_cast<uint32_t>(g_cycle_latency_us), g_cycle_timed_out);
  if (g_lat_count >= MAX_NUM_SAMPLES) {
    // Copy + sort to compute median/percentiles.
    static uint32_t tmp[MAX_NUM_SAMPLES];
    memcpy(tmp, g_lat_samples, sizeof(tmp));
    std::sort(tmp, tmp + MAX_NUM_SAMPLES);

    const uint32_t median =
        (MAX_NUM_SAMPLES % 2 == 1)
            ? tmp[MAX_NUM_SAMPLES / 2]
            : static_cast<uint32_t>((static_cast<uint64_t>(tmp[MAX_NUM_SAMPLES / 2 - 1]) +
                                     static_cast<uint64_t>(tmp[MAX_NUM_SAMPLES / 2])) /
                                    2ull);
    const uint32_t p90 = percentile_from_sorted(tmp, MAX_NUM_SAMPLES, 90.0);
    const uint32_t p95 = percentile_from_sorted(tmp, MAX_NUM_SAMPLES, 95.0);
    const uint32_t p99 = percentile_from_sorted(tmp, MAX_NUM_SAMPLES, 99.0);

    const double variance =
        (MAX_NUM_SAMPLES > 1) ? (g_lat_M2 / static_cast<double>(MAX_NUM_SAMPLES - 1)) : 0.0;
    const double std_us = std::sqrt(variance);

    debug_print("cycle_latency_stats n=%u mean_us=%.2f median_us=%lu std_us=%.2f min_us=%lu max_us=%lu p90_us=%lu p95_us=%lu p99_us=%lu timeouts=%lu\n",
           (unsigned)MAX_NUM_SAMPLES,
           g_lat_mean,
           (unsigned long)median,
           std_us,
           (unsigned long)g_lat_min,
           (unsigned long)g_lat_max,
           (unsigned long)p90,responseLastActFlag
           (unsigned long)p95,
           (unsigned long)p99,
           (unsigned long)g_lat_timeouts);

    latency_stats_reset();
  }
  #endif

  _iter++;
}

// Ethernet setup
void setupEthernet() {
  uint8_t mac[6];
  Ethernet.macAddress(mac);
  #ifdef DEBUG_MODE
  debug_print("MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  #endif

  IPAddress ip(TEENSY_IP[0], TEENSY_IP[1], TEENSY_IP[2], TEENSY_IP[3]);
  IPAddress dns(TEENSY_DNS[0], TEENSY_DNS[1], TEENSY_DNS[2], TEENSY_DNS[3]);
  IPAddress gateway(TEENSY_GATEWAY[0], TEENSY_GATEWAY[1], TEENSY_GATEWAY[2], TEENSY_GATEWAY[3]);
  IPAddress subnet(TEENSY_SUBNET[0], TEENSY_SUBNET[1], TEENSY_SUBNET[2], TEENSY_SUBNET[3]);

  Ethernet.onLinkState([](bool state) {
#ifdef DEBUG_MODE
    debug_print("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
#endif
  });

  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    if (ip != INADDR_NONE) {
#ifdef DEBUG_MODE
      debug_print("[Ethernet] Address changed:");
      // printNetworkConfig();
      printIPAddress();
#endif
    } else {
#ifdef DEBUG_MODE
      debug_print("[Ethernet] Address changed: No IP address");
#endif
    }
  });

  if (!Ethernet.begin(mac, ip, dns, gateway, subnet)) {
#ifdef DEBUG_MODE
    debug_print("Failed to start Ethernet");
#endif
  } else {
#ifdef DEBUG_MODE
    debug_print("Ethernet setup complete. Ready to respond to ping.");
#endif
  }
}

void setupUDP() {
  if (udp.begin(kPort)) {
#ifdef DEBUG_MODE
    debug_print("UDP server started on port %d.\n", kPort);
#endif
  } else {
#ifdef DEBUG_MODE
    debug_print("Failed to start UDP server.");
#endif
  }
}

void setupCAN() {

  for(int i=0;i<NUM_BUSSES;++i){

    if(CanArray[BUS_IDS[i]]){

      TeensyCAN* can_i = CanArray[BUS_IDS[i]];

      can_i->begin();
      can_i->setBaudRate(1000000);
      can_i->setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
      can_i->setMBFilter(MB1, 0, 0x0);
      can_i->enhanceFilter(MB1);
      can_i->distribute();
      can_i->enableMBInterrupts();
      can_i->onReceive(canReceiveFuncs[BUS_IDS[i]]);
#ifdef DEBUG_MODE
      debug_print("Assigned CAN receive function for bus %d\n",BUS_IDS[i]);
#endif
      can_i->setClock(CLK_60MHz);


#ifdef DEBUG_MODE
    debug_print("Done setting up CAN %d\n", BUS_IDS[i]);
#endif
    }
  }
}

void printIPAddress() {
#ifdef DEBUG_MODE
  IPAddress ip = Ethernet.localIP();
  debug_print("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  debug_print("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.broadcastIP();
  debug_print("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  debug_print("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  debug_print("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
#endif
}

void receiveUDPPacket()
{       

    // noInterrupts();
    int size = udp.parsePacket();
    // Serial.println(size);
    if (size > 0) {
        // Capture where this packet came from so we can reply back to the sender.
        // g_last_remote_ip = udp.remoteIP();
        // g_last_remote_port = udp.remotePort();
        // g_have_remote = true;

        // Serial.printf("🔹 Received UDP packet of size: %d bytes\n", size);
        const uint8_t *data = udp.data();
        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
#ifdef DEBUG_MODE
            debug_print("Reset command received. Buffers and variables reset.\n");
#endif
        }
        else if (size >= TOTAL_NODES * 8) {
            // Serial.printf("None reset command, processing\n");
            if (!first_packet_recv) {
#ifdef DEBUG_MODE
                debug_print("first packet recv\n");
#endif
            }
            first_packet_recv = true;

            // Extract the payload data
            std::vector<uint8_t> payload(data, data + TOTAL_NODES * 8);

            // Calculate CRC-8 for the payload
            uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());

            if (size == TOTAL_NODES * 8 + 1)
            {
                const uint8_t received_crc = data[TOTAL_NODES * 8];

                if (received_crc == calculated_crc)
                {
                    // CRC check passed, process the data
                   int offset = 0;
                    for (int k = 0; k < NUM_BUSSES; k++)
                    {
                        std::vector<std::array<uint8_t, 8>> busCommands;
                        int nodes_per_bus = NUM_NODES[k]; // Cache the value for clarity and efficiency
                        for (int i = 0; i < nodes_per_bus; i++)
                        {
                            // Use memcpy for efficient data copying
                            memcpy(can_command[k][i], &data[offset], 8 * sizeof(data[0]));
                            offset += 8;
                           
                        }
                        
                    }
                }
                else
                {
                    // CRC check failed, discard the data
#ifdef DEBUG_MODE
                    debug_print("CRC check failed\n");
                    debug_print("Received CRC: %02X\n", received_crc);
                    debug_print("Calculated CRC: %02X\n", calculated_crc);
#endif
                }
                // printCANCommand();
            }
            else
            {
#ifdef DEBUG_MODE
                debug_print("Invalid packet size\n");
#endif
            }
        }
    }

    // interrupts();
}

void printCANCommand2(CAN_message_t msg) { 
  for (int j = 0; j < 8; j++) {
    debug_print("%02X ", msg.buf[j]);
  }
  debug_print("\n");
}


void printCANData(){
  for(int i=0;i<NUM_BUSSES;i++){
    debug_print("CAN data bus %d:\n", i + 1);
    for(int j=0;j<NUM_NODES[i];j++){
      debug_print("Node %d: ", j + 1);
      for(int k=0;k<8;k++){
        debug_print("%02X ", can_data[i][j][k]);
      }
      debug_print("\n");
    }
  }
}

void sendCANCMD() {
  
  static int _iter=0;
  // New cycle: clear all per-node/per-bus response flags so we only accept
  // responses belonging to this command burst.
  for (int j = 0; j < NUM_BUSSES; j++) {
    responseLastActFlag[j] = false;
    for (int i = 0; i < MAX_NODES; i++) {
      responseFlag[j][i] = false;
    }
  }

  CAN_message_t msg[NUM_BUSSES];

  // Phase 1: transmit commands to all nodes on all busses as a burst.
  const unsigned long cycle_start = micros();
  #ifndef DISABLE_LATENCY_PROFILING
  g_cycle_latency_us = 0;
  g_cycle_timed_out = false;
  for (int j = 0; j < NUM_BUSSES; j++) {
    g_cycle_missing_mask[j] = 0;
  }
  #endif
  for (int i = 0; i < MAX_NODES; i++) {  // Iterate over node IDs first
    for (int j = 0; j < NUM_BUSSES; j++) {  // Iterate over each CAN bus
        if (i < NUM_NODES[j]) {  // Ensure the node exists in this bus
            msg[j].id = i + 1;
            msg[j].len = 8;

            memcpy(msg[j].buf, can_command[j][i], 8);  // Copy command to message buffer

            TeensyCAN* can_i = CanArray[BUS_IDS[j]];

            if(can_i){
              #ifdef PRINT_CAN_TX
              debug_print("CAN_TX bus=%u can_id=0x%lX data=", BUS_IDS[j], (unsigned long)msg[j].id);
              for (int b = 0; b < 8; b++) {
                debug_print("%02X%s", msg[j].buf[b], (b == 7) ? "" : " ");
              }
              debug_print("\n");
              #endif
              can_i->write(msg[j]);  // Send message
              // Optional pacing to avoid overrunning slow transceivers / arbitration
              // on very busy busses. Keep this small to preserve "burst" behavior.
              if (send_delay_us > 0) {
                delayMicroseconds(send_delay_us);
              }
            }
        }
      }
  }

  // Phase 2: pump CAN events until *all* expected nodes responded (or timeout).
  while ((micros() - cycle_start) < RESPONSE_TIMEOUT) {
    // Process incoming frames on every bus.
    for (int j = 0; j < NUM_BUSSES; j++) {
      TeensyCAN* can_i = CanArray[BUS_IDS[j]];
      if (can_i) {
        can_i->events();
      }
    }

    // Completion condition: every node on every bus responded at least once.
    bool all_nodes = true;
    for (int j = 0; j < NUM_BUSSES && all_nodes; j++) {
      for (int i = 0; i < NUM_NODES[j]; i++) {
        if (!responseFlag[j][i]) {
          all_nodes = false;
          break;
        }
      }
    }
    if (all_nodes) {
      #ifndef DISABLE_LATENCY_PROFILING
      g_cycle_latency_us = micros() - cycle_start;
      #endif
      break;
    }

    // Small delay to reduce CPU spin while still keeping latency low.
    delayMicroseconds(5);
  }

  #ifndef DISABLE_LATENCY_PROFILING
  // End-of-cycle bookkeeping (compute missing actuators + final latency/timeout).
  bool all_nodes = true;
  for (int j = 0; j < NUM_BUSSES; j++) {
    uint8_t mask = 0;
    for (int i = 0; i < NUM_NODES[j]; i++) {
      if (!responseFlag[j][i]) {
        mask |= (1u << i);
        all_nodes = false;
      }
    }
    g_cycle_missing_mask[j] = mask;
  }

  if (g_cycle_latency_us == 0) {
    g_cycle_latency_us = micros() - cycle_start;
  }
  g_cycle_timed_out = !all_nodes;
  #endif

  _iter++;
}

bool allBusesCompleted() {
    // Kept for compatibility with older debugging logic.
    // NOTE: This checks only the last actuator per bus, which is not sufficient
    // to guarantee that *all* nodes responded.
    for (int j = 0; j < NUM_BUSSES; j++) {
        if (!responseLastActFlag[j]) {
            return false;
        }
    }
    return true;
}

void resetResponseFlags() {
    // Kept for compatibility with older debugging logic.
    for (int j = 0; j < NUM_BUSSES; j++) {
        responseLastActFlag[j] = false;
        for (int i = 0; i < MAX_NODES; i++) {
          responseFlag[j][i] = false;
        }
    }
}

void sendUDPPacket() {
  uint8_t buffer[TOTAL_NODES * 8];
  uint8_t *buffer_ptr = buffer;
  for (int j = 0; j < NUM_BUSSES; j++) {
    for (int i = 0; i < NUM_NODES[j]; i++) {
      memcpy(buffer_ptr, can_data[j][i], 8);
      buffer_ptr += 8;
    }
  }
  // if (!g_have_remote) {
  //   // No known destination yet (no incoming UDP command received).
  //   return;
  // }
  if (!udp.send(HOST_IP, kPort, buffer, TOTAL_NODES * 8)) {
#ifdef DEBUG_MODE
    debug_print("ERROR.");
#endif
  }
}

void reset() {
  memset(can_command, 0, sizeof(can_command));
  memset(can_data, 0, sizeof(can_data));
  first_packet_recv = false;
}


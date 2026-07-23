#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <string>
#include <queue>
#include <array>
#include <lwip/ip_addr.h>
#include <variant>
#include <numeric>
#include "Watchdog_t4.h"

// #define DEBUG_RESPONSE
// #define DEBUG_COMMAND
// #define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do {} while (0)
#endif

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define RESPONSE_TIMEOUT 1000
#define MAX_NODES 2  // Maximum number of nodes per bus

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -65.0f
#define T_MAX 65.0f

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

std::variant<
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*,
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*,
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*
> CanArray[] = { &Can0, &Can1, &Can2 };

WDT_T4<WDT1> wdt;

constexpr uint16_t kPort = 8003;  // udp port
constexpr int MAX_NUM_SAMPLES = 5000;

constexpr int NUM_BUSSES = 3;
int NUM_NODES[NUM_BUSSES] = {2, 2, 2};
int BUS_IDS[NUM_BUSSES]={0, 1, 2};

int TOTAL_NODES = std::accumulate(NUM_NODES, NUM_NODES + NUM_BUSSES, 0); 
volatile bool responseFlag[NUM_BUSSES][MAX_NODES] = {false};

EthernetUDP udp;
constexpr uint8_t TEENSY_IP[4] = { 192, 168, 0, 111 };
constexpr uint8_t TEENSY_DNS[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_GATEWAY[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_SUBNET[4] = { 255, 255, 255, 0 };


uint8_t can_data[NUM_BUSSES][MAX_NODES][8];     // CAN data buffer for each node
uint8_t can_command[NUM_BUSSES][MAX_NODES][8];  // CAN command buffer for each node
uint8_t can_bus_idx[3]={0,1,2};

unsigned long last_packet_time_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned long total_latency_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned int packet_count_bus[NUM_BUSSES][MAX_NODES] = { 0 };

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

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

void canReceive_i(const CAN_message_t &msg, int i) {
  int node_id = msg.buf[0] - 1;

  if(node_id>=0 && node_id <NUM_NODES[i]){
      memcpy(can_data[can_bus_idx[i]][node_id], msg.buf, 8);
      responseFlag[can_bus_idx[i]][node_id] = true;

    
      DEBUG_PRINT("Received response from CAN bus %d, node %d\n",i,node_id);
      
  }else{
      DEBUG_PRINT("Invalid node ID %d received on CAN bus %d\n",node_id,i);
  }
//   if (node_id >= 0 && node_id < NUM_NODES[i]) {
//     unsigned long current_time = micros();
//     unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[i]][node_id];
//     total_latency_bus[can_bus_idx[i]][node_id] += latency;
//     packet_count_bus[can_bus_idx[i]][node_id]++;
// // 
//     if (packet_count_bus[can_bus_idx[i]][node_id] == MAX_NUM_SAMPLES) {
//       // unsigned long average_latency = total_latency_bus[can_bus_idx[i]][node_id] / MAX_NUM_SAMPLES;
//       // printf("Average latency of %d CAN receives for node %d on bus %d: %lu us\n", MAX_NUM_SAMPLES, node_id + 1,BUS_IDS[i],average_latency);
//       total_latency_bus[can_bus_idx[i]][node_id] = 0;
//       packet_count_bus[can_bus_idx[i]][node_id] = 0;
//     }
// // 
//     last_packet_time_bus[can_bus_idx[i]][node_id] = current_time;
//   }
  
}

void CanReceive1(const CAN_message_t &msg) {
  canReceive_i(msg, 0);
}

void CanReceive2(const CAN_message_t &msg) {
  canReceive_i(msg, 1);
}

void CanReceive3(const CAN_message_t &msg) {
  canReceive_i(msg, 2);
}

void (*canReceiveFuncs[])(const CAN_message_t &) = {CanReceive1, CanReceive2, CanReceive3};

void wdCallback() {
  DEBUG_PRINT("FEED THE DOG SOON, OR RESET!");
}

void setup() {

  #ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for Serial
  }
  #endif

  WDT_timings_t config;
  config.trigger = 2; /* in seconds, 0->128 */
  config.timeout = 5; /* in seconds, 0->128 */
  config.callback = wdCallback;
  wdt.begin(config);

  DEBUG_PRINT("Starting...\r\n");
  DEBUG_PRINT("Total Nodes: %d \n", TOTAL_NODES);

  setupEthernet();
  setupUDP();
  setupCAN();
}


void loop() {
  
  wdt.feed();
  static int _iter=0;
  unsigned long start_time=micros();
  receiveUDPPacket();
  if (!first_packet_recv){
    return;
  }
  printf("=========================================\n");
  sendCANCMD();
  sendUDPPacket();
  printf("-----------------------------------------\n");
  unsigned long end_time=micros();
  unsigned long elapsed_time=end_time-start_time;
  // printf("CAN bandwidth (us): %lu \n",elapsed_time);
  _iter++;
}

// Ethernet setup
void setupEthernet() {
  uint8_t mac[6];
  Ethernet.macAddress(mac);
  #ifdef DEBUG_MODE
  Serial.print("MAC = ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();
  #endif

  IPAddress ip(TEENSY_IP[0], TEENSY_IP[1], TEENSY_IP[2], TEENSY_IP[3]);
  IPAddress dns(TEENSY_DNS[0], TEENSY_DNS[1], TEENSY_DNS[2], TEENSY_DNS[3]);
  IPAddress gateway(TEENSY_GATEWAY[0], TEENSY_GATEWAY[1], TEENSY_GATEWAY[2], TEENSY_GATEWAY[3]);
  IPAddress subnet(TEENSY_SUBNET[0], TEENSY_SUBNET[1], TEENSY_SUBNET[2], TEENSY_SUBNET[3]);

  Ethernet.onLinkState([](bool state) {
    DEBUG_PRINT("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    if (ip != INADDR_NONE) {
      DEBUG_PRINT("[Ethernet] Address changed:");
      // printNetworkConfig();
      printIPAddress();
    } else {
      DEBUG_PRINT("[Ethernet] Address changed: No IP address");
    }
  });

  if (!Ethernet.begin(mac, ip, dns, gateway, subnet)) {
    DEBUG_PRINT("Failed to start Ethernet");
  } else {
    DEBUG_PRINT("Ethernet setup complete. Ready to respond to ping.");
  }
}

void setupUDP() {
  if (udp.begin(kPort)) {
    DEBUG_PRINT("UDP server started on port %d.\n", kPort);
  } else {
    DEBUG_PRINT("Failed to start UDP server.");
  }
}

void setupCAN() {

  for(int i=0;i<NUM_BUSSES;++i){
    std::visit([&](auto&& can_i) {
      can_i->begin();
      can_i->setBaudRate(1000000);
      can_i->setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
      can_i->setMBFilter(MB1, 0, 0x0);
      can_i->enhanceFilter(MB1);
      can_i->distribute();
      can_i->enableMBInterrupts();
      can_i->onReceive(canReceiveFuncs[BUS_IDS[i]]);
      DEBUG_PRINT("Assigned CAN receive function for bus %d\n",BUS_IDS[i]);
      can_i->setClock(CLK_60MHz);

    }, CanArray[BUS_IDS[i]]);

    DEBUG_PRINT("Done setting up CAN %d\n", BUS_IDS[i]);
  }
}

void printIPAddress() {
  IPAddress ip = Ethernet.localIP();
  DEBUG_PRINT("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  DEBUG_PRINT("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.broadcastIP();
  DEBUG_PRINT("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  DEBUG_PRINT("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  DEBUG_PRINT("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

void receiveUDPPacket()
{       

    // noInterrupts();
    int size = udp.parsePacket();
    // Serial.println(size);
    if (size > 0) {
        // Serial.printf("🔹 Received UDP packet of size: %d bytes\n", size);
        const uint8_t *data = udp.data();
        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
            DEBUG_PRINT("Reset command received. Buffers and variables reset.\n");
        }
        else if (size >= TOTAL_NODES * 8) {
            // Serial.printf("None reset command, processing\n");
            if (!first_packet_recv)
                DEBUG_PRINT("first packet recv\n");
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
                            // memcpy(nodeCommand.data(), &data[offset], 8 * sizeof(data[0]));
                            offset += 8;
                            // printCANCommand3(can_command[k][i]);
                           
                        }
                        // commandQueue.push(busCommands);
                        
                    }
                }
                else
                {
                    // CRC check failed, discard the data
                    DEBUG_PRINT("CRC check failed\n");
                    DEBUG_PRINT("Received CRC: %02X\n", received_crc);
                    DEBUG_PRINT("Calculated CRC: %02X\n", calculated_crc);
                }
                // printCANCommand();
            }
            else
            {
                DEBUG_PRINT("Invalid packet size\n");
            }
        }
    }

    // interrupts();
}

void printCANCommand() {
  for (int k = 0; k < NUM_BUSSES; k++) {
    printf("CAN Command bus %d:\n", k + 1);
    for (int i = 0; i < NUM_NODES[k]; i++) {
      printf("Node %d: ", i + 1);
      for (int j = 0; j < 8; j++) {
        printf("%02X ", can_command[k][i][j]);
      }
      printf("\n");
    }
  }
}

void printCANCommand2(CAN_message_t msg) {
  // double value;
  // memcpy(&value, msg.buf, sizeof(double)); 
  for (int j = 0; j < 8; j++) {
    printf("%02X ", msg.buf[j]);
  }
  printf("\n");
  // Serial.print("Received double: ");
  // Serial.println(value, 10);
}

void printCANCommand3(uint8_t canmsg[8]){
  for (int j = 0; j < 8; j++) {
    printf("%02X ", canmsg[j]);
  }
  printf("\n");
}

void printCANData(){
  for(int i=0;i<NUM_BUSSES;i++){
    printf("CAN data bus %d:\n", i + 1);
    for(int j=0;j<NUM_NODES[i];j++){
      printf("Node %d: ", j + 1);
      for(int k=0;k<8;k++){
        printf("%02X ", can_data[i][j][k]);
      }
      printf("\n");
    }
  }
}

void waitForResponse(int bus, int node_id) {
  unsigned long start_time = micros();
  while (micros() - start_time < RESPONSE_TIMEOUT) {
    if (responseFlag[bus][node_id]) {
      responseFlag[bus][node_id] = false;

      // #ifdef DEBUG_RESPONSE
        Serial.printf("Response received from CAN bus %d, node %d\n",bus,node_id);
      // #endif
      return;
    }
  }
  // #ifdef DEBUG_RESPONSE
    Serial.printf("Timed out waiting for a response from CAN bus %d, node %d\n",bus,node_id);
  // #endif
}

void waitForResponses(int node_id) {
  unsigned long start_time = micros();
  while (micros() - start_time < RESPONSE_TIMEOUT) {
        bool all_responses_received = true;

        for (int bus = 0; bus < NUM_BUSSES; bus++) {
            if (!responseFlag[bus][node_id]) {
                all_responses_received = false;
                break;  // Exit early if any response is missing
            }
        }

        if (all_responses_received) {
            // Reset response flags
            for (int bus = 0; bus < NUM_BUSSES; bus++) {
                responseFlag[bus][node_id] = false;
            }

            // #ifdef DEBUG_RESPONSE
            Serial.printf("Responses received from node %d\n", node_id);
            // #endif
            return;
        }
    }

    // #ifdef DEBUG_RESPONSE
    Serial.printf("Timed out waiting for responses from node %d\n", node_id);
    // #endif
}

void sendCANCMD() {
  // noInterrupts();
  static int _iter=0;
  CAN_message_t msg[NUM_BUSSES];
  #ifdef DEBUG_COMMAND
    Serial.println("======================================");
  #endif
  for (int i = 0; i < MAX_NODES; i++) {  // Iterate over node IDs first
    for (int j = 0; j < NUM_BUSSES; j++) {  // Iterate over each CAN bus
        if (i < NUM_NODES[j]) {  // Ensure the node exists in this bus
            msg[j].id = i + 1;
            msg[j].len = 8;

            memcpy(msg[j].buf, can_command[j][i], 8);  // Copy command to message buffer

            auto* can_i = std::get_if<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *>(&CanArray[BUS_IDS[j]]);
            if (!can_i) can_i = std::get_if<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> *>(&CanArray[BUS_IDS[j]]);
            if (!can_i) can_i = std::get_if<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> *>(&CanArray[BUS_IDS[j]]);

            if(can_i && *can_i){
              (*can_i)->write(msg[j]);  // Send message
              // delayMicroseconds(10);
              (*can_i)->events();
            }

            // std::visit([&](auto&& can_i) {
                
            //     can_i->write(msg[j]);  // Send message
            //     delayMicroseconds(10);
            //     can_i->events();
            //     #ifdef DEBUG_COMMAND
            //         Serial.print("Sending CAN command from ");
                
            //         if constexpr (std::is_same_v<std::decay_t<decltype(can_i)>, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *>) {
            //               Serial.print("CAN0");
            //         } else if constexpr (std::is_same_v<std::decay_t<decltype(can_i)>, FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> *>) {
            //               Serial.print("CAN1");
            //         } else if constexpr (std::is_same_v<std::decay_t<decltype(can_i)>, FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> *>) {
            //               Serial.print("CAN2");
            //         } else {
            //               Serial.print("Unknown CAN Bus");
            //         }
            //         Serial.print(", Node: ");
            //         Serial.println(i);
            //     #endif
            //     waitForResponse(j, i); // Wait for response from node
            //     // delayMicroseconds(200);
            // }, CanArray[BUS_IDS[j]]);

        }
      }
      waitResponses(i);
      // delayMicroseconds(100);
  }
  #ifdef DEBUG_COMMAND
    Serial.println("======================================");
  #endif
  // interrupts();
  _iter++;
}

void sendUDPPacket() {
  uint8_t buffer[TOTAL_NODES * 8];
  uint8_t *buffer_ptr = buffer;
  // Serial.println("==============================================");
  for (int j = 0; j < NUM_BUSSES; j++) {
    for (int i = 0; i < NUM_NODES[j]; i++) {
      memcpy(buffer_ptr, can_data[j][i], 8);
      buffer_ptr += 8;
      // printCANResponse(can_data[j][i],j);
    }
  }
  // Serial.println("==============================================");

  if (!udp.send("192.168.0.110", kPort, buffer, TOTAL_NODES * 8)) {
    DEBUG_PRINT("ERROR.");
  }
}

void reset() {
  memset(can_command, 0, sizeof(can_command));
  memset(can_data, 0, sizeof(can_data));
  first_packet_recv = false;
}

void printCANResponse(const uint8_t *buf,int canbus_id) {
   
    int id = buf[0];
    int p_int = (buf[1] << 8) | buf[2];
    int v_int = (buf[3] << 4) | (buf[4] >> 4);
    int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert to float
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, T_MIN, T_MAX, 12);

    // Print the results
    printf("response from can bus %d node %d::::",canbus_id,id-1);
    printf("position: %.6f velocity: %.6f tau_ff: %.6f\n",p,v,t);

    // Serial.print("Position: "); Serial.println(p, 6);
    // Serial.print("Velocity: "); Serial.println(v, 6);
    // Serial.print("Torque: "); Serial.println(t, 6);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
#include "TeensyCANInterface.h"
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
// #define  LATENCY_TEST

#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do {} while (0)
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
constexpr uint16_t kPort = 8006;  // udp port
const char * HOST_IP = "192.168.0.110";
constexpr uint8_t TEENSY_IP[4] = { 192, 168, 0, 114 };
constexpr uint8_t TEENSY_DNS[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_GATEWAY[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_SUBNET[4] = { 255, 255, 255, 0 };

//======================================================================
//            CAN Settings
//======================================================================
constexpr uint8_t NUM_BUSSES = 2;
uint8_t NUM_NODES[NUM_BUSSES] = {2, 2};
uint8_t BUS_IDS[NUM_BUSSES]={0, 2};
int8_t can_bus_idx[3]={0,-1, 1};
long send_delay_us = 80;
//========================================================================

uint8_t TOTAL_NODES = std::accumulate(NUM_NODES, NUM_NODES + NUM_BUSSES, 0); 
volatile bool responseFlag[NUM_BUSSES][MAX_NODES] = {false};
volatile bool responseLastActFlag[NUM_BUSSES] = {false};

uint8_t can_data[NUM_BUSSES][MAX_NODES][8];     // CAN data buffer for each node
uint8_t can_command[NUM_BUSSES][MAX_NODES][8];  // CAN command buffer for each node

unsigned long last_packet_time_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned long total_latency_bus[NUM_BUSSES][MAX_NODES] = { 0 };
unsigned int packet_count_bus[NUM_BUSSES][MAX_NODES] = { 0 };
constexpr int MAX_NUM_SAMPLES = 5000;
const char* CAN_BUS_NAMES[] = { "CAN0", "CAN1", "CAN2" };

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

void canReceive_i(const CAN_message_t &msg, int bus_id) {
  int node_id = msg.buf[0] - 1;

  if(node_id>=0 && node_id <NUM_NODES[can_bus_idx[bus_id]]){
      memcpy(can_data[can_bus_idx[bus_id]][node_id], msg.buf, 8);
      responseFlag[can_bus_idx[bus_id]][node_id] = true;

      if (node_id == (NUM_NODES[can_bus_idx[bus_id]] - 1)) {
            responseLastActFlag[can_bus_idx[bus_id]] = true;
      }
    
      // DEBUG_PRINT("Received response from CAN bus %d, node %d\n",bus_id,node_id);
      // printf("Received response from CAN bus %d, node %d\n",bus_id,node_id);
      
  }else{
      // DEBUG_PRINT("Invalid node ID %d received on CAN bus %d\n",node_id,bus_id);
      // printf("Invalid node ID %d received on CAN bus %d\n",node_id,bus_id);
  }
  #ifdef LATENCY_TEST
    if (node_id >= 0 && node_id < NUM_NODES[can_bus_idx[bus_id]]) {
      unsigned long current_time = micros();
      unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[bus_id]][node_id];
      total_latency_bus[can_bus_idx[bus_id]][node_id] += latency;
      packet_count_bus[can_bus_idx[bus_id]][node_id]++;
  
      if (packet_count_bus[can_bus_idx[bus_id]][node_id] == MAX_NUM_SAMPLES) {
        unsigned long average_latency = total_latency_bus[can_bus_idx[bus_id]][node_id] / MAX_NUM_SAMPLES;
        printf("Average latency of %d CAN receives for node %d on bus %d: %lu us\n", MAX_NUM_SAMPLES, node_id + 1,BUS_IDS[bus_id],average_latency);
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
  DEBUG_PRINT("FEED THE DOG SOON, OR RESET!");
}

void setup() {

  #ifdef DEBUG_MODE
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
  // printf("=========================================\n");
  sendCANCMD();
  sendUDPPacket();
  // printf("-----------------------------------------\n");
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
      DEBUG_PRINT("Assigned CAN receive function for bus %d\n",BUS_IDS[i]);
      can_i->setClock(CLK_60MHz);

      DEBUG_PRINT("Done setting up CAN %d\n", BUS_IDS[i]);
    }
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
                            offset += 8;
                           
                        }
                        
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

void printCANCommand2(CAN_message_t msg) { 
  for (int j = 0; j < 8; j++) {
    printf("%02X ", msg.buf[j]);
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

void sendCANCMD() {
  
  static int _iter=0;
  CAN_message_t msg[NUM_BUSSES];
  for (int i = 0; i < MAX_NODES; i++) {  // Iterate over node IDs first
    for (int j = 0; j < NUM_BUSSES; j++) {  // Iterate over each CAN bus
        if (i < NUM_NODES[j]) {  // Ensure the node exists in this bus
            msg[j].id = i + 1;
            msg[j].len = 8;

            memcpy(msg[j].buf, can_command[j][i], 8);  // Copy command to message buffer

            TeensyCAN* can_i = CanArray[BUS_IDS[j]];

            if(can_i){
              can_i->write(msg[j]);  // Send message
              delayMicroseconds(send_delay_us);                
              can_i->events();
              // Serial.print("sending CAN command to ");
              // Serial.print(CAN_BUS_NAMES[BUS_IDS[j]]);
              // Serial.print(" node ");
              // Serial.println(i);
            }
        }
      }
  }
  unsigned long start_time = micros();
  while (micros() - start_time < RESPONSE_TIMEOUT) {
    if(allBusesCompleted()){break;}
        delayMicroseconds(10);  // Small delay to prevent CPU overload
  }

  resetResponseFlags();  // Reset flags for the next cycle
  _iter++;
}

bool allBusesCompleted() {
    for (int j = 0; j < NUM_BUSSES; j++) {
        if (!responseLastActFlag[j]) {  // Check if last actuator responded
            return false;  // Still waiting for responses
        }
    }
    return true;  // All last actuators have responded
}

void resetResponseFlags() {
    for (int j = 0; j < NUM_BUSSES; j++) {
        responseLastActFlag[j] = false;  // Clear last actuator response flag
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
  if (!udp.send(HOST_IP, kPort, buffer, TOTAL_NODES * 8)) {
    DEBUG_PRINT("ERROR.");
  }
}

void reset() {
  memset(can_command, 0, sizeof(can_command));
  memset(can_data, 0, sizeof(can_data));
  first_packet_recv = false;
}


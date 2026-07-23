#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <string>
#include <queue>
#include <array>
#include <lwip/ip_addr.h>

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define RESPONSE_TIMEOUT 500
#define MAX_NODES 3  // Maximum number of nodes per bus
using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;


String COMPUTER_IP_STRING = "192.168.0.222";
IPAddress COMPUTER_IP;
constexpr uint16_t kPort = 8004;  // udp port
constexpr int MAX_NUM_SAMPLES = 5000;


// constexpr int NUM_BUSSES = 3;
constexpr int NUM_BUSSES = 2;
// constexpr int NUM_NODES[3] = {3, 0, 0 };

// !!! MODIFY THIS WHEN ADDING MOTORS TO THE BUS
// int NUM_NODES[3] = {2, 0, 1};
// int NUM_NODES[NUM_BUSSES] = {2,2,2};
// int BUS_IDS[NUM_BUSSES]={0,1,2};
int NUM_NODES[NUM_BUSSES] = {1,1};
int BUS_IDS[NUM_BUSSES]={0,1};


int TOTAL_NODES = 2;
bool responseFlag[NUM_BUSSES][MAX_NODES] = {false};

EthernetUDP udp;
constexpr uint8_t TEENSY_IP[4] = { 192, 168, 0, 112 };
constexpr uint8_t TEENSY_DNS[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_GATEWAY[4] = { 192, 168, 0, 1 };
constexpr uint8_t TEENSY_SUBNET[4] = { 255, 255, 255, 0 };


uint8_t can_data[NUM_BUSSES][MAX_NODES][8];     // CAN data buffer for each node
uint8_t can_command[NUM_BUSSES][MAX_NODES][8];  // CAN command buffer for each node
uint8_t can_bus_idx[3]={0,1,-1};

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

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for Serial
  }
  printf("Starting...\r\n");

  // memset(&COMPUTER_IP,0,sizeof(COMPUTER_IP));
  // memcpy(COMPUTER_IP,COMPUT)
  // if(!ipaddr_aton(COMPUTER_IP_STRING.c_str(),&COMPUTER_IP)){
  //   printf("IP conversion failed!");
  // }

  setupEthernet();
  setupUDP();
  setupCAN();
}


void loop() {
  unsigned long start_time=micros();
  static int _iter=0;
  for (int i=0;i<NUM_BUSSES;i++){
    if(BUS_IDS[i]==0){
      Can0.events();
    }
    else if(BUS_IDS[i]==1){
      Can1.events();
    }
    else if(BUS_IDS[i]==2){
      Can2.events();
    }
  }
  Can0.events();
  Can1.events();
  Can2.events();
  receiveUDPPacket();
  // printCANCommand();
  if (!first_packet_recv){
    // Serial.println("i am stuck here");
    return;
  }
  // if(_iter<10){
  //   printCANCommand();
  // }
  sendCANCMD();
  // printCANData();
  sendUDPPacket();
  unsigned long end_time=micros();
  unsigned long elapsed_time=end_time-start_time;
  // printf("CAN bandwidth (us): %lu \n",elapsed_time);
  _iter++;
}

void printNetworkConfig() {
  Serial.print("- Local IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("- Subnet Mask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("- Gateway: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("- DNS: ");
  Serial.println(Ethernet.dnsServerIP());
}

// Ethernet setup
void setupEthernet() {
  uint8_t mac[6];
  Ethernet.macAddress(mac);
  Serial.print("MAC = ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();

  IPAddress ip(TEENSY_IP[0], TEENSY_IP[1], TEENSY_IP[2], TEENSY_IP[3]);
  IPAddress dns(TEENSY_DNS[0], TEENSY_DNS[1], TEENSY_DNS[2], TEENSY_DNS[3]);
  IPAddress gateway(TEENSY_GATEWAY[0], TEENSY_GATEWAY[1], TEENSY_GATEWAY[2], TEENSY_GATEWAY[3]);
  IPAddress subnet(TEENSY_SUBNET[0], TEENSY_SUBNET[1], TEENSY_SUBNET[2], TEENSY_SUBNET[3]);

  Ethernet.onLinkState([](bool state) {
    Serial.printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    if (ip != INADDR_NONE) {
      Serial.println("[Ethernet] Address changed:");
      printNetworkConfig();
    } else {
      Serial.println("[Ethernet] Address changed: No IP address");
    }
  });

  if (!Ethernet.begin(mac, ip, dns, gateway, subnet)) {
    Serial.println("Failed to start Ethernet");
  } else {
    Serial.println("Ethernet setup complete. Ready to respond to ping.");
  }
}

void setupUDP() {
  if (udp.begin(kPort)) {
    Serial.printf("UDP server started on port %d.\n", kPort);
  } else {
    Serial.println("Failed to start UDP server.");
  }
}


// void setupCAN()
// {
//     for (int i = 0; i < NUM_BUSSES; i++)
//     {
//        if(i==0){
//             Can0.begin();
//             Can0.setBaudRate(1000000);
//             Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
//             Can0.setMBFilter(MB1, 0, 0x0);
//             Can0.enhanceFilter(MB1);
//             Can0.distribute();
//             Can0.enableMBInterrupts();

//             Can0.onReceive(canReceive);
//             Can0.setClock(CLK_60MHz);
//             // Can0.mailboxStatus();
//             printf("Done setting up CAN 0\n");
//        }
//        else if (i==1){

//             Can1.begin();
//             Can1.setBaudRate(1000000);
//             Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
//             Can1.setMBFilter(MB1, 0, 0x0);
//             Can1.enhanceFilter(MB1);
//             Can1.enableMBInterrupts();
//             Can1.onReceive(canReceive2);
//             Can1.distribute();
//             Can1.setClock(CLK_60MHz);
//             // Can1.mailboxStatus();
//             printf("Done setting up CAN 1\n");
//        }
//        else if (i==2){
//             Can2.begin();printf("CAN Command bus %d:\n", j + 1);
//             Can2.setBaudRate(1000000);
//             Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
//             Can2.setMBFilter(MB1, 0, 0x0);
//             Can2.enhanceFilter(MB1);
//             Can2.enableMBInterrupts();
//             Can2.onReceive(canReceive3);
//             Can2.distribute();
//             Can2.setClock(CLK_60MHz);
//             // Can1.mailboxStatus();
//             printf("Done setting up CAN 2\n");
//        }

//     }

// }

void setupCAN() {

  for(int i=0;i<NUM_BUSSES;++i){
    if(BUS_IDS[i]==0){
      Can0.begin();
      Can0.setBaudRate(1000000);
      Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
      Can0.setMBFilter(MB1, 0, 0x0);
      Can0.enhanceFilter(MB1);
      Can0.distribute();
      Can0.enableMBInterrupts();
      Can0.onReceive(canReceive);
      Can0.setClock(CLK_60MHz);
      printf("Done setting up CAN 0\n");
    }
    else if(BUS_IDS[i]==1){
      Can1.begin();
      Can1.setBaudRate(1000000);
      Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
      Can1.setMBFilter(MB1, 0, 0x0);
      Can1.enhanceFilter(MB1);
      Can1.enableMBInterrupts();
      Can1.onReceive(canReceive2);
      Can1.setClock(CLK_60MHz);
      printf("Done setting up CAN 1\n");
    }
     if(BUS_IDS[i]==2){
      Can2.begin();
      Can2.setBaudRate(1000000);
      Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
      Can2.setMBFilter(MB1, 0, 0x0);
      Can2.enhanceFilter(MB1);
      Can2.enableMBInterrupts();
      Can2.onReceive(canReceive3);
      Can2.setClock(CLK_60MHz);
      printf("Done setting up CAN 2\n");
    }
  }
}


void printIPAddress() {
  IPAddress ip = Ethernet.localIP();
  printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.broadcastIP();
  printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

void canReceive(const CAN_message_t &msg) {
  int node_id = msg.buf[0] - 1;
  memcpy(can_data[can_bus_idx[0]][node_id], msg.buf, 8);
  if (node_id >= 0 && node_id < NUM_NODES[0]) {
    unsigned long current_time = micros();
    unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[0]][node_id];
    total_latency_bus[can_bus_idx[0]][node_id] += latency;
    packet_count_bus[can_bus_idx[0]][node_id]++;

    if (packet_count_bus[can_bus_idx[0]][node_id] == MAX_NUM_SAMPLES) {
      // unsigned long average_latency = total_latency_bus[0][node_id] / MAX_NUM_SAMPLES;
      // printf("Average latency of %d CAN receives for node %d on bus 1: %lu us\n", MAX_NUM_SAMPLES, node_id + 1, average_latency);
      total_latency_bus[can_bus_idx[0]][node_id] = 0;
      packet_count_bus[can_bus_idx[0]][node_id] = 0;
    }

    last_packet_time_bus[can_bus_idx[0]][node_id] = current_time;
  }
  responseFlag[can_bus_idx[0]][node_id] = true;
}

void canReceive2(const CAN_message_t &msg) {
    int node_id = msg.buf[0] - 1;
    memcpy(can_data[can_bus_idx[1]][node_id], msg.buf, 8);
    Serial.println("i am in can receive 2");
    if (node_id >= 0 && node_id < NUM_NODES[1]) {
      unsigned long current_time = micros();
      unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[1]][node_id];
      total_latency_bus[can_bus_idx[1]][node_id] += latency;
      packet_count_bus[can_bus_idx[1]][node_id]++;

      if (packet_count_bus[can_bus_idx[1]][node_id] == MAX_NUM_SAMPLES) {
        // unsigned long average_latency = total_latency_bus[1][node_id] / MAX_NUM_SAMPLES;
        // printf("Average latency of %d CAN receives for node %d on bus 2: %lu us\n", MAX_NUM_SAMPLES, node_id + 1, average_latency);
        total_latency_bus[can_bus_idx[1]][node_id] = 0;
        packet_count_bus[can_bus_idx[1]][node_id] = 0;
      }

      last_packet_time_bus[can_bus_idx[1]][node_id] = current_time;
    }
    responseFlag[can_bus_idx[1]][node_id] = true;
}


void canReceive3(const CAN_message_t &msg) {
    int node_id = msg.buf[0] - 1;
    memcpy(can_data[can_bus_idx[2]][node_id], msg.buf, 8);
    // Serial.println("i am in can receive 3");
    if (node_id >= 0 && node_id < MAX_NODES) {
      unsigned long current_time = micros();
      unsigned long latency = current_time - last_packet_time_bus[can_bus_idx[2]][node_id];
      total_latency_bus[can_bus_idx[2]][node_id] += latency;
      packet_count_bus[can_bus_idx[2]][node_id]++;

      if (packet_count_bus[can_bus_idx[2]][node_id] == MAX_NUM_SAMPLES) {
        // unsigned long average_latency = total_latency_bus[2][node_id] / MAX_NUM_SAMPLES;
        // printf("Average latency of %d CAN receives for node %d on bus 1: %lu us\n", MAX_NUM_SAMPLES, node_id + 1, average_latency);
        total_latency_bus[can_bus_idx[2]][node_id] = 0;
        packet_count_bus[can_bus_idx[2]][node_id] = 0;
      }

      last_packet_time_bus[can_bus_idx[2]][node_id] = current_time;
    }
    responseFlag[can_bus_idx[2]][node_id] = true;
}

void receiveUDPPacket()
{       

    // noInterrupts();
    int size = udp.parsePacket();
    // Serial.println(size);
    if (size > 0)
    {
        const uint8_t *data = udp.data();
        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
            printf("Reset command received. Buffers and variables reset.\n");
        }
        else if (size >= TOTAL_NODES * 8)
        {
            if (!first_packet_recv)
                printf("first packet recv\n");
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
                            std::array<uint8_t, 8> nodeCommand;
                            // Use memcpy for efficient data copying
                            memcpy(can_command[k][i], &data[offset], 8 * sizeof(data[0]));
                            // memcpy(nodeCommand.data(), &data[offset], 8 * sizeof(data[0]));
                            offset += 8;
                            // printCANCommand3(can_command[k][i]);
                           
                        }
                        
                    }
                }
                else
                {
                    // CRC check failed, discard the data
                    printf("CRC check failed\n");
                    printf("Received CRC: %02X\n", received_crc);
                    printf("Calculated CRC: %02X\n", calculated_crc);
                }
                // printCANCommand();
            }
            else
            {
                printf("Invalid packet size\n");
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
      for (int j = 0; j < 8; j++) {
        printf("%02X ", msg.buf[j]);
      }
      printf("\n");
    
  
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
      break;
    }
  }
}

// void sendCANCMD() {
//   // noInterrupts();
//   static int _iter=0;
//   CAN_message_t msg[NUM_BUSSES];
//   for (int j = 0; j < NUM_BUSSES; j++) {
//     for (int i = 0; i < NUM_NODES[j]; i++) {
//       msg[j].id = i + 1;
//       msg[j].len = 8;

//       memcpy(msg[j].buf, can_command[j][i], 8);

      
//       if (j == 0) {
//         Can0.write(msg[j]);
//         waitForResponse(j, i);
//         // printf("CAN Command bus %d:\n", j + 1);
//         // printf("Node %d: ", i + 1);
//         //  if(_iter<20){
//         //   printf("CAN Command bus %d:\n", j + 1);
//         //   printf("Node %d: ", i + 1);
//           // printCANCommand2(msg[j]);
//         //   printCANCommand3(can_command[j][i]);  
//         // }
//         Serial.println("i am in can0");
//       } else if (j == 1) {
       
//         Can1.write(msg[j]);
//         waitForResponse(j, i);
//         Serial.println("i am in can1");
       
//       } else if (j == 2) {
//         Can2.write(msg[j]);
//         waitForResponse(j, i);
//         printCANCommand2(msg[j]);
//       }
      
//     }
//   }
//   // interrupts();
//   _iter++;
// }

void sendCANCMD() {
  // noInterrupts();
  static int _iter=0;
  CAN_message_t msg[NUM_BUSSES];
  for (int j = 0; j < NUM_BUSSES; j++) {
    for (int i = 0; i < NUM_NODES[j]; i++) {
      msg[j].id = i + 1;
      msg[j].len = 8;

      memcpy(msg[j].buf, can_command[j][i], 8);
      if (BUS_IDS[j] == 0) {
        Can0.write(msg[j]);
        waitForResponse(j, i);
        // printCANCommand2(msg[j]);
        // printf("CAN Command bus %d:\n", j + 1);
        // printf("Node %d: ", i + 1);
        //  if(_iter<20){
        //   printf("CAN Command bus %d:\n", j + 1);
        //   printf("Node %d: ", i + 1);
        //   // printCANCommand2(msg[j]);
        //   printCANCommand3(can_command[j][i]);  
        // }
      } else if (BUS_IDS[j] == 1) {
       
        Can1.write(msg[j]);
        waitForResponse(j, i);
       
      } else if (BUS_IDS[j] == 2) {
        Can2.write(msg[j]);
        printCANCommand2(msg[j]);
        waitForResponse(j, i);
      }
      
    }
  }
  // interrupts();
  _iter++;
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

  if (!udp.send("192.168.0.110", kPort, buffer, TOTAL_NODES * 8)) {
    printf("ERROR.");
  }
}

void reset() {
  memset(can_command, 0, sizeof(can_command));
  memset(can_data, 0, sizeof(can_data));
  first_packet_recv = false;
}
#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#define DEBUG_MODE
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do {} while (0)
#endif
#define FIFO_VERSION

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define CAN_LED_PIN 15

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

const float WRAP_RANGE = 25.0f; // -12.5 to 12.5, total range is 25
const float WRAP_MIN = -12.5f;
const float WRAP_MAX = 12.5f;

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_2, TX_SIZE_2> Can0;
FlexCAN_T4<CAN2, RX_SIZE_2, TX_SIZE_2> Can1;
FlexCAN_T4<CAN3, RX_SIZE_2, TX_SIZE_2> Can2;

constexpr uint32_t kDHCPTimeout = 5000; // 15 seconds
constexpr uint16_t kPort = 8003;         // udp port
constexpr int MAX_NODES = 2;             // Maximum number of nodes
constexpr int MAX_NUM_SAMPLES = 5000;
constexpr int NUM_BUSES = 3;

IPAddress ip(192, 168, 0, 111);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);

EthernetUDP udp;
bool ethernet_setup_done = false;

uint8_t can_data[MAX_NODES][8]={{0}};    // CAN data buffer for each node
uint8_t can_command[MAX_NODES][8]={{0}}; // CAN command buffer for each node

uint8_t can_data_bus2[MAX_NODES][8]={{0}};    // CAN data buffer for each node
uint8_t can_command_bus2[MAX_NODES][8]={{0}}; // CAN command buffer for each node

uint8_t can_data_bus3[MAX_NODES][8]={{0}};    // CAN data buffer for each node
uint8_t can_command_bus3[MAX_NODES][8]={{0}}; // CAN command buffer for each node

unsigned long last_packet_time_bus1[MAX_NODES] = {0};
unsigned long total_latency_bus1[MAX_NODES] = {0};
unsigned int packet_count_bus1[MAX_NODES] = {0};
unsigned int max_latency_bus1[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus1[MAX_NODES] = {0};

unsigned long last_packet_time_bus2[MAX_NODES] = {0};
unsigned long total_latency_bus2[MAX_NODES] = {0};
unsigned int packet_count_bus2[MAX_NODES] = {0};
unsigned int max_latency_bus2[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus2[MAX_NODES] = {0};


unsigned long last_packet_time_bus3[MAX_NODES] = {0};
unsigned long total_latency_bus3[MAX_NODES] = {0};
unsigned int packet_count_bus3[MAX_NODES] = {0};
unsigned int max_latency_bus3[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus3[MAX_NODES] = {0};

const uint8_t exit_motor_mode_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t enter_motor_mode_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t zero_motor_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

CAN_message_t msg;
CAN_message_t msg2;
CAN_message_t msg3;

unsigned long last_udp_packet_time = 0;

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

// Calculate CRC-8 checksum
// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;
uint8_t calculate_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void setup()
{
#ifdef DEBUG_MODE
    Serial.begin(115200);
    while (!Serial && millis() < 4000) {
        // Wait for Serial
    }
#endif

  DEBUG_PRINT("Starting...\r\n");
  reset();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAN_LED_PIN, OUTPUT);
  setupEthernetStatic();
noInterrupts();
  setupCAN();
interrupts();

  memset(can_data, 0, sizeof(can_data));
  memset(can_command, 0, sizeof(can_command));

  memset(can_data_bus2, 0, sizeof(can_data_bus2));
  memset(can_command_bus2, 0, sizeof(can_command_bus2));

  memset(can_data_bus3, 0, sizeof(can_data_bus3));
  memset(can_command_bus3, 0, sizeof(can_command_bus3));
  DEBUG_PRINT("Setup done...\r\n");

}

void setupEthernet()
{
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    DEBUG_PRINT("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Ethernet.onLinkState([](bool state)
                         { DEBUG_PRINT("[Ethernet] Link %s\r\n", state ? "ON" : "OFF"); });

    DEBUG_PRINT("Starting Ethernet with DHCP...\r\n");
    if (!Ethernet.begin())
    {
        DEBUG_PRINT("Failed to start Ethernet\r\n");
        return;
    }
    if (!Ethernet.waitForLocalIP(kDHCPTimeout))
    {
        DEBUG_PRINT("Failed to get IP address from DHCP\r\n");
        return;
    }

    DEBUG_PRINT("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    udp.begin(kPort);
    DEBUG_PRINT("Done setting Ethernet\r\n");

}
void setupEthernetStatic()
{
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    DEBUG_PRINT("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Ethernet.onLinkState([](bool state)
                         {ethernet_setup_done = state; 
                         if (state)
                            //  digitalWrite(LED_BUILTIN, HIGH);
                            digitalWrite(CAN_LED_PIN, HIGH);

                          else
                            // digitalWrite(LED_BUILTIN, LOW);
                              digitalWrite(CAN_LED_PIN, LOW);

                         DEBUG_PRINT("[Ethernet] Link %s\r\n", state ? "ON" : "OFF"); });

    // Set the static IP address

    DEBUG_PRINT("Starting Ethernet with static IP...\r\n");
    if (!Ethernet.begin(ip, subnet, gateway))
    {
        DEBUG_PRINT("Failed to start Ethernet with static IP\r\n");
        return;
    }

    // if (!Ethernet.waitForLink(kDHCPTimeout))
    // {
    //     DEBUG_PRINT("Failed to get IP address from DHCP\r\n");
    //     return;
    // }

    DEBUG_PRINT("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    udp.begin(kPort);
    DEBUG_PRINT("Done setting Ethernet\r\n");
    
}
void setupCAN()
{
  
    Can0.begin();
    Can0.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.setFIFOFilter(REJECT_ALL);
    Can0.setFIFOFilter(0, 0x0, STD);
    Can0.onReceive(canReceive);
#else
    Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can0.setMBFilter(MB1, 0, 0x0);
    Can0.enhanceFilter(MB1);
    Can0.distribute();
    Can0.enableMBInterrupts();
    Can0.onReceive(canReceive);
    Can0.setClock(CLK_60MHz);
#endif
    Can0.mailboxStatus();
    DEBUG_PRINT("Done setting up CAN 0\n");

    delayMicroseconds(10);

    Can1.begin();
    Can1.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(canReceive2);
#else

    Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can1.setMBFilter(MB1, 0, 0x0);
    Can1.enhanceFilter(MB1);
    Can1.enableMBInterrupts();
    Can1.onReceive(canReceive2);
    Can1.distribute();
    Can1.setClock(CLK_60MHz);
    Can1.mailboxStatus();
#endif
    DEBUG_PRINT("Done setting up CAN 1\n");
    delayMicroseconds(10);

    if (NUM_BUSES < 3)
    {
        return;
    }

    Can2.begin();
    Can2.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can2.enableFIFO();
    Can2.enableFIFOInterrupt();
    Can2.onReceive(canReceive3);
#else
    Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can2.setMBFilter(MB1, 0, 0x0);
    Can2.enhanceFilter(MB1);
    Can2.enableMBInterrupts();
    Can2.onReceive(canReceive3);
    Can2.distribute();
    Can2.setClock(CLK_60MHz);
    Can2.mailboxStatus();
#endif
    DEBUG_PRINT("Done setting up CAN 2\n");
    delayMicroseconds(10);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void printIPAddress()
{
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

void canReceive(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
      int node_id = msg.buf[0] - 1;
      memcpy(can_data[node_id], msg.buf, sizeof(msg.buf));
#ifndef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus1[node_id];
            total_latency_bus1[node_id] += latency;
            sum_squares_latency_bus1[node_id] += latency * latency;
            if (latency > max_latency_bus1[node_id]) max_latency_bus1[node_id] = latency;
            packet_count_bus1[node_id]++;

            if (packet_count_bus1[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus1[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus1[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 1, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus1[node_id]);
                
                total_latency_bus1[node_id] = 0;
                sum_squares_latency_bus1[node_id] = 0;
                max_latency_bus1[node_id] = 0;
                packet_count_bus1[node_id] = 0;
            }

            last_packet_time_bus1[node_id] = current_time;
        }
#endif
    }
}

void canReceive2(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
      int node_id = msg.buf[0] - 1;
      memcpy(can_data_bus2[node_id], msg.buf, sizeof(msg.buf));
    


#ifndef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus2[node_id];
            total_latency_bus2[node_id] += latency;
            sum_squares_latency_bus2[node_id] += latency * latency;
            if (latency > max_latency_bus2[node_id]) max_latency_bus2[node_id] = latency;
            packet_count_bus2[node_id]++;

            if (packet_count_bus2[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus2[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus2[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 2, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus2[node_id]);
                
                total_latency_bus2[node_id] = 0;
                sum_squares_latency_bus2[node_id] = 0;
                max_latency_bus2[node_id] = 0;
                packet_count_bus2[node_id] = 0;
            }

            last_packet_time_bus2[node_id] = current_time;
        }
#endif
}
}
void canReceive3(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
      int node_id = msg.buf[0] - 1;
      memcpy(can_data_bus3[node_id], msg.buf, sizeof(msg.buf));
    

#ifndef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus3[node_id];
            total_latency_bus3[node_id] += latency;
            sum_squares_latency_bus3[node_id] += latency * latency;
            if (latency > max_latency_bus3[node_id]) max_latency_bus3[node_id] = latency;
            packet_count_bus3[node_id]++;

            if (packet_count_bus3[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus3[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus3[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 3, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus3[node_id]);
                
                total_latency_bus3[node_id] = 0;
                sum_squares_latency_bus3[node_id] = 0;
                max_latency_bus3[node_id] = 0;
                packet_count_bus3[node_id] = 0;
            }

            last_packet_time_bus3[node_id] = current_time;
        }
#endif
}}

void loop()
{
    Can0.events();
    Can1.events();
    if (NUM_BUSES == 3)
        Can2.events();

    receiveUDPPacket();

    if (!first_packet_recv)
        return;

    sendCANCMD();
    sendUDPPacket();

    // delayMicroseconds(100);
}

void receiveUDPPacket()
{
    // noInterrupts();
    int size = udp.parsePacket();
    if (size > 0)
    {
        const uint8_t *data = udp.data();
        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
            DEBUG_PRINT("Reset command received. Buffers and variables reset.\n");
        }
        else if (size >= NUM_BUSES * MAX_NODES * 8)
        {
            if (!first_packet_recv)
                DEBUG_PRINT("first packet recv\n");
            first_packet_recv = true;

            // Extract the payload data
            std::vector<uint8_t> payload(data, data + NUM_BUSES * MAX_NODES * 8);

            // Calculate CRC-8 for the payload
            uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());

            if (size == NUM_BUSES * MAX_NODES * 8 + 1)
            {
                const uint8_t received_crc = data[NUM_BUSES * MAX_NODES * 8];

                if (received_crc == calculated_crc)
                {
                    last_udp_packet_time = micros();
                    // CRC check passed, process the data
                    for (int i = 0; i < MAX_NODES; i++)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            can_command[i][j] = data[i * 8 + j];
                        }
                    }
                    for (int i = 0; i < MAX_NODES; i++)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            can_command_bus2[i][j] = data[MAX_NODES * 8 + i * 8 + j];
                        }
                    }
                    if (NUM_BUSES == 3)
                    {

                        for (int i = 0; i < MAX_NODES; i++)
                        {
                            for (int j = 0; j < 8; j++)
                            {
                                can_command_bus3[i][j] = data[MAX_NODES * 8 * 2 + i * 8 + j];
                            }
                        }
                    }
                    // printCANCommand();
                    // sendCANCMD();
                }
                else
                {
                    // CRC check failed, discard the data
                    DEBUG_PRINT("CRC check failed\n");
                    DEBUG_PRINT("Received CRC: %02X\n", received_crc);
                    DEBUG_PRINT("Calculated CRC: %02X\n", calculated_crc);
                }
            }
            else
            {
                DEBUG_PRINT("Invalid packet size\n");
            }
        }
    }

    // interrupts();
}
void printCANCommand()
{
    // DEBUG_PRINT("CAN Command bus 1:\n");
    // for (int i = 0; i < MAX_NODES; i++)
    // {
    //     DEBUG_PRINT("Node %d: ", i + 1);
    //     for (int j = 0; j < 8; j++)
    //     {
    //         DEBUG_PRINT("%02X ", can_command[i][j]);
    //     }
    //     DEBUG_PRINT("\n");
    // }
    // DEBUG_PRINT("CAN Command bus 2:\n");
    // for (int i = 0; i < MAX_NODES; i++)
    // {
    //     DEBUG_PRINT("Node %d: ", i + 1);
    //     for (int j = 0; j < 8; j++)
    //     {
    //         DEBUG_PRINT("%02X ", can_command_bus2[i][j]);
    //     }
    //     DEBUG_PRINT("\n");
    // }

    if (NUM_BUSES == 3)
    {
        DEBUG_PRINT("CAN Command bus 3:\n");
        for (int i = 0; i < MAX_NODES; i++)
        {
            DEBUG_PRINT("Node %d: ", i + 1);
            for (int j = 0; j < 8; j++)
            {
                DEBUG_PRINT("%02X ", can_command_bus3[i][j]);
            }
            DEBUG_PRINT("\n");
        }
    }
    
}
void sendCANCMD()
{

    // noInterrupts();
    // exit_motor_mode();

    for (int i = 0; i < MAX_NODES; i++)
    {

        msg.id = i + 1;
        msg.len = 8;

        msg2.id = i + 1;
        msg2.len = 8;

        msg3.id = i + 1;
        msg3.len = 8;
        msg3.flags.extended = 0;

        memcpy(msg.buf, can_command[i], 8);
        memcpy(msg2.buf, can_command_bus2[i], 8);
        memcpy(msg3.buf, can_command_bus3[i], 8);

        
        Can0.write(msg);
        Can1.write(msg2);
        if (NUM_BUSES == 3){
            
            Can2.write(msg3);
        }

        #ifndef DEBUG_MODE
          // if(!is_can_message_match(msg,exit_motor_mode_cmd) && !is_can_message_match(msg,enter_motor_mode_cmd) && !is_can_message_match(msg,zero_motor_cmd)){
          //   printf("==================================\n");
          //   if(msg.id==2){
          //     printf("CAN0 message: \n");
          //     printCANCommand2(msg);
          //   }
          if(is_can_message_match(msg,exit_motor_mode_cmd) || is_can_message_match(msg,enter_motor_mode_cmd) || is_can_message_match(msg,zero_motor_cmd)){
            printf("==================================\n");
            if(msg.id==2){
              printf("CAN0 message: \n");
              printCANCommand2(msg);
            }
            // printf("CAN1 message: \n");
            // print_can_message(msg2);
            // printf("CAN2 message: \n");
            // print_can_message(msg3);
          }
        #endif
        
        delayMicroseconds(100);
    }
    // interrupts();
}
void sendUDPPacket()
{
    uint8_t buffer[MAX_NODES * 8 * NUM_BUSES];
    uint8_t *buffer_ptr = buffer;
    // printf("---------can0 data--------------\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(buffer_ptr, can_data[i], 8);
        buffer_ptr += 8;
        if(can_data[i][0]=2){
        print_can_message2(can_data[i]);
        }
    }
    // printf("---------can1 data--------------\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(buffer_ptr, can_data_bus2[i], 8);
        buffer_ptr += 8;
        // print_can_message2(can_data_bus2[i]);
    }
    if (NUM_BUSES == 3)
    {
        // printf("---------can2 data--------------\n");
        for (int i = 0; i < MAX_NODES; i++)
        {
            memcpy(buffer_ptr, can_data_bus3[i], 8);
            buffer_ptr += 8;
            // print_can_message2(can_data_bus3[i]);
        }
    }

    if (!udp.send("192.168.0.110", kPort, buffer, MAX_NODES * 8 * NUM_BUSES))
    {
        DEBUG_PRINT("ERROR.");
    }
}

void exit_motor_mode()
{
    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(can_command[i], exit_motor_mode_cmd, 8);
        memcpy(can_command_bus2[i], exit_motor_mode_cmd, 8);
        memcpy(can_command_bus3[i], exit_motor_mode_cmd, 8);
    }
}

void reset()
{
    // memset(can_command, 0, sizeof(can_command));
    // memset(can_command_bus2, 0, sizeof(can_command_bus2));
    // memset(can_command_bus3, 0, sizeof(can_command_bus3));

    // memset(can_data, 0, sizeof(can_data));
    // memset(can_data_bus2, 0, sizeof(can_data_bus2));
    // memset(can_data_bus3, 0, sizeof(can_data_bus3));


    for (int i = 0; i < MAX_NODES; i++)
    {
        last_packet_time_bus1[i] = 0;
        total_latency_bus1[i] = 0;
        packet_count_bus1[i] = 0;

        last_packet_time_bus2[i] = 0;
        total_latency_bus2[i] = 0;
        packet_count_bus2[i] = 0;

        last_packet_time_bus3[i] = 0;
        total_latency_bus3[i] = 0;
        packet_count_bus3[i] = 0;

    }
    exit_motor_mode();
    first_packet_recv = false;
}

void printCANCommand2(CAN_message_t msg) {
      for (int j = 0; j < 8; j++) {
        printf("%02X ", msg.buf[j]);
      }
      printf("\n");
    
}

void unpack_cmd(const uint8_t *msg, float &p_des, float &v_des, float &kp, float &kd, float &t_ff) 
{
    // Extract integers from the packed message
    int p_int = (msg[0] << 8) | msg[1];
    int v_int = (msg[2] << 4) | (msg[3] >> 4);
    int kp_int = ((msg[3] & 0xF) << 8) | msg[4];
    int kd_int = (msg[5] << 4) | (msg[6] >> 4);
    int t_int = ((msg[6] & 0xF) << 8) | msg[7];

    // Convert back to floating-point values
    p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
    kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    t_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void print_can_message(const CAN_message_t &msg) {
    if (msg.len != 8) {
        Serial.println("Invalid CAN message length!");
        return;
    }

    float p_des, v_des, kp, kd, t_ff;
    // Unpack the message using the raw data array
    unpack_cmd(msg.buf, p_des, v_des, kp, kd, t_ff);

    // Print the decoded values
    Serial.println("Decoded CAN Message:");
    Serial.print("Position: "); Serial.println(p_des, 6);
    Serial.print("Velocity: "); Serial.println(v_des, 6);
    Serial.print("Kp: "); Serial.println(kp, 6);
    Serial.print("Kd: "); Serial.println(kd, 6);
    Serial.print("Torque: "); Serial.println(t_ff, 6);
    Serial.println("----------------------");
}

bool is_can_message_match(const CAN_message_t &msg, const uint8_t pattern[8]) {
    // Ensure the message has the correct length
    if (msg.len != 8) {
        return false;
    }

    // Compare each byte in the message with the pattern
    for (int i = 0; i < 8; i++) {
        if (msg.buf[i] != pattern[i]) {
            return false;  // Return false if any byte is different
        }
    }

    return true;  // Return true if all bytes match
}

void print_can_message2(const uint8_t *buf) {
   
    int id = buf[0];
    int p_int = (buf[1] << 8) | buf[2];
    int v_int = (buf[3] << 4) | (buf[4] >> 4);
    int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert to float
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, T_MIN, T_MAX, 12);

    
    // Print the results
    Serial.print("Position: "); Serial.println(p, 6);
    Serial.print("Velocity: "); Serial.println(v, 6);
    Serial.print("Torque: "); Serial.println(t, 6);
    Serial.println("----------------------");
}


float wrap_angle(float angle) {
    return fmodf(angle - WRAP_MIN, WRAP_RANGE) + WRAP_MIN;
}

#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <string>
#include <queue>
#include <array>
#include <lwip/ip_addr.h>

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define RESPONSE_TIMEOUT 500

#define CANBUS_NO 2
#define ACT_ID 1
#define TURN_ON 1

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for Serial
  }
  
  printf("Starting...\r\n");
  if(CANBUS_NO == 0){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can0.setMBFilter(MB1, 0, 0x0);
    Can0.enhanceFilter(MB1);
    Can0.distribute();
    Can0.enableMBInterrupts();
    Can0.setClock(CLK_60MHz);
    printf("Done setting up CAN 0\n");
  }

  if(CANBUS_NO == 1){
    Can1.begin();
    Can1.setBaudRate(1000000);
    Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can1.setMBFilter(MB1, 0, 0x0);
    Can1.enhanceFilter(MB1);
    Can1.enableMBInterrupts();
    Can1.setClock(CLK_60MHz);
    printf("Done setting up CAN 1\n");
  }

  if (CANBUS_NO==2){
    Can2.begin();
    Can2.setBaudRate(1000000);
    Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can2.setMBFilter(MB1, 0, 0x0);
    Can2.enhanceFilter(MB1);
    Can2.enableMBInterrupts();
    Can2.setClock(CLK_60MHz);
    printf("Done setting up CAN 2\n");
  }
   
}


void loop() {
  unsigned long start_time=micros();
  static int _iter=0;
  
  if(CANBUS_NO == 0){
    Can0.events();
  }
  if(CANBUS_NO == 1){
    Can1.events();
  }
  if(CANBUS_NO == 2){
    Can2.events();
  }
    
  CAN_message_t msg;
  if(ACT_ID == 1){
    msg.id=1; //change this to the can id
  }
  if(ACT_ID == 2){
    msg.id=2; //change this to the can id
  }
  msg.len=8;

if(TURN_ON){
  msg.buf[0]=0xFF;
  msg.buf[1]=0xFF;
  msg.buf[2]=0xFF;
  msg.buf[3]=0xFF;
  msg.buf[4]=0xFF;
  msg.buf[5]=0xFF;
  msg.buf[6]=0xFF;
  msg.buf[7]=0xFC;
}
else{
  msg.buf[0]=0xFF;
  msg.buf[1]=0xFF;
  msg.buf[2]=0xFF;
  msg.buf[3]=0xFF;
  msg.buf[4]=0xFF;
  msg.buf[5]=0xFF;
  msg.buf[6]=0xFF;
  msg.buf[7]=0xFD;
}
  if(CANBUS_NO == 0){
    Can0.write(msg); //for can channel 0

  }
  if(CANBUS_NO == 1){
    Can1.write(msg); //for can channel 1
  }
  if(CANBUS_NO == 2){
    Can2.write(msg); //for can channel 2
  }
  
}

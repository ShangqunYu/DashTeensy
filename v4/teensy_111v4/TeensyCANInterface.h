#ifndef TEENSY_CAN_INTERFACE_H
#define TEENSY_CAN_INTERFACE_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Base class for CAN buses
class TeensyCAN {
public:
    virtual void begin() = 0;
    virtual void setBaudRate(uint32_t baud) = 0;
    virtual void setMaxMB(uint8_t mb_count) = 0;
    virtual void setMB(const FLEXCAN_MAILBOX &mb, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide) = 0;
    virtual void setMBFilter(FLEXCAN_MAILBOX mb, uint32_t filter_index, uint32_t id) = 0;
    virtual void setMBFilter(FLEXCAN_MAILBOX mb, uint32_t id) = 0;
    virtual void setMBFilter(FLEXCAN_FLTEN input) = 0;
    virtual void enhanceFilter(FLEXCAN_MAILBOX mb) = 0;
    virtual void distribute() = 0;
    virtual void enableMBInterrupts() = 0;
    virtual void onReceive(_MB_ptr handler) = 0;
    virtual void onReceive(const FLEXCAN_MAILBOX &mb, _MB_ptr handler)=0;
    virtual void setClock(FLEXCAN_CLOCK clk) = 0;
    virtual void mailboxStatus() = 0;

    virtual void write(const CAN_message_t& msg) = 0;
    virtual void events() = 0;
    virtual ~TeensyCAN() {}
};

// Template wrapper for FlexCAN_T4
template <CAN_DEV_TABLE CAN_BUS, FLEXCAN_RXQUEUE_TABLE RX_SIZE, FLEXCAN_TXQUEUE_TABLE TX_SIZE>
class FlexCANWrapper : public TeensyCAN {
private:
    FlexCAN_T4<CAN_BUS, RX_SIZE, TX_SIZE>* canDevice;

public:
    explicit FlexCANWrapper(FlexCAN_T4<CAN_BUS, RX_SIZE, TX_SIZE>* device) : canDevice(device) {}

    // Implement all virtual functions
    void begin() override {
        canDevice->begin();
    }

    void setBaudRate(uint32_t baud) override {
        canDevice->setBaudRate(baud);
    }

    void setMaxMB(uint8_t mb_count) override {
        canDevice->setMaxMB(mb_count);
    }

    void setMB(const FLEXCAN_MAILBOX &mb, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide) override{
      canDevice->setMB(mb,mb_rx_tx,ide);
    }

    void setMBFilter(FLEXCAN_MAILBOX mb, uint32_t filter_index, uint32_t id) override {
        canDevice->setMBFilter(mb, filter_index, id);
    }

    void setMBFilter(FLEXCAN_MAILBOX mb, uint32_t id) override {
        canDevice->setMBFilter(mb, id);
    }

    void setMBFilter(FLEXCAN_FLTEN input) override{
        canDevice->setMBFilter(input);
    }

    void enhanceFilter(FLEXCAN_MAILBOX mb) override {
        canDevice->enhanceFilter(mb);
    }

    void distribute() override {
        canDevice->distribute();
    }

    void enableMBInterrupts() override {
        canDevice->enableMBInterrupts();
    }

    void onReceive(_MB_ptr handler) override {
        canDevice->onReceive(handler);
    }

    virtual void onReceive(const FLEXCAN_MAILBOX &mb, _MB_ptr handler) override{
        canDevice->onReceive(mb,handler);
    }
    void setClock(FLEXCAN_CLOCK clk) override {
        canDevice->setClock(clk);
    }

    void write(const CAN_message_t& msg) override {
        canDevice->write(msg);
    }

    void events() override {
        canDevice->events();
    }

    void mailboxStatus () override{
      canDevice->mailboxStatus();
    }
};

#endif  // TEENSY_CAN_INTERFACE_H

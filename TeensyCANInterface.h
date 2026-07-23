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
    virtual void setMBFilter(uint8_t mb, uint8_t filter_index, uint32_t id) = 0;
    virtual void enhanceFilter(uint8_t mb) = 0;
    virtual void distribute() = 0;
    virtual void enableMBInterrupts() = 0;
    virtual void onReceive(void (*callback)(const CAN_message_t &)) = 0;
    virtual void setClock(uint8_t clk) = 0;

    virtual void write(const CAN_message_t& msg) = 0;
    virtual void events() = 0;
    virtual ~TeensyCAN() {}
};

// Template wrapper for FlexCAN_T4
template <typename CAN_TYPE, size_t RX_SIZE, size_t TX_SIZE>
class FlexCANWrapper : public TeensyCAN {
private:
    FlexCAN_T4<CAN_TYPE, RX_SIZE, TX_SIZE>* canDevice;

public:
    explicit FlexCANWrapper(FlexCAN_T4<CAN_TYPE, RX_SIZE, TX_SIZE>* device) : canDevice(device) {}

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

    void setMBFilter(uint8_t mb, uint8_t filter_index, uint32_t id) override {
        canDevice->setMBFilter(mb, filter_index, id);
    }

    void enhanceFilter(uint8_t mb) override {
        canDevice->enhanceFilter(mb);
    }

    void distribute() override {
        canDevice->distribute();
    }

    void enableMBInterrupts() override {
        canDevice->enableMBInterrupts();
    }

    void onReceive(void (*callback)(const CAN_message_t &)) override {
        canDevice->onReceive(callback);
    }

    void setClock(uint8_t clk) override {
        canDevice->setClock(clk);
    }

    void write(const CAN_message_t& msg) override {
        canDevice->write(msg);
    }

    void events() override {
        canDevice->events();
    }
};

#endif  // TEENSY_CAN_INTERFACE_H

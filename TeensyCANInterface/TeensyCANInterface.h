#ifndef TEENSY_CAN_INTERFACE_H
#define TEENSY_CAN_INTERFACE_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Base class for CAN buses
class TeensyCAN {
public:
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

    void write(const CAN_message_t& msg) override {
        canDevice->write(msg);
    }

    void events() override {
        canDevice->events();
    }
};

#endif  // TEENSY_CAN_INTERFACE_H

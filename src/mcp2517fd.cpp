#include "Arduino.h"
#include <SPI.h>
#include "ACAN2517.h"
#include "mcp2517fd.h"
#include "freertos/queue.h"
#include "freertos/task.h"


// Constructor defining which pins to use for CS and INT
MCP2517FD::MCP2517FD(uint8_t CS_Pin, SPIClass & SPI, uint8_t INT_Pin) : 
    CAN_COMMON(32),
    driver((const uint8_t)CS_Pin, SPI, (const uint8_t)INT_Pin),
    settings(ACAN2517Settings::OSC_40MHz, 500000)
{
}

void MCP2517FD::sendCallback(CAN_FRAME *frame)
{
    if (cbGeneral != NULL)
        (*cbGeneral)(frame);
    else
        Serial.printf("no callback for mcp %s!\n", name);
}

// Required CAN_COMMON overrides
uint32_t MCP2517FD::init(uint32_t ul_baudrate)
{
    ;
    // settings.mRequestedMode = ACAN2517Settings::Normal20B;
    // init(settings);
}

uint32_t MCP2517FD::init(ACAN2517Settings &settings)
{
    // 
}

uint32_t MCP2517FD::set_baudrate(uint32_t ul_baudrate)
{
    Serial.println("ACAN2517 doesn't seem to support changing baud rate");
    return 1;
}

void MCP2517FD::enable()
{
    ;
}

void MCP2517FD::disable()
{
    ;
}


int MCP2517FD::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    Serial.println("_setFilterSpecific not implemented!");
}

int MCP2517FD::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    ;
}

void MCP2517FD::resetHardware()
{
    Serial.println("resetHardware not implemented!");
}

uint32_t MCP2517FD::beginAutoSpeed()
{
    Serial.println("beginAutoSpeed not implemented!");
}

void MCP2517FD::setListenOnlyMode(bool state)
{
    Serial.println("setListenOnlyMode not implemented!");
}

bool MCP2517FD::sendFrame(CAN_FRAME& txFrame)
{
    CANMessage msg;

    msg.id = txFrame.id;
    msg.ext = txFrame.extended;
    msg.rtr = txFrame.rtr;
    msg.len = txFrame.length;
    msg.data64 = txFrame.data.int64;

    return driver.tryToSend(msg);
}

bool MCP2517FD::rx_avail()
{
    return driver.available();
}

uint16_t MCP2517FD::available() //like rx_avail but returns the number of waiting frames
{
    return driver.available();
}

uint32_t MCP2517FD::get_rx_buff(CAN_FRAME &msg)
{
    Serial.printf("get_rx_buff not implemented\n");
    return 0; 
}
uint32_t MCP2517FD::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    Serial.println("get_rx_buff not implemented!");
    return driver.available();
}

uint32_t MCP2517FD::set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed)
{
    Serial.println("FD not implemented");
    return 0;
}

bool MCP2517FD::sendFrameFD(CAN_FRAME_FD& txFrame)
{
    Serial.println("FD not implemented");
    return false;
}
uint32_t MCP2517FD::initFD(uint32_t nominalRate, uint32_t dataRate)
{
    Serial.println("FD not implemented");
    return 0;
}


#include "Arduino.h"
#include <SPI.h>
#include "ACAN2517.h"
#include "mcp2517fd.h"


// Remove this once shown working
static const byte MCP2517_SCK  = 18 ; // SCK input of MCP2517
static const byte MCP2517_MOSI = 23 ; // SDI input of MCP2517
static const byte MCP2517_MISO = 19 ; // SDO output of MCP2517



// Constructor defining which pins to use for CS and INT
MCP2517FD::MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin, ACAN2517 *acan) : 
    CAN_COMMON(32)
{
    driver = acan;
}

// Required CAN_COMMON overrides
uint32_t MCP2517FD::init(uint32_t ul_baudrate)
{
    Serial.println("init not implemented!");
    return 0;
}

uint32_t MCP2517FD::set_baudrate(uint32_t ul_baudrate)
{
    init(ul_baudrate);
}

void MCP2517FD::enable()
{
    Serial.println("enable not implemented!");
}

void MCP2517FD::disable()
{
    Serial.println("disable not implemented!");
}


int MCP2517FD::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    Serial.println("_setFilterSpecific not implemented!");
}

int MCP2517FD::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    Serial.println("_setFilter not implemented!");
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

    return driver->tryToSend(msg);
}

bool MCP2517FD::rx_avail()
{
    return driver->available();
}

uint16_t MCP2517FD::available() //like rx_avail but returns the number of waiting frames
{
    return driver->available();
}

uint32_t MCP2517FD::get_rx_buff(CAN_FRAME &msg)
{
    Serial.println("get_rx_buff not implemented!");
    return driver->available();
}
uint32_t MCP2517FD::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    Serial.println("get_rx_buff not implemented!");
    return driver->available();
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
#ifndef MCP2517_h
#define MCP2517_h

#include "Arduino.h"
#include "mcp25xxfd.h"
#include "mcp2517fd_defines.h"
#include <can_common.h>
#include "ACAN2517.h"

//#define DEBUG_SETUP

#define NUM_FILTERS 32

class MCP2517FD : public CAN_COMMON
{
  public:
	// Constructor defining which pins to use for CS and INT
    MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin, ACAN2517 *acan);
	
	// Overloaded initialization function
	int Init(uint32_t nominalBaud, uint8_t freq);
	int Init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW);
	int InitFD(uint32_t nominalBaud, uint32_t dataBaud, uint8_t freq);

    //block of functions which must be overriden from CAN_COMMON to implement functionality for this hardware
	int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    int _setFilter(uint32_t id, uint32_t mask, bool extended);
	void resetHardware();
	uint32_t init(uint32_t ul_baudrate);
    uint32_t beginAutoSpeed();
    uint32_t set_baudrate(uint32_t ul_baudrate);
    void setListenOnlyMode(bool state);
	void enable();
	void disable();
	bool sendFrame(CAN_FRAME& txFrame);
	bool rx_avail();
	uint16_t available(); //like rx_avail but returns the number of waiting frames
	uint32_t get_rx_buff(CAN_FRAME &msg);
	//special FD functions required to reimplement to support FD mode
	uint32_t get_rx_buffFD(CAN_FRAME_FD &msg);
    uint32_t set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed);
    bool sendFrameFD(CAN_FRAME_FD& txFrame);
    uint32_t initFD(uint32_t nominalRate, uint32_t dataRate);
	
  private:
	ACAN2517 *driver;
	bool _init(uint32_t baud, uint8_t freq, uint8_t sjw, bool autoBaud);
	bool _initFD(uint32_t nominalSpeed, uint32_t dataSpeed, uint8_t freq, uint8_t sjw, bool autoBaud);
	void initSPI();
	void commonInit();
    void handleFrameDispatch(CAN_FRAME_FD &frame, int filterHit);
	void handleTXFifoISR(int fifo);
	void handleTXFifo(int fifo, CAN_FRAME_FD &newFrame);
    void initializeResources();
};

#endif

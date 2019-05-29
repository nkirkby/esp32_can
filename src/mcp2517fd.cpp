#include "Arduino.h"
#include <SPI.h>
#include "ACAN2517.h"
#include "mcp2517fd.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TX_QUEUE_SIZE 1000

// Constructor defining which pins to use for CS and INT
MCP2517FD::MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin, ACAN2517 *acan) : 
    CAN_COMMON(32),
    tx_queue(xQueueCreate(1000, sizeof(CANMessage*)))
{
    driver = acan;
    if (tx_queue == NULL) Serial.printf("WARNING: couldn't allocate tx queue for MCP2517FD with CS_Pin %d\n", CS_Pin);
}

void MCP2517FD::sendCallback(CAN_FRAME *frame)
{
    if (cbGeneral != NULL)
        (*cbGeneral)(frame);
    else
        Serial.printf("no callback for mcp %p!\n", this->driver);
}

#define MIN(A, B) (A > B ? B : A)
#define MAX(A, B) (A > B ? A : B)

void rx_task( void *pvParameters )
{
    MCP2517FD* mcp= (MCP2517FD*)pvParameters;
    static CANMessage msg;
    static CAN_FRAME frame;
    bool hasReceived;
    static int counter;
    static int msgs_retrieved_per_cycle[1000];
    static int has_incremented;
    static int max_idx;

    while (1)
    {
        while (mcp->driver->receive(msg))
        {
            if (!has_incremented)
            {
                counter++;
                msgs_retrieved_per_cycle[counter % 1000] = 0;
                has_incremented = 1;
                max_idx = MAX(max_idx, MIN(counter, 999));
            }
            frame.id = msg.id;
            frame.extended = msg.ext;
            frame.rtr = msg.rtr;
            frame.length = msg.len;
            frame.data.int64 = msg.data64;
            mcp->sendCallback(&frame);

            msgs_retrieved_per_cycle[counter % 1000]++;
        }
        has_incremented = 0;
        if (counter % 10000 == 0)
        {
            double avg = 0;
            for (int i = 0; i < max_idx ; i++)
            {
                avg += msgs_retrieved_per_cycle[i];
            }
            avg /= (max_idx + 1);
            Serial.printf("receive poll task with mcp %p is still running.  receiving avg of %f frames per 1ms cycle from driver. (avg of %d cycles)\n", mcp, avg, max_idx);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void tx_task(void *pvParameters)
{
    MCP2517FD* mcp= (MCP2517FD*)pvParameters;
    CANMessage *msg;
    bool sent;

    while (1)
    {
        if (mcp->tx_queue != NULL)
        {
            Serial.printf("tx queue spaces remaining: %d\n", uxQueueSpacesAvailable(mcp->tx_queue));
            xQueueReceive(mcp->tx_queue, &msg, portMAX_DELAY);
            while(false == ((sent = mcp->driver->tryToSend(*msg)))) { /* keep trying  */ } // Returns false if buffer is full
            free(msg);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}


int g_counter = 0;

#include <unistd.h>

// Required CAN_COMMON overrides
uint32_t MCP2517FD::init(uint32_t ul_baudrate)
{
    /*
    This is gross.  We've got to call driver->begin, but I've not yet figured out how to do that
    without using an ACAN2517 instance that's declared at global scope
    */
    char *name = (char*)malloc(sizeof(char) * 20);
    snprintf(name, 19, "MCP_TX_%d", g_counter);
    snprintf(name, 19, "MCP_RX_%d", g_counter++);
    xTaskCreatePinnedToCore(&rx_task, name, 2048, this, 10, NULL, 1);
    xTaskCreatePinnedToCore(&tx_task, name, 2048, this, 4, NULL, 1);
    return 0;
}

uint32_t MCP2517FD::set_baudrate(uint32_t ul_baudrate)
{
    Serial.println("ACAN2517 doesn't seem to support changing baud rate");
    return 1;
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
    CANMessage *msg = new CANMessage;
    static int counter;
    bool sent;

    msg->id = txFrame.id;
    msg->ext = txFrame.extended;
    msg->rtr = txFrame.rtr;
    msg->len = txFrame.length;
    msg->data64 = txFrame.data.int64;

    if (xQueueSend(tx_queue, &msg, 0) == errQUEUE_FULL)
    {
        Serial.printf("ERROR: CAN %p tx queue is full\n", this->driver);
        return -1;
    }
    return 0;
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


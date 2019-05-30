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
            // Serial.printf("receive poll task with mcp %p is still running.  receiving avg of %f frames per 1ms cycle from driver. (avg of %d cycles)\n", mcp, avg, max_idx);
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
            xQueueReceive(mcp->tx_queue, &msg, portMAX_DELAY);  // blocks until a message is ready
            if (false == ((mcp->driver->tryToSend(*msg))))
            {
                Serial.printf("Failed to send CAN message with id 0x%x\n", msg->id);
            } 
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


void poll_timer_task(void *pvParameters)
{
    MCP2517FD *mcp = (MCP2517FD *)pvParameters;
	while(mcp->driver->isr_core()) {}
}

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
    xTaskCreatePinnedToCore(&rx_task, name, 4096, this, 10, NULL, 1);
    xTaskCreatePinnedToCore(&tx_task, name, 2048, this, 4, NULL, 1);


	esp_timer_create_args_t poll_timer_task_args;
	poll_timer_task_args.callback = &poll_timer_task;
    poll_timer_task_args.arg = this;
	poll_timer_task_args.name = "poll timer task";

    esp_timer_handle_t poll_timer;
    ESP_ERROR_CHECK(esp_timer_create(&poll_timer_task_args, &poll_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(poll_timer, 1000));
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
    static int missed_messages;
    bool sent;

    msg->id = txFrame.id;
    msg->ext = txFrame.extended;
    msg->rtr = txFrame.rtr;
    msg->len = txFrame.length;
    msg->data64 = txFrame.data.int64;

    if (xQueueSend(tx_queue, &msg, 0) == errQUEUE_FULL)
    {
        missed_messages++;
        if (missed_messages % 1000 == 1)
            Serial.printf("ERROR: %s MCP2517FD tx queue is full.  Dropped %d frames since last successful send\n", this->name, missed_messages);
        free(msg);
        return -1;
    }
    if (missed_messages)
        Serial.printf("queue's empty now");
    missed_messages = 0;
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
    Serial.printf("get_rx_buff not implemented\n");
    return 0; 
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


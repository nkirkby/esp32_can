/*
  ESP32_CAN.cpp - Library for ESP32 built-in CAN module
  
  Author: Collin Kidder
  
  Created: 31/1/18
*/

#include "Arduino.h"
#include "esp32_can_builtin.h"

CAN_device_t CAN_cfg = {
    CAN_SPEED_500KBPS,
    GPIO_NUM_17,
    GPIO_NUM_16,
    NULL,
    NULL
};

QueueHandle_t callbackQueue;
extern QueueHandle_t lowLevelRXQueue;

extern volatile uint32_t needReset;

ESP32CAN::ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin) : CAN_COMMON(32)
{
    CAN_cfg.rx_pin_id = rxPin;
    CAN_cfg.tx_pin_id = txPin;
    cyclesSinceTraffic = 0;
}

void ESP32CAN::setCANPins(gpio_num_t rxPin, gpio_num_t txPin)
{
    CAN_cfg.rx_pin_id = rxPin;
    CAN_cfg.tx_pin_id = txPin;
}

void CAN_WatchDog_Builtin( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;

     for(;;)
     {
        vTaskDelay( xDelay );
        espCan->cyclesSinceTraffic++;
        if (needReset)
        {
            espCan->cyclesSinceTraffic = 0;
            needReset = 0;
            if (CAN_cfg.speed > 0 && CAN_cfg.speed <= 1000000ul && espCan->initializedResources == true) 
            {
                //Serial.println("XXXX");
                CAN_stop();
                CAN_init();
            }
        }
     }
}

void task_LowLevelRX(void *pvParameters)
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    CAN_frame_t rxFrame;
    
    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if(xQueueReceive(lowLevelRXQueue, &rxFrame, portMAX_DELAY)==pdTRUE)
        {
            espCan->processFrame(rxFrame);
        }
    }
}

/*
Issue callbacks to registered functions and objects
Used to keep this kind of thing out of the interrupt handler
The callback type and mailbox are passed in the fid member of the
CAN_FRAME struct. It isn't really used by anything.
Layout of the storage:
bit   31 -    If set indicates an object callback
bits  24-30 - Idx into listener table
bits  0-7   - Mailbox number that triggered callback
*/
void task_CAN( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    CAN_FRAME rxFrame;

    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if(xQueueReceive(callbackQueue, &rxFrame, portMAX_DELAY)==pdTRUE)
        {
            espCan->sendCallback(&rxFrame);
        }
    }
}

void ESP32CAN::sendCallback(CAN_FRAME *frame)
{
    //frame buffer
    CANListener *thisListener;
    int mb;
    int idx;

    (*cbGeneral)(frame);
}

ESP32CAN::ESP32CAN() : CAN_COMMON(NUM_FILTERS) 
{
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }
    initializedResources = false;
    cyclesSinceTraffic = 0;
}

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox < NUM_FILTERS)
    {
        filters[mailbox].id = id & mask;
        filters[mailbox].mask = mask;
        filters[mailbox].extended = extended;
        filters[mailbox].configured = true;
        return mailbox;
    }
    return -1;
}

int ESP32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        if (!filters[i].configured) 
        {
            _setFilterSpecific(i, id, mask, extended);
            return i;
        }
    }
    return -1;
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }

    if (!initializedResources) {
                                 //Queue size, item size
        CAN_cfg.rx_queue = xQueueCreate(RX_BUFFER_SIZE,sizeof(CAN_frame_t));
        CAN_cfg.tx_queue = xQueueCreate(TX_BUFFER_SIZE,sizeof(CAN_frame_t));
        callbackQueue = xQueueCreate(16, sizeof(CAN_FRAME));
        CAN_initRXQueue();
                  //func        desc    stack, params, priority, handle to task
        xTaskCreatePinnedToCore(&task_CAN, "CAN_RX", 4096, this, 15, NULL, 1);  // Bumped this stack size to handle callbacks
        xTaskCreatePinnedToCore(&task_LowLevelRX, "CAN_LORX", 2048, this, 19, NULL, 1);
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10, NULL, 1);
        initializedResources = true;
    }

    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    needReset = 0;
    CAN_init();
}

uint32_t ESP32CAN::beginAutoSpeed()
{

}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    CAN_stop();
    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    needReset = 0;
    CAN_init();
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    CAN_SetListenOnly(state);
}

void ESP32CAN::enable()
{
    CAN_stop();
    CAN_init();
    needReset = 0;
}

void ESP32CAN::disable()
{
    CAN_stop();
    needReset = 0;
}

//This function is too big to be running in interrupt context. Refactored so it doesn't.
bool ESP32CAN::processFrame(CAN_frame_t &frame)
{
    CANListener *thisListener;
    CAN_FRAME msg;

    cyclesSinceTraffic = 0; //reset counter to show that we are receiving traffic

    msg.id = frame.MsgID;
    msg.length = frame.FIR.B.DLC;
    msg.rtr = frame.FIR.B.RTR;
    msg.extended = frame.FIR.B.FF;
    for (int i = 0; i < 8; i++) msg.data.byte[i] = frame.data.u8[i];
    
    if (xQueueSend(CAN_cfg.rx_queue, &frame, 0) == pdPASS)
        return true;
    return false;
}

bool ESP32CAN::_sendFrame(CAN_FRAME& txFrame)
{
    static unsigned int unable_to_tx_counter;
    static unsigned int c;

    CAN_frame_t __TX_frame;


    __TX_frame.MsgID = txFrame.id;
    __TX_frame.FIR.B.DLC = txFrame.length;
    __TX_frame.FIR.B.RTR = (CAN_RTR_t)txFrame.rtr;
    __TX_frame.FIR.B.FF = (CAN_frame_format_t)txFrame.extended;
    for (int i = 0; i < 8; i++) __TX_frame.data.u8[i] = txFrame.data.byte[i];

    c++;
    if (CAN_TX_IsBusy()) //hardware already sending, queue for sending when possible
    {
        if ((xQueueSend(CAN_cfg.tx_queue,&__TX_frame,0)) != pdTRUE)
        {
            if (unable_to_tx_counter % 1000 == 0)
                Serial.printf("tx Buffer overflow on CAN0.  %d frames untransmitted.  Is it connected properly?\n", unable_to_tx_counter + 1);
            unable_to_tx_counter++;
            return 0;
        }
        else
        {
        //  if ( c % 100 == 0)  Serial.printf("qid: %x\n", txFrame.id);
            unable_to_tx_counter = 0;  // One successful enqueue resets this counter
            return 1;
        }
    }
    else //hardware is free, send immediately
    {
        // if ( c % 100 == 0) Serial.printf(">id: %x\n", txFrame.id);
        CAN_write_frame(&__TX_frame);
        return 1;
    }
}

bool ESP32CAN::rx_avail()
{
    if (!CAN_cfg.rx_queue) return false;
    return uxQueueMessagesWaiting(CAN_cfg.rx_queue) > 0?true:false;
}

uint16_t ESP32CAN::available()
{
    if (!CAN_cfg.rx_queue) return 0;
    return uxQueueMessagesWaiting(CAN_cfg.rx_queue);
}

uint32_t ESP32CAN::_get_rx_buff(CAN_FRAME &msg)
{
    CAN_frame_t __RX_frame;
    //receive next CAN frame from queue
    if(xQueueReceive(CAN_cfg.rx_queue,&__RX_frame, 0)==pdTRUE)
    {
        msg.id = __RX_frame.MsgID;
        msg.length = __RX_frame.FIR.B.DLC;
        msg.rtr = __RX_frame.FIR.B.RTR;
        msg.extended = __RX_frame.FIR.B.FF;
        for (int i = 0; i < 8; i++) msg.data.byte[i] = __RX_frame.data.u8[i];
        return true;
    }
    return false;
}

uint32_t ESP32CAN::set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed) {}
uint32_t ESP32CAN::initFD(uint32_t nominalRate, uint32_t dataRate) {}

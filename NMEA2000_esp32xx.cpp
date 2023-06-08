/*
NMEA2000_esp32.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for ESP32 modules. See also NMEA2000 library.

Thanks to Thomas Barth, barth-dev.de, who has written ESP32 CAN code. To avoid extra
libraries, I implemented his code directly to the NMEA2000_esp32 to avoid extra
can.h library, which may cause even naming problem.
*/

#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "NMEA2000_esp32xx.h"

#if !defined(round)
#include <math.h>
#endif
static const char *TAG = "TWAI_C3";
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9

bool tNMEA2000_esp32xx::CanInUse=false;
tNMEA2000_esp32xx *pNMEA2000_esp32c3=0;


/* --------------------------- Tasks and Functions -------------------------- */

void tNMEA2000_esp32xx::twai_transmit_task(void *arg)
{
  tNMEA2000_esp32xx *l_pThis = (tNMEA2000_esp32xx *) arg;   

  while (1) {
    tCANFrame frame;
    xQueueReceive(l_pThis->TxQueue,&frame,0);
    l_pThis->CAN_send_frame(frame);
  }
  vTaskDelete(NULL);
}

void tNMEA2000_esp32xx::twai_receive_task(void *arg)
{
  tNMEA2000_esp32xx *l_pThis = (tNMEA2000_esp32xx *) arg;   
  while (1) {
    tCANFrame frame;
  
    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);
    frame.len=rx_msg.data_length_code;
    frame.id=rx_msg.identifier;
    for( size_t i=0; i<frame.len; i++ ) {
      frame.buf[i]=rx_msg.data[i];
    }
    //send frame to input queue
    xQueueSendToBack(l_pThis->RxQueue,&frame,0);
  }
  vTaskDelete(NULL);
}

//*****************************************************************************
tNMEA2000_esp32xx::tNMEA2000_esp32xx(gpio_num_t _TxPin,  gpio_num_t _RxPin) :
  tNMEA2000(), IsOpen(false),
                                                                      speed(CAN_SPEED_250KBPS), TxPin(_TxPin), RxPin(_RxPin),
  RxQueue(NULL), TxQueue(NULL) {
  printf("hello world\n\r");
}

//*****************************************************************************
bool tNMEA2000_esp32xx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/) {
  if ( uxQueueSpacesAvailable(TxQueue)==0 ) return false; // can not send to queue

  tCANFrame frame;
  frame.id=id;
  frame.len=len>8?8:len;
  memcpy(frame.buf,buf,len);
  xQueueSendToBack(TxQueue,&frame,0);  // Add frame to queue
  return true;
}

//*****************************************************************************
void tNMEA2000_esp32xx::InitCANFrameBuffers() {
  if (MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=50; // ESP32 has plenty of RAM
  if (MaxCANSendFrames<10 ) MaxCANSendFrames=40;
  uint16_t CANGlobalBufSize=MaxCANSendFrames-4;
  MaxCANSendFrames=4;  // we do not need much libary internal buffer since driver has them.
  RxQueue=xQueueCreate(MaxCANReceiveFrames,sizeof(tCANFrame));
  TxQueue=xQueueCreate(CANGlobalBufSize,sizeof(tCANFrame));
  
  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_esp32xx::CANOpen() {
    if (IsOpen) return true;

    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

    pNMEA2000_esp32c3=this;
    IsOpen=true;
    CAN_init();

    CanInUse=IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32xx::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
  tCANFrame frame;

    //receive next CAN frame from queue
    if ( xQueueReceive(RxQueue,&frame, 0)==pdTRUE ) {
      HasFrame=true;
      id=frame.id;
      len=frame.len;
      memcpy(buf,frame.buf,frame.len);
  }

  return HasFrame;
}

//*****************************************************************************
void tNMEA2000_esp32xx::CAN_init() {
  twai_timing_config_t t_config;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TxPin, RxPin, TWAI_MODE_NORMAL);
  
  switch (speed) {
  case CAN_SPEED_1000KBPS:
    t_config = TWAI_TIMING_CONFIG_1MBITS();
    break;

  case CAN_SPEED_800KBPS:
    t_config = TWAI_TIMING_CONFIG_800KBITS();
    break;
  default:
    t_config = TWAI_TIMING_CONFIG_250KBITS();
    break;
  }
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  xTaskCreatePinnedToCore(this->twai_receive_task, "TWAI_rx", 4096, this, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(this->twai_transmit_task, "TWAI_tx", 4096, this, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
  ESP_LOGE(TAG, "Driver installed");
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGE(TAG, "Driver started");

}
#if 0
//*****************************************************************************
void tNMEA2000_esp32xx::CAN_read_frame() {
  tCANFrame frame;
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);
  if(status_info.msgs_to_rx > 0) {
    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);
    frame.len=rx_msg.data_length_code;
    frame.id=rx_msg.identifier;
    for( size_t i=0; i<frame.len; i++ ) {
      frame.buf[i]=rx_msg.data[i];
    }
    //send frame to input queue
    xQueueSendToBackFromISR(RxQueue,&frame,0);
  }
}
#endif
//*****************************************************************************
void tNMEA2000_esp32xx::CAN_send_frame(tCANFrame &frame) {

  twai_message_t tx_msg;
  tx_msg.extd=1;  /**< Extended Frame Format (29bit ID) */
  tx_msg.data_length_code=frame.len>8?8:frame.len;  /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */

  tx_msg.dlc_non_comp=frame.len>8?1:0;  /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
  tx_msg.ss=1; /**< Transmit as a Single Shot Transmission. Unused for received. */
  tx_msg.identifier=frame.id;
  twai_transmit((const twai_message_t*)&tx_msg, portMAX_DELAY);
  
  // Copy the frame data to the hardware
  for ( size_t i=0; i<frame.len; i++) {
    tx_msg.data[i]=frame.buf[i];
  }
  twai_transmit(&tx_msg, portMAX_DELAY);
}

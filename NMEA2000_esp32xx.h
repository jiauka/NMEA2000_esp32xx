/*
NMEA2000_esp32xx.h

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi
Copyright (c) 2023 Jaume Clarens "jiauka"
* 
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

The library sets as default CAN Tx pin to GPIO 16 and CAN Rx pint to GPIO 4. If you
want to use other pins (I have not tested can any pins be used), add defines e.g.
#define ESP32_CAN_TX_PIN GPIO_NUM_34
#define ESP32_CAN_RX_PIN GPIO_NUM_35
before including NMEA2000_esp32xx.h
*/

#ifndef _NMEA2000_ESP32_H_
#define _NMEA2000_ESP32_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "NMEA2000.h"
#include "N2kMsg.h"

//#define NMEA2000_ESP32XX_USE_TASKS // Uncomment to use RX and TX tasks, needs around 10K  more memory

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

typedef enum  {
	CAN_SPEED_100KBPS=100, 				/**< \brief CAN Node runs at 100kBit/s. */
	CAN_SPEED_125KBPS=125, 				/**< \brief CAN Node runs at 125kBit/s. */
	CAN_SPEED_250KBPS=250, 				/**< \brief CAN Node runs at 250kBit/s. */
	CAN_SPEED_500KBPS=500, 				/**< \brief CAN Node runs at 500kBit/s. */
	CAN_SPEED_800KBPS=800, 				/**< \brief CAN Node runs at 800kBit/s. */
	CAN_SPEED_1000KBPS=1000				/**< \brief CAN Node runs at 1000kBit/s. */
}CAN_speed_t;

class tNMEA2000_esp32xx : public tNMEA2000
{
private:
  bool IsOpen;
  static bool CanInUse;
public:

  struct tCANFrame {
    uint32_t id; // can identifier
    uint8_t len; // length of data
    uint8_t buf[8];
  };



protected:
	CAN_speed_t    speed;	
  gpio_num_t     TxPin;	
  gpio_num_t     RxPin;
#ifdef NMEA2000_ESP32XX_USE_TASKS
  QueueHandle_t  RxQueue;
  QueueHandle_t  TxQueue;
#endif
protected:
  void CAN_send_frame(tCANFrame &frame); // Send frame
  void CAN_init();

protected:
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent=true);
  bool CANOpen();
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
  virtual void InitCANFrameBuffers();
#ifdef NMEA2000_ESP32XX_USE_TASKS
  static void twai_receive_task(void *arg);
  static void twai_transmit_task(void *arg);
#endif

public:
  tNMEA2000_esp32xx(gpio_num_t _TxPin=ESP32_CAN_TX_PIN,  gpio_num_t _RxPin=ESP32_CAN_RX_PIN);

  void InterruptHandler();
};

#endif

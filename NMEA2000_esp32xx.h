#ifndef _NMEA2KTWAI_H
#define _NMEA2KTWAI_H
#include "NMEA2000.h"

class tNMEA2000_esp32xx : public tNMEA2000
{
public:
    tNMEA2000_esp32xx(int _TxPin, int _RxPin, unsigned long recP = 0, unsigned long logPeriod = 0);
    typedef enum
    {
        ST_STOPPED,
        ST_RUNNING,
        ST_BUS_OFF,
        ST_RECOVERING,
        ST_OFFLINE,
        ST_DISABLED,
        ST_ERROR
    } STATE;
    typedef struct
    {
        // see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#_CPPv418twai_status_info_t
        uint32_t rx_errors = 0;
        uint32_t tx_errors = 0;
        uint32_t tx_failed = 0;
        uint32_t rx_missed = 0;
        uint32_t rx_overrun = 0;
        uint32_t tx_timeouts = 0;
        STATE state = ST_ERROR;
    } Status;
    Status getStatus();
    unsigned long getLastRecoveryStart() { return lastRecoveryStart; }
    void loop();
    static const char *stateStr(const STATE &st);
    virtual bool CANOpen();
    virtual ~tNMEA2000_esp32xx(){};
    static const int LOG_ERR = 0;
    static const int LOG_INFO = 1;
    static const int LOG_DEBUG = 2;
    static const int LOG_MSG = 3;

protected:
    // Virtual functions for different interfaces. Currently there are own classes
    // for Arduino due internal CAN (NMEA2000_due), external MCP2515 SPI CAN bus controller (NMEA2000_mcp),
    // Teensy FlexCAN (NMEA2000_Teensy), NMEA2000_avr for AVR, NMEA2000_mbed for MBED and NMEA2000_socketCAN for e.g. RPi.
    virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
    virtual bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
    // This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
    // and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
    virtual void InitCANFrameBuffers();
    virtual void logDebug(int level, const char *fmt, ...) {}

private:
    void initDriver();
    bool startRecovery();
    bool checkRecovery();
    Status logStatus();
    int RxPin;
    int TxPin;
    uint32_t txTimeouts = 0;
    // GwIntervalRunner timers;
    tN2kSyncScheduler recTimer;
    tN2kSyncScheduler logTimer;
    bool disabled = false;
    unsigned long lastRecoveryStart = 0;
};

#endif
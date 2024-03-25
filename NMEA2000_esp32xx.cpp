/*
NMEA2000_esp32xx.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi
Copyright (c) 2023 Jaume Clarens "jiauka"
2024 - Improved with error handling by wellenvogel - see https://github.com/wellenvogel/esp32-nmea2000/issues/67

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
*/
#include "NMEA2000_esp32xx.h"
#include "driver/twai.h"

#define LOGID(id) ((id >> 8) & 0x1ffff)
static const int TIMEOUT_OFFLINE = 256; // # of timeouts to consider offline

tNMEA2000_esp32xx::tNMEA2000_esp32xx(int _TxPin, int _RxPin, unsigned long recP, unsigned long logP) : tNMEA2000(), RxPin(_RxPin), TxPin(_TxPin)
{
    if (RxPin < 0 || TxPin < 0)
    {
        disabled = true;
    }
    else
    {
        // timers.addAction(logP,[this](){logStatus();});
        // timers.addAction(recP,[this](){checkRecovery();});
        recTimer = tN2kSyncScheduler(false, recP, 0);
        logTimer = tN2kSyncScheduler(false, logP, 0);
    }
}

bool tNMEA2000_esp32xx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
    if (disabled)
        return true;
    twai_message_t message;
    memset(&message, 0, sizeof(message));
    message.identifier = id;
    message.extd = 1;
    message.data_length_code = len;
    memcpy(message.data, buf, len);
    esp_err_t rt = twai_transmit(&message, 0);
    if (rt != ESP_OK)
    {
        if (rt == ESP_ERR_TIMEOUT)
        {
            if (txTimeouts < TIMEOUT_OFFLINE)
                txTimeouts++;
        }
        logDebug(LOG_MSG, "twai transmit for %ld failed: %x", LOGID(id), (int)rt);
        return false;
    }
    txTimeouts = 0;
    logDebug(LOG_MSG, "twai transmit id %ld, len %d", LOGID(id), (int)len);
    return true;
}
bool tNMEA2000_esp32xx::CANOpen()
{
    if (disabled)
    {
        logDebug(LOG_INFO, "CAN disabled");
        return true;
    }
    esp_err_t rt = twai_start();
    if (rt != ESP_OK)
    {
        logDebug(LOG_ERR, "CANOpen failed: %x", (int)rt);
        return false;
    }
    else
    {
        logDebug(LOG_INFO, "CANOpen ok");
    }
    // Start timers now.
    recTimer.UpdateNextTime();
    logTimer.UpdateNextTime();
    return true;
}
bool tNMEA2000_esp32xx::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
    if (disabled)
        return false;
    twai_message_t message;
    esp_err_t rt = twai_receive(&message, 0);
    if (rt != ESP_OK)
    {
        return false;
    }
    if (!message.extd)
    {
        return false;
    }
    id = message.identifier;
    len = message.data_length_code;
    if (len > 8)
    {
        logDebug(LOG_DEBUG, "twai: received invalid message %lld, len %d", LOGID(id), len);
        len = 8;
    }
    logDebug(LOG_MSG, "twai rcv id=%ld,len=%d, ext=%d", LOGID(message.identifier), message.data_length_code, message.extd);
    if (!message.rtr)
    {
        memcpy(buf, message.data, message.data_length_code);
    }
    return true;
}
void tNMEA2000_esp32xx::initDriver()
{
    if (disabled)
        return;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TxPin, (gpio_num_t)RxPin, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 20;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t rt = twai_driver_install(&g_config, &t_config, &f_config);
    if (rt == ESP_OK)
    {
        logDebug(LOG_INFO, "twai driver initialzed, rx=%d,tx=%d", (int)RxPin, (int)TxPin);
    }
    else
    {
        logDebug(LOG_ERR, "twai driver init failed: %x", (int)rt);
    }
}
// This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
// and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
void tNMEA2000_esp32xx::InitCANFrameBuffers()
{
    if (disabled)
    {
        logDebug(LOG_INFO, "twai init - disabled");
    }
    else
    {
        initDriver();
    }
    tNMEA2000::InitCANFrameBuffers();
}
tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::getStatus()
{
    twai_status_info_t state;
    Status rt;
    if (disabled)
    {
        rt.state = ST_DISABLED;
        return rt;
    }
    if (twai_get_status_info(&state) != ESP_OK)
    {
        return rt;
    }
    switch (state.state)
    {
    case TWAI_STATE_STOPPED:
        rt.state = ST_STOPPED;
        break;
    case TWAI_STATE_RUNNING:
        rt.state = ST_RUNNING;
        break;
    case TWAI_STATE_BUS_OFF:
        rt.state = ST_BUS_OFF;
        break;
    case TWAI_STATE_RECOVERING:
        rt.state = ST_RECOVERING;
        break;
    }
    rt.rx_errors = state.rx_error_counter;
    rt.tx_errors = state.tx_error_counter;
    rt.tx_failed = state.tx_failed_count;
    rt.rx_missed = state.rx_missed_count;
    rt.rx_overrun = state.rx_overrun_count;
    rt.tx_timeouts = txTimeouts;
    if (rt.tx_timeouts >= TIMEOUT_OFFLINE && rt.state == ST_RUNNING)
    {
        rt.state = ST_OFFLINE;
    }
    return rt;
}
bool tNMEA2000_esp32xx::checkRecovery()
{
    if (disabled)
        return false;
    Status canState = getStatus();
    bool strt = false;
    if (canState.state != tNMEA2000_esp32xx::ST_RUNNING)
    {
        if (canState.state == tNMEA2000_esp32xx::ST_BUS_OFF)
        {
            strt = true;
            bool rt = startRecovery();
            logDebug(LOG_INFO, "twai BUS_OFF: start can recovery - result %d", (int)rt);
        }
        if (canState.state == tNMEA2000_esp32xx::ST_STOPPED)
        {
            bool rt = CANOpen();
            logDebug(LOG_INFO, "twai STOPPED: restart can driver - result %d", (int)rt);
        }
    }
    return strt;
}

void tNMEA2000_esp32xx::loop()
{
    if (disabled)
        return;

    // timers.loop();
    if (recTimer.IsTime())
    {
        recTimer.UpdateNextTime();
        checkRecovery();
    }
    if (logTimer.IsTime())
    {
        logTimer.UpdateNextTime();
        logStatus();
    }
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::logStatus()
{
    Status canState = getStatus();
    logDebug(LOG_INFO, "twai state %s, rxerr %d, txerr %d, txfail %d, txtimeout %d, rxmiss %d, rxoverrun %d",
             stateStr(canState.state),
             canState.rx_errors,
             canState.tx_errors,
             canState.tx_failed,
             canState.tx_timeouts,
             canState.rx_missed,
             canState.rx_overrun);
    return canState;
}

bool tNMEA2000_esp32xx::startRecovery()
{
    if (disabled)
        return false;
    lastRecoveryStart = millis();
    esp_err_t rt = twai_driver_uninstall();
    if (rt != ESP_OK)
    {
        logDebug(LOG_ERR, "twai: deinit for recovery failed with %x", (int)rt);
    }
    initDriver();
    bool frt = CANOpen();
    return frt;
}
const char *tNMEA2000_esp32xx::stateStr(const tNMEA2000_esp32xx::STATE &st)
{
    switch (st)
    {
    case ST_BUS_OFF:
        return "BUS_OFF";
    case ST_RECOVERING:
        return "RECOVERING";
    case ST_RUNNING:
        return "RUNNING";
    case ST_STOPPED:
        return "STOPPED";
    case ST_OFFLINE:
        return "OFFLINE";
    case ST_DISABLED:
        return "DISABLED";
    default:
        break;
    }
    return "ERROR";
}
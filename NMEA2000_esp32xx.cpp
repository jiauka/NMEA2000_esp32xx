/*
NMEA2000_esp32xx.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi
Copyright (c) 2023 Jaume Clarens "jiauka"
2024 - Improved with error handling by wellenvogel - see https://github.com/wellenvogel/esp32-nmea2000/issues/67
2024/10/18 - Improved with proper CAN bus recovery process. see https://github.com/phatpaul/NMEA2000_esp32xx


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

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG /* Enable this to show debug logging for this file only. */
#include "esp_log.h"

#define logDebug(level, fmt, args...)                     \
    do                                                    \
    {                                                     \
        int esp_level = ESP_LOG_VERBOSE;                  \
        if (level == LOG_ERR)                             \
        {                                                 \
            esp_level = ESP_LOG_ERROR;                    \
        }                                                 \
        else if (level == LOG_INFO)                       \
        {                                                 \
            esp_level = ESP_LOG_INFO;                     \
        }                                                 \
        else if (level == LOG_DEBUG)                      \
        {                                                 \
            esp_level = ESP_LOG_DEBUG;                    \
        }                                                 \
        ESP_LOG_LEVEL_LOCAL(esp_level, TAG, fmt, ##args); \
    } while (0)

const char *TAG = "N2K-drv"; // just a tag for logging

static const int TIMEOUT_OFFLINE = 256; // # of consecutive timeouts to consider OFFLINE

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
    case ST_RESTARTING:
        return "RESTARTING";
    case ST_DISABLED:
        return "DISABLED";
    default:
        break;
    }
    return "ERROR";
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::getStatus()
{
    Status rt;
    twai_status_info_t twai_status;
    if (ESP_OK != twai_get_status_info(&twai_status))
    {
        rt.state = ST_ERROR;
        return rt;
    }
    rt.rx_errors = twai_status.rx_error_counter;
    rt.tx_errors = twai_status.tx_error_counter;
    rt.tx_failed = twai_status.tx_failed_count;
    rt.rx_missed = twai_status.rx_missed_count;
    rt.rx_overrun = twai_status.rx_overrun_count;
    rt.tx_timeouts = txTimeouts;
    rt.state = state;
    return rt;
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

/**
 * @brief Construct a new tNMEA2000 esp32xx::tNMEA2000 esp32xx object
 *
 * @param _TxPin CAN bus Tx pin #
 * @param _RxPin CAN bus Rx pin #
 * @param recoveryPeriod Interval in ms to wait before CAN bus recovery after BUS_OFF condition. Set 0 to disable auto recovery.
 * @param logPeriod Interval in ms for periodic logging of stats. Default: 0 <- disabled.
 */
tNMEA2000_esp32xx::tNMEA2000_esp32xx(int _TxPin, int _RxPin, unsigned long recoveryPeriod, unsigned long logPeriod)
    : tNMEA2000(), RxPin(_RxPin), TxPin(_TxPin)
{
    if (RxPin < 0 || TxPin < 0)
    {
        state = ST_DISABLED;
    }
    else
    {
        recoveryTimer = tN2kSyncScheduler(false, recoveryPeriod, 0);
        logTimer = tN2kSyncScheduler(false, logPeriod, 0);
    }
}

bool tNMEA2000_esp32xx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
    if (ST_RUNNING != state)
    {
        return false;
    }
    logDebug(LOG_MSG, "twai transmit id %ld, len %d", LOGID(id), (int)len);

    twai_message_t message;
    memset(&message, 0, sizeof(message));
    message.identifier = id;
    message.extd = 1;
    message.data_length_code = len;
    memcpy(message.data, buf, len);
    esp_err_t rt = twai_transmit(&message, 0);
    if (rt == ESP_ERR_INVALID_STATE)
    {
        // Probably driver not in RUNNING state, don't spam the console here.
        return false;
    }
    if (rt == ESP_ERR_TIMEOUT)
    {
        txTimeouts++;
        logDebug(LOG_DEBUG, "twai tx timeout");
        return false;
    }
    if (ESP_OK != rt)
    {
        logDebug(LOG_ERR, "twai tx %ld failed: %x", LOGID(id), (int)rt);
        return false;
    }

    txTimeouts = 0;

    return true;
}

bool tNMEA2000_esp32xx::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
    if (ST_RUNNING != state)
    {
        return false;
    }
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
        logDebug(LOG_DEBUG, "twai: received invalid message %ld, len %d", LOGID(id), len);
        len = 8;
    }
    logDebug(LOG_MSG, "twai rcv id=%d,len=%d, ext=%d", LOGID(message.identifier), message.data_length_code, message.extd);
    if (!message.rtr)
    {
        memcpy(buf, message.data, message.data_length_code);
    }
    return true;
}

bool tNMEA2000_esp32xx::CANOpen()
{
    if (ST_STOPPED != state) // CANOpen should only be called in STOPPED state
    {
        logDebug(LOG_INFO, "CANOpen invalid state");
        return true;
    }
    esp_err_t rt = twai_start();
    if (rt != ESP_OK)
    {
        logDebug(LOG_ERR, "CANOpen failed: %x", (int)rt);
        state = ST_ERROR;
        return false;
    }
    else
    {
        logDebug(LOG_DEBUG, "CANOpen ok");
    }
    // Start timers now.
    logTimer.UpdateNextTime();
    return true;
}

// This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
// and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
void tNMEA2000_esp32xx::InitCANFrameBuffers()
{
    if (ST_DISABLED == state)
    {
        logDebug(LOG_INFO, "twai init - disabled");
    }
    else
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TxPin, (gpio_num_t)RxPin, TWAI_MODE_NORMAL);
        g_config.tx_queue_len = 20;
        g_config.rx_queue_len = 40; // need a large Rx buffer to prevent rxmiss
        g_config.intr_flags |= ESP_INTR_FLAG_LOWMED; // LOWMED might be needed if you run out of LEVEL1 interrupts.
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
            state = ST_DISABLED;
        }
    }

    // call parent function
    tNMEA2000::InitCANFrameBuffers();
}

/**
 * @brief This must be called periodically from your task loop
 *
 */
void tNMEA2000_esp32xx::loop()
{
    // No way out of DISABLED state
    if (ST_DISABLED != state)
    {
        twai_status_info_t twai_status;
        if (ESP_OK != twai_get_status_info(&twai_status))
        {
            logDebug(LOG_ERR, "twai driver broken");
            state = ST_DISABLED; // twai driver is broken, no need to keep trying and spamming logs
            return;
        }

        const bool autoRecoveryEnabled = (0 != recoveryTimer.GetPeriod());

        switch (twai_status.state)
        {
        case TWAI_STATE_STOPPED:
            // Stopped > Running
            if (autoRecoveryEnabled)
            {
                logDebug(LOG_DEBUG, "twai STOPPED --> start");
                twai_start();
                state = ST_RESTARTING;
                txTimeouts = 0;
                recoveryTimer.UpdateNextTime(); // start recovery timer
            }
            else
            {
                logDebug(LOG_DEBUG, "twai STOPPED");
                state = ST_STOPPED;
            }

            break;
        case TWAI_STATE_RUNNING:
            if (txTimeouts >= TIMEOUT_OFFLINE)
            {
                twai_stop();
                state = ST_STOPPED;
            }
            else if (autoRecoveryEnabled && (ST_RESTARTING == state))
            {
                if (recoveryTimer.IsTime())
                {
                    recoveryTimer.Disable(); // one-shot
                    // Now the higher-level NMEA2000 library should to be restarted!
                    logDebug(LOG_DEBUG, "twai start --> Restart");
                    tNMEA2000::Restart();
                    state = ST_RUNNING;
                }
            }
            else
            {
                state = ST_RUNNING;
            }
            break;
        case TWAI_STATE_BUS_OFF:
            // Bus-Off > Recovering
            if (autoRecoveryEnabled)
            {
                logDebug(LOG_DEBUG, "twai BUS_OFF --> recovery");
                twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
                state = ST_RECOVERING;
            }
            else
            {
                logDebug(LOG_ERR, "twai BUS_OFF");
                state = ST_BUS_OFF;
            }

            break;
        case TWAI_STATE_RECOVERING:
            state = ST_RECOVERING;
            break;
        default:
            state = ST_ERROR;
            break;
        }
    }

    if (logTimer.IsTime())
    {
        logTimer.UpdateNextTime();
        logStatus();
    }
}

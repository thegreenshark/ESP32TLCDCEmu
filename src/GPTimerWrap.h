#ifndef _GPTIMERWRAP_H
#define _GPTIMERWRAP_H

#include "driver/gptimer.h"

class GPTimerWrap
{
private:
    gptimer_handle_t timer;
    bool is_running;
public:
    GPTimerWrap
    (
        uint32_t resolution_hz, double interval_sec, bool auto_reload = true,
        gptimer_alarm_cb_t alarm_callback = NULL
    );
    ~GPTimerWrap();
    bool isRunning();
    void start();
    void stop();
    void setCounter(uint64_t value);
    uint64_t getCounter();
};

#endif

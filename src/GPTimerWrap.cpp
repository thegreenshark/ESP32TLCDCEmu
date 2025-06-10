#include "GPTimerWrap.h"

GPTimerWrap::GPTimerWrap
(
    uint32_t resolution_hz, double interval_sec, bool auto_reload,
    gptimer_alarm_cb_t alarm_callback
)
:
is_running(false)
{
    gptimer_config_t timer_config =
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = resolution_hz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_alarm_config_t alarm_config =
    {
        .alarm_count = uint64_t(interval_sec * resolution_hz),
        .reload_count = 0,
        .flags =
        {
            auto_reload //auto_reload_on_alarm
        }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    if (alarm_callback != NULL)
    {
        gptimer_event_callbacks_t event_callbacks =
        {
            .on_alarm = alarm_callback
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &event_callbacks, this));
    }

    setCounter(0);
    ESP_ERROR_CHECK(gptimer_enable(timer));
}

GPTimerWrap::~GPTimerWrap()
{
    stop();
    ESP_ERROR_CHECK(gptimer_disable(timer));
    ESP_ERROR_CHECK(gptimer_del_timer(timer));
}

bool GPTimerWrap::isRunning()
{
    return is_running;
}

void GPTimerWrap::start()
{
    if (!is_running)
    {
        ESP_ERROR_CHECK(gptimer_start(timer));
        is_running = true;
    }
}

void GPTimerWrap::stop()
{
    if (is_running)
    {
        ESP_ERROR_CHECK(gptimer_stop(timer));
        is_running = false;
    }
}

void GPTimerWrap::setCounter(uint64_t value)
{
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer, value));
}

uint64_t GPTimerWrap::getCounter()
{
    uint64_t value;
    ESP_ERROR_CHECK(gptimer_get_raw_count(timer, &value));
    return value;
}

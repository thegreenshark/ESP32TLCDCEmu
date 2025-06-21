/*
	Tuner List CD Changer Emulator
	based on code from https://github.com/Tomasz-Mankowski/MeganeBT
*/

#ifndef _TLCDCEMU_H
#define _TLCDCEMU_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_avrc_api.h"
#include "esp_types.h"
#include "driver/uart.h"
#include "driver/gptimer.h"

#include "esp32-hal-log.h"
#include "esp32-hal-uart.h"

#include "AudioConfigLocal.h"
#include "AudioTools.h"
#include "AudioTools/AudioLibs/SPDIFOutput.h"
#include "BluetoothA2DPSink.h"
#include "GPTimerWrap.h"

// #define HEAP_DEBUGGING

#define LOG_TAG "TLCDCEmu"
#define BUFF_SIZE 256

//timer defs
#define TIMERS_RESOLUTION_HZ  1000000
#define FAKEPLAY_TIMER_INTERVAL_SEC 1
#define CDC_WAIT_TIMER_INTERVAL_SEC 0.20

typedef enum{
	WAITING,
	CONFIRMED,
	TIMEOUT
}CDC_Wait_E;

typedef enum{
	WAIT_BOOT,
	BOOT_SEQUENCE,
	WAIT_HU_VERSION,
	CONFIRM_HU_VERSION,

	RECEIVED_PLAY,
	RECEIVED_PAUSE,
	RECEIVED_STANDBY,
	RECEIVED_CD_CHANGE,
	RECEIVED_NEXT,
	RECEIVED_PREV,

	OPERATE_STANDBY,
	OPERATE_PAUSED,
	OPERATE_PREPARE_PLAY,
	OPERATE_PLAYING
}CDC_State;

typedef struct {
    GPTimerWrap *timer_wrap_ptr; //timer that has triggered the event
} timer_event_t;

class TLCDCEmu
{
	public:
		TLCDCEmu();
		~TLCDCEmu();
		void init(int spdif_pin, int cdc_tx_pin, int cdc_rx_pin, const char * btName = "Tuner List CDC Emulator");
		void talk();

	private:
		uint8_t CDC_SendPacket(uint8_t *data, uint8_t length, uint8_t retries);
		static uint8_t CDC_checksum(const uint8_t *data, uint8_t length);
		void CDC_ConfirmSongChange();
		static void readHU(const uint8_t *data, uint16_t length);
		void fakePlay();
		static void uart_event_task(void *pvParameters);
		uint8_t con1, con2, con3, con4;

		uart_config_t uart_config;
		xTaskHandle uartHandle;

		//timer
		static void timer_evt_task(void *arg);
};

#endif

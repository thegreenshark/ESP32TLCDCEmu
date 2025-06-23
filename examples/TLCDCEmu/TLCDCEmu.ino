#include <TLCDCEmu.h>

const int spdif_pin = 22;  //ESP32 SPDIF output ---> Head unit connector C3 pin 18
const int cdc_tx_pin = 15; //ESP32 UART TX ---> Head unit connector C3 pin 14
const int cdc_rx_pin = 16; //ESP32 UART RX <--- Head unit connector C3 pin 13

TLCDCEmu emulator;

void setup(){
  delay(10);
  emulator.init(spdif_pin, cdc_tx_pin, cdc_rx_pin, "My Car CDC Emulator");
}

void loop(){
  emulator.talk();
}
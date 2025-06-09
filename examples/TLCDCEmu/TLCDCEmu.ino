#include <TLCDCEmu.h>

const int spdif_pin = 22;
const int cdc_tx_pin = 15;
const int cdc_rx_pin = 16;

TLCDCEmu emulator;

void setup(){
  delay(10);
  emulator.init(spdif_pin, cdc_tx_pin, cdc_rx_pin, "My Car CDC Emulator");
}

void loop(){
  emulator.talk();
}
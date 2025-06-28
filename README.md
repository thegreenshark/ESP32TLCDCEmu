# ESP32 TLCDCEmu Arduino library
CD changer emulator for Tuner List / Update List Renault radio. Allows to play audio via Bluetooth (A2DP). Some AVRCP commands are also suppoerted (play, pause, next, previous) which allows to control the playback by pressing the buttons on the head unit.

This is a fork of JohnyMielony's [ESP32TLCDCEmu](https://github.com/JohnyMielony/ESP32TLCDCEmu) repo.

Lots of useful info about Renault CDC emulation at [TLCDCEmu SourceForge](https://tlcdcemu.sourceforge.net/hardware.html).

### Compatibility
The device has only been tested with a 8200444073T (RENRCW201-00) Update List radio found in 2006 Renault Symbol (Clio mk2).

**It may not be compatible with other radio models.** But it's expected to work with most Tuner List and Update List models.

### Hardware
The recommended ESP32 module is ESP32-WROOM-32 or a board based on it (such as ESP32 DevKit V1).

If your're using a bare ESP32-WROOM-32 module, you'll also need a programmer device in order to upload and debug the firmware. A popular option is a PL2303HX or CH340G based USB-TTL adapter.

The radio connector is a Mini-iso 8 pin blue connector.

Original schematic by JohnyMielony: [Discussion](https://github.com/JohnyMielony/ESP32TLCDCEmu/issues/3).

Recommended values: R3 = 510 ohm, R4 = 100 ohm  
![Schematic](/schematic.png)

### Installing
This library has the following requirements:  
[Espressif ESP32 Arduino core v3.2.0](https://github.com/espressif/arduino-esp32/releases/tag/3.2.0)  
[ESP32-A2DP v1.8.7](https://github.com/pschatzmann/ESP32-A2DP/releases/tag/v1.8.7)  
[arduino-audio-tools v1.1.2](https://github.com/pschatzmann/arduino-audio-tools/releases/tag/v1.1.2)  

Instructions on installing ESP32 Arduino core can be found [here](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html). Make sure to install the correct version.

This library and the other requirements can be installed using git:
```
cd /path/to/arduino/libraries/folder/
git clone https://github.com/thegreenshark/ESP32TLCDCEmu
git clone --branch v1.8.7 https://github.com/pschatzmann/ESP32-A2DP
git clone --branch v1.1.2 https://github.com/pschatzmann/arduino-audio-tools
```
Or manually download and put them in the arduino libraries folder.

### Building and uploading
Open the example sketch included in this library:
```
File -> Examples -> ESP32 Tuner List CDC Emulator -> TLCDCEmu
```
Pinout in the example sketch corresponds to the provided schematic. If you need to change the pin numbers or make any other modifications to the sketch, you can create a new sketch and copy the code.

Now the ESP32 board settings (found in Tools menu) need to be configured. The recommended settings are:

- Board: "ESP32 Dev Module" *(if using ESP32-WROOM-32)*
- Upload speed: "921600"
- CPU Frequency: "240MHz (WiFi/BT)"
- Flash Frequency: "80MHz"
- Flash Mode: "QIO"
- Flash Size: "4MB (32Mb)"
- Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
- Core Debug Level: "Info" *(when you're testing)* or "None" *(when you're done)*
- PSRAM: "Disabled"
- Arduino Runs On: "Core 1"
- Events Run On: "Core 1"
- Erase All Flash Before Sketch Upload: "Disabled"
- JTAG Adapted: "Disabled"
- Zigbee Mode: "Disabled:

Connect ESP32 to your PC, compile and upload the sketch.

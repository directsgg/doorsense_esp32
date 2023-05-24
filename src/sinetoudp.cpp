/**
 * @file example-udp-send.ino
 * @author Phil Schatzmann
 * @brief Sending audio over udp. Because the SineWaveGenerator is providing the data as fast 
 * as possible we need to throttle the speed.
 * @version 0.1
 * @date 2022-03-09
 *
 * @copyright Copyright (c) 2022
 */

/*

#include <Arduino.h>
#include "AudioTools.h"
#include "AudioLibs/Communication.h"
//#include "AudioLibs/AudioKit.h"
const char* ssid = "Nexxt_C44058";
const char* password = "miinternet";
AudioInfo info(16000, 1, 16);
SineWaveGenerator<int16_t> sineWave(32000);
GeneratedSoundStream<int16_t> sound( sineWave); // strean generated from sine wave
UDPStream udp(ssid, password);
Throttle throttle;
IPAddress udpAddress(255, 255, 255, 255);
const int udpPort = 12345;
StreamCopy copier(udp, sound); // copies sound into i2s
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  // define udp address and port
  udp.begin(udpAddress, udpPort);

  // Define Throttle
  auto cfg = throttle.defaultConfig();
  cfg.copyFrom(info);
  throttle.begin(cfg);

  // Setup sine wave
  sineWave.begin(info, N_B4);

  Serial.println("started...");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  throttle.startDelay();
  int bytes = copier.copy();
  throttle.delayBytes(bytes);
}

*/
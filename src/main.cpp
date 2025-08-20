

// User defines
#define SERIAL_BAUD_RATE 115200
#define LD2410_BAUD_RATE 115200
#include <Arduino.h>
#include "LD2410S.h"

// Pin definitions
constexpr int RX_PIN = 2;
constexpr int TX_PIN = 1;
constexpr size_t SERIAL_NUMBER_BUFFER_SIZE = 8;

// Sensor object
LD2410S ld2410s(Serial1);

// Firmware version variables
uint16_t firmwareMajor = 0;
uint16_t firmwareMinor = 0;
uint16_t firmwarePatch = 0;

// Serial number buffer
char serialNumberBufferOut[SERIAL_NUMBER_BUFFER_SIZE];
char serialNumberBufferIn[SERIAL_NUMBER_BUFFER_SIZE];
String serialNumberString;

// Common parameters
uint32_t farDistance = 0;
uint32_t nearDistance = 0;
uint32_t delayTime = 0;
uint32_t freqStatus = 0;
uint32_t freqDistance = 0;
uint32_t responseTime = 0;

//motion variables
bool motionDetected = false;
uint16_t motionDistance = 0;
uint32_t motionSeq = 0;
uint64_t motionTs = 0;
uint16_t reserved = 0;
uint32_t gatesEnergy[16] = {0};

uint8_t progress = 0;

LD2410S::StandardData data;

// Timing variables
unsigned long lastStatusMillis = 0;

void printProgressBar(int percent) {
    const int barWidth = 50; // characters
    int pos = (percent * barWidth) / 100;

    Serial.print("\r[");
    for (int i = 0; i < barWidth; i++) {
        if (i < pos) Serial.print("=");
        else if (i == pos) Serial.print(">");
        else Serial.print(" ");
    }
    Serial.printf("] %3d%%", percent);
    Serial.flush();
}

void setup() {
  Serial.begin(115200);
  ld2410s.begin(RX_PIN, TX_PIN, LD2410_BAUD_RATE);
  Serial.println("Starting");
  int now = millis();
  delay(1000);

  if (ld2410s.enterConfigMode())
  {
      Serial.println("Entered config mode successfully");
  } else {
      Serial.println("Failed to enter config mode");
  }
  delay(250);
  if (ld2410s.readFirmwareVersion(firmwareMajor, firmwareMinor, firmwarePatch, 5, 1000)) {
    Serial.printf("Firmware Version: %d.%d.%d\n", firmwareMajor, firmwareMinor, firmwarePatch);
  } else {
    Serial.println("Failed to read firmware version");
  }

  String firmwareString = ld2410s.firmwareString();
  Serial.printf("Firmware String: %s\n", firmwareString.c_str());

  delay(250);
  if (ld2410s.readSerialString(serialNumberString, 5, 1000)) {
    Serial.printf("Serial Number: %s\n", serialNumberString.c_str());
  } else {
    Serial.println("Failed to read serial number");
  }
  delay(250);
  serialNumberBufferOut[0] = '8';
  serialNumberBufferOut[1] = '7';
  serialNumberBufferOut[2] = '6';
  serialNumberBufferOut[3] = '5';
  serialNumberBufferOut[4] = '4';
  serialNumberBufferOut[5] = '3';
  serialNumberBufferOut[6] = '2';
  serialNumberBufferOut[7] = '1';

  if (ld2410s.writeSerialNumber(serialNumberBufferOut, SERIAL_NUMBER_BUFFER_SIZE, 5, 1000)) {
    Serial.println("Serial number written successfully");
  } else {
    Serial.println("Failed to write serial number");
  }
  delay(250);
  if (ld2410s.readSerialNumber(serialNumberBufferIn, SERIAL_NUMBER_BUFFER_SIZE, 5, 1000)) {
    Serial.printf("Serial Number Buffer In: ");
    for (int i = 0; i < SERIAL_NUMBER_BUFFER_SIZE; i++) {
      Serial.print(serialNumberBufferIn[i]);
    }
    Serial.println();
  } else {
    Serial.println("Failed to read serial number");
  }
  delay(250);
  if (ld2410s.readCommonParameters(farDistance, nearDistance, delayTime, freqStatus, freqDistance, responseTime, 5, 1000)) {
    Serial.printf("Far Distance: %d m\n", farDistance);
    Serial.printf("Near Distance: %d m\n", nearDistance);
    Serial.printf("Delay Time: %d ms\n", delayTime);
    Serial.printf("Frequency Status: %d Hz\n", freqStatus);
    Serial.printf("Frequency Distance: %d Hz\n", freqDistance);
    Serial.printf("Response 5:Slow, 10:Fast : %d\n", responseTime);
  } else {
    Serial.println("Failed to read common parameters");
  }
  farDistance = 4;
  nearDistance = 0;
  delayTime = 20;
  freqStatus = 4;
  freqDistance = 4;
  responseTime = 10;
  delay(250);
  if (ld2410s.writeGenericParameters(farDistance, nearDistance, delayTime, freqStatus, freqDistance, responseTime, 5, 1000)) {
    Serial.println("Common parameters written successfully");
  } else {
    Serial.println("Failed to write common parameters");
  }
  delay(250);
  if (ld2410s.readCommonParameters(farDistance, nearDistance, delayTime, freqStatus, freqDistance, responseTime, 5, 1000)) {
    Serial.printf("Far Distance: %d m\n", farDistance);
    Serial.printf("Near Distance: %d m\n", nearDistance);
    Serial.printf("Delay Time: %d ms\n", delayTime);
    Serial.printf("Frequency Status: %d Hz\n", freqStatus);
    Serial.printf("Frequency Distance: %d Hz\n", freqDistance);
    Serial.printf("Response 5:Slow, 10:Fast : %d\n", responseTime);
  } else {
    Serial.println("Failed to read common parameters");
  }
  delay(250);
  uint32_t triggers[16];
    if (ld2410s.readTriggerThresholds(triggers, sizeof(triggers)/sizeof(triggers[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(triggers[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read trigger threshold");
    }
  delay(250);
  uint32_t hold[16];
    if(ld2410s.readHoldThresholds(hold, sizeof(hold)/sizeof(hold[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(hold[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read hold threshold");
    }
  delay(250);
  uint32_t triggerOut[16] = {
    32, 32, 32, 32,
    32, 32, 32, 32,
    32, 32, 32, 32,
    32, 32, 32, 32
  };
  if (ld2410s.writeTriggerThresholds(triggers, sizeof(triggers)/sizeof(triggers[0]), 5, 1000)) {
    Serial.println("Trigger threshold written successfully");
  } else {
    Serial.println("Failed to write trigger threshold");
  }
  delay(250);
  uint32_t holdOut[16] = {
    31, 31, 31, 31,
    31, 31, 31, 31,
    31, 31, 31, 31,
    31, 31, 31, 31
  };
  if (ld2410s.writeHoldThresholds(hold, sizeof(hold)/sizeof(hold[0]), 5, 1000)) {
    Serial.println("Hold threshold written successfully");
  } else {
    Serial.println("Failed to write hold threshold");
  }
  delay(250);
  if (ld2410s.readTriggerThresholds(triggers, sizeof(triggers)/sizeof(triggers[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(triggers[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read trigger threshold");
    }
  delay(250);
  if(ld2410s.readHoldThresholds(hold, sizeof(hold)/sizeof(hold[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(hold[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read hold threshold");
    }
    delay(250);
    Serial.println("Starting auto update threshold command in 10 seconds, leave room for 4 minutes...");
    delay(10000);
    if (ld2410s.autoUpdateThresholds(2,1,120,4,1000)) {
      delay(250);
      ld2410s.exitConfigMode();
      int failCount = 0;
      for (;;) {
        ld2410s.loop();
        ld2410s.getProgressData(progress, &motionSeq, &motionTs);
        if (progress == 0) {
          Serial.println("No progress frame yet...");
          if (++failCount >= 10) {
            Serial.println("Auto-config progress failed after 10 attempts, exiting...");
            break;
          }
          delay(1000);
          } else {    
          printProgressBar(progress);
          if (progress >= 100) {
            Serial.println("Auto-config complete!");
            break;
          }
        }
        delay(250);
      }
    } else {
      Serial.println("Failed to execute auto update threshold command");
    }
  delay(250);
  if (ld2410s.enterConfigMode())
  {
      Serial.println("Entered config mode successfully");
  } else {
      Serial.println("Failed to enter config mode");
  }
  delay(250);
  if (ld2410s.readTriggerThresholds(triggers, sizeof(triggers)/sizeof(triggers[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(triggers[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read trigger threshold");
    }
  delay(250);
  if(ld2410s.readHoldThresholds(hold, sizeof(hold)/sizeof(hold[0]), 5, 1000)) {
      for(int i = 0; i < 16; i++) {
        Serial.print(hold[i]);
        Serial.print(" ");        
      }
      Serial.println();
    } else {
      Serial.println("Failed to read hold threshold");
    }
   delay(250);
  if (ld2410s.switchToStandardMode()) {
    Serial.println("Switched to standard mode successfully");
  } else {
    Serial.println("Failed to switch to standard mode");
  }
  delay(250);
  if (ld2410s.exitConfigMode())
  {
      Serial.println("Exited config mode successfully");
  }
  
  lastStatusMillis = millis();
}

void loop() {
  ld2410s.loop();    
  
    //Print status every second
    if (millis() - lastStatusMillis >= 1000) {
      if (ld2410s.getStandardData(data)) {
        Serial.printf("-------------------------------------------------------------------------------------------\n");
        Serial.printf("Target Distance: %.2f feet, (%d cm), Sequence Number: %d, State: %d\n", ld2410s.latestDistanceFeet(), data.distance_cm, data.seq, data.target_state);
        Serial.printf("Gates Energy: ");
        for (int i = 0; i < 16; i++) {
          Serial.print(data.energy[i]);
          Serial.print(" ");
          }
          Serial.println();
          Serial.printf("Gates Noise: ");
          for (int i = 0; i < 16; i++) {
            Serial.print(data.noise[i]);
            Serial.print(" ");
          }
          Serial.println();
          Serial.printf("Gates SNR: ");
          for (int i = 0; i < 16; i++) {
            Serial.printf("%.2f", data.snr_db_q8[i] / 256.0);
            Serial.print(" ");
          }
          Serial.println();
          lastStatusMillis = millis();
      }
    }
}
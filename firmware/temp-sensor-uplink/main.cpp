

#include "events/EventQueue.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "mbed.h"
#include <cstdint>
#include <stdio.h>

// Application helpers
// #include "trace_helper.h"
#include "helpers.h"


PinName SDAPin = PA_11;
PinName SCLPin = PA_12;

bool nodeOk = true;
 uint32_t sleepIntervalMs = 30000; // default sleep time 30s



#include "STM32WL_LoRaRadio.h"
STM32WL_LoRaRadio radio;
using namespace events;


// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t txBufferPointer = 0;
uint8_t rx_buffer[30];

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS 15

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER 3

/**
 * This event queue is the global event queue for both the
 * application and stack. To conserve memory, the stack is designed to run
 * in the same thread as the application and the application is responsible for
 * providing an event queue to the stack that will be used for ISR deferment as
 * well as application information event queuing.
 */
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

// /**
//  * Event handler.
//  *
//  * This will be passed to the LoRaWAN stack to queue events for the
//  * application which in turn drive the application.
//  */
static void lora_event_handler(lorawan_event_t event);

// /**
//  * Constructing Mbed LoRaWANInterface and passing it the radio object from
//  lora_radio_helper.
//  */
static LoRaWANInterface lorawan(radio);

// /**
//  * Application specific callbacks
//  */
static lorawan_app_callbacks_t callbacks;



int main(void) {
      // as per low-power suggestion
  mbed_file_handle(STDIN_FILENO)->enable_input(false);

  // stores the status of a call to LoRaWAN protocol
  lorawan_status_t retcode;

  // Initialize LoRaWAN stack
  lorawan.initialize(&ev_queue);


   // prepare application callbacks
  callbacks.events = mbed::callback(lora_event_handler);
  lorawan.add_app_callbacks(&callbacks);

  // Set number of retries in case of CONFIRMED messages
  if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER) !=
      LORAWAN_STATUS_OK) {
    printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
    return -1;
  }

  // Enable adaptive data rate
  if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
    printf("\r\n enable_adaptive_datarate failed! \r\n");
    return -1;
  }

  ev_queue.dispatch_forever();

  return 0;
}



// /**
//  * Copyright (c) 2017, Arm Limited and affiliates.
//  * SPDX-License-Identifier: Apache-2.0
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */
// #include "events/EventQueue.h"
// #include "lorawan/LoRaWANInterface.h"
// #include "lorawan/system/lorawan_data_structures.h"
// #include "mbed.h"
// #include <cstdint>
// #include <stdio.h>

// // Application helpers
// // #include "trace_helper.h"
// #include "helpers.h"

// #define DEBUGGING false
// bool nodeOk = true;

// //DigitalOut led1(LED1);
// //   DigitalOut led2(LED2);

// PinName SDAPin = PA_11;
// PinName SCLPin = PA_12;

// // I2C slave addreses
// const uint8_t sensorAddr[1] = {0x44};
// const uint8_t sensType[1] = {0x03};
// /**
// values for reading battery voltage (5-3.3V)
// */
// DigitalOut batVoltageADCEN(PB_5);
// PinName batADCPin = PB_4;

// // calibration is done to find out 2 coefficients
// // vBat = ADCk*ADCValue*1000 + ADCb
// const float ADCk = 0.0083;
// const float ADCb = -0.2801;

// #include "STM32WL_LoRaRadio.h"
// STM32WL_LoRaRadio radio;

// using namespace events;

// // Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// // This example only communicates with much shorter messages (<30 bytes).
// // If longer messages are used, these buffers must be changed accordingly.
// uint8_t tx_buffer[30];
// uint8_t txBufferPointer = 0;
// uint8_t rx_buffer[30];

// uint32_t sleepIntervalMs = 30000; // default sleep time 30s

// /**
//  * Maximum number of events for the event queue.
//  * 10 is the safe number for the stack events, however, if application
//  * also uses the queue for whatever purposes, this number should be increased.
//  */
// #define MAX_NUMBER_OF_EVENTS 15

// /**
//  * Maximum number of retries for CONFIRMED messages before giving up
//  */
// #define CONFIRMED_MSG_RETRY_COUNTER 3

// /**
//  * This event queue is the global event queue for both the
//  * application and stack. To conserve memory, the stack is designed to run
//  * in the same thread as the application and the application is responsible for
//  * providing an event queue to the stack that will be used for ISR deferment as
//  * well as application information event queuing.
//  */
// static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

// // /**
// //  * Event handler.
// //  *
// //  * This will be passed to the LoRaWAN stack to queue events for the
// //  * application which in turn drive the application.
// //  */
// static void lora_event_handler(lorawan_event_t event);

// // /**
// //  * Constructing Mbed LoRaWANInterface and passing it the radio object from
// //  lora_radio_helper.
// //  */
// static LoRaWANInterface lorawan(radio);

// // /**
// //  * Application specific callbacks
// //  */
// static lorawan_app_callbacks_t callbacks;

// // I2C low level functions
// static uint8_t readReg(uint8_t addr, uint8_t reg) {
//   I2C i2c(SDAPin, SCLPin);

//   char regAddr[1] = {reg};
//   char data[1];

//   i2c.write(addr, regAddr, 1, true);

//   i2c.read(addr, data, 1, false);

//   i2c.stop();

//   i2c.free();

//   return data[0];
// }

// static uint16_t readReg16(uint8_t addr, uint8_t reg) {
//   I2C i2c(SDAPin, SCLPin);

//   char regAddr[1] = {reg};
//   char data[2];

//   i2c.write(addr, regAddr, 1, true);

//   i2c.read(addr, data, 2, false);

//   i2c.stop();

//   i2c.free();
//   return (data[0] << 8 | data[1]);
// }

// static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
//   I2C i2c(SDAPin, SCLPin);
//   char msg[2] = {reg, value};
//   bool response = i2c.write(addr, msg, 2, false);
//   i2c.free();
//   return response;
// }

// static bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value) {
//   I2C i2c(SDAPin, SCLPin);
//   uint8_t MSB = value >> 8;
//   uint8_t LSB = value & 0xFF;
//   char msg[3] = {reg, MSB, LSB};

//   bool response = i2c.write(addr, msg, 3, false);
//   i2c.free();
//   return response;
// }

// static float getTemp(uint8_t addr) {
//     return 1.0;
// }

// static float getHum(uint8_t addr){
//     return 1.0;
// }

// float readBatteryVoltage() {
//   sleep_manager_lock_deep_sleep();
//   batVoltageADCEN = 1;
//   AnalogIn batVoltageADC(batADCPin);
//   batVoltageADC.set_reference_voltage(3.3);
//   batVoltageADC.read(); // first read is nonsense

//   //   thread_sleep_for(100); //this did not help
//   float adcValRaw = batVoltageADC.read();
//   int adcVal = (int)(adcValRaw * 1000);

//   float vBat = adcVal * ADCk + ADCb;
//   batVoltageADCEN = 0;

// #if DEBUGGING
//   printf("adcraw: %1.3f\t", adcValRaw);
//   printf("adcVal: %d \t", adcVal);
//   printf("adcVoltage %1.4f \r\n", vBat);
// #endif
//   sleep_manager_unlock_deep_sleep();
//   return vBat;
// }


// /**
//  * Entry point for application
//  */
// int main(void) {

//   // as per low-power suggestion
//   mbed_file_handle(STDIN_FILENO)->enable_input(false);

//   // stores the status of a call to LoRaWAN protocol
//   lorawan_status_t retcode;

//   // Initialize LoRaWAN stack
//   if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
//     printf("\r\n LoRa initialization failed! \r\n");
//     return -1;
//   }

//   printf("\r\n Mbed LoRaWANStack initialized \r\n");

//   // prepare application callbacks
//   callbacks.events = mbed::callback(lora_event_handler);
//   lorawan.add_app_callbacks(&callbacks);

//   // Set number of retries in case of CONFIRMED messages
//   if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER) !=
//       LORAWAN_STATUS_OK) {
//     printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
//     return -1;
//   }

//   printf("\r\n CONFIRMED message retries : %d \r\n",
//          CONFIRMED_MSG_RETRY_COUNTER);

//   // Enable adaptive data rate
//   if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
//     printf("\r\n enable_adaptive_datarate failed! \r\n");
//     return -1;
//   }

//   printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

//   retcode = lorawan.connect();

//   if (retcode == LORAWAN_STATUS_OK ||
//       retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
//   } else {
//     printf("\r\n Connection error, code = %d \r\n", retcode);
//     return -1;
//   }

//   printf("\r\n Connection - In Progress ...\r\n");

//   // make your event queue dispatching events forever
//   ev_queue.dispatch_forever();

//   return 0;
// }

// /**
//  * Sends a message to the Network Server
//  */
static void send_message() {

//   //   /**
//   //   NOTES for encoding
//   //   each sensor node data start with idByte

//   //   bit 0:4 - data type:
//   //       0x00 - soil temperature
//   //       0x01 - air temperature
//   //       0x02 - soil moisture
//   //       0x03 - air temp/hum
//   //       0x04 -
//   //       0x05 -
//   //       0x06 -
//   //       0x07 -
//   //       0x08 -
//   //       0x09 -
//   //       0x0A - battery voltage
//   //       0x0B -
//   //       0x0C -
//   //       0x0D -
//   //       0x0E -
//   //       0x0F - error

//   //   bit 5:8 - sensor level
//   //       0x00 - closes to the bottom
//   //       0xF0 - closes to the top
//   //       always start by using 0x00 and then move to the top
//   //       as different sensor can be easly added above the surface.
//   //   **/

//   // read all sensors
//   uint8_t soilsensIndex = 0;
//   for (uint8_t sensId = 0; sensId < sizeof(sensorAddr); sensId++) {

//     uint8_t addr8Bit = sensorAddr[sensId] << 1;
//     uint8_t idByte = (sensId << 4) | sensType[sensId];

//     switch (sensType[sensId]) {
//     case 0x03: {
//         char sensData[6];

//         I2C i2c(SDAPin, SCLPin);
//         char msg[2] = {0x24, 0x16};
//         bool response = i2c.write(addr8Bit, msg, 2, false);

//         thread_sleep_for(2000);
        
//         i2c.read(addr8Bit, sensData, 6, false);

//         i2c.stop();
//         i2c.free();
//         uint8_t dataFromSens[6];

//         //calcualte Temp
//         uint16_t tempData = (uint16_t)dataFromSens[0] << 8 | dataFromSens[1];
//         float tempC = -45 + 175 * (float)tempData / (65536 - 1);

//         //prepeare for sending temp
//         // this is so that we never have negative values and use as much bits as
//         // necessary, not more
//         uint8_t tempOffset = 40; //(273.15-233.15);
//         float tempOffseted = tempC + tempOffset;
//         uint16_t tempDataToSend = tempOffseted * 100; //*100 to have 2 decimals

//         uint8_t tempDataUpper = tempDataToSend >> 8;
//         uint8_t tempDataLower = tempDataToSend & 0xFF;

//         // fill tx buffer with the temperature reading
//         tx_buffer[txBufferPointer] = idByte;
//         tx_buffer[txBufferPointer + 1] = tempDataUpper;
//         tx_buffer[txBufferPointer + 2] = tempDataLower;
//         txBufferPointer = txBufferPointer + 3; // set ready for the next player.
       
//         //calcualte RH
//         uint16_t humData = (uint16_t)dataFromSens[3] << 8 | dataFromSens[4];
//         float hum = 100 * (float)humData / (65536 - 1);

//         uint16_t humDataToSend = humDataToSend *100;
//         uint8_t humDataUpper = humDataToSend >> 8;
//         uint8_t humDataLower = humDataToSend & 0xFF;

//         // fill tx buffer with the temperature reading
//         tx_buffer[txBufferPointer] = idByte;
//         tx_buffer[txBufferPointer + 1] = humDataUpper;
//         tx_buffer[txBufferPointer + 2] = humDataLower;
//         txBufferPointer = txBufferPointer + 3; // set ready for the next player.
  
//       break;
//     }
//     };
//   }

//   // read battery voltage
//   float voltageOffset = 3.3;
//   uint8_t batVValue = (readBatteryVoltage() - voltageOffset) * 100;
//   uint8_t idByte = (0x00 | 0x0A); // probe level is left empty and type value is
//                                   // added to the 0:3 bits
//   // fill tx buffer
//   tx_buffer[txBufferPointer] = idByte;
//   tx_buffer[txBufferPointer + 1] = batVValue;
//   txBufferPointer = txBufferPointer + 2; // set ready for the next player.

//   int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer,
//                                  txBufferPointer, MSG_UNCONFIRMED_FLAG);

//   if (retcode < 0) {
//     retcode == LORAWAN_STATUS_WOULD_BLOCK
//         ? printf("send - WOULD BLOCK\r\n")
//         : printf("\r\n send() - Error code %d \r\n", retcode);

//     if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
//       // retry in 3 seconds
//       if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
//         ev_queue.call_in(3000, send_message);
//       }
//     }
//     return;
//   }

//   //   printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
//   memset(tx_buffer, 0, sizeof(tx_buffer)); // clear buffer
//   txBufferPointer = 0;
}

/**
 *Send error message that something is worong and node is not functional
 */
void send_error_message() {
//   tx_buffer[txBufferPointer] = 0x0F; // error message
//   txBufferPointer = txBufferPointer + 1;

//   int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer,
//                                  txBufferPointer, MSG_UNCONFIRMED_FLAG);

//   memset(tx_buffer, 0, sizeof(tx_buffer)); // clear buffer
//   txBufferPointer = 0;
}

/**
 * Receive a message from the Network Server
 */
static void receive_message() {
//   uint8_t port;
//   int flags;
//   int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

//   if (retcode < 0) {
//     printf("\r\n receive() - Error code %d \r\n", retcode);
//     return;
//   }

// #if DEBUGGING
//   printf(" RX Data on port %u (%d bytes): ", port, retcode);
//   for (uint8_t i = 0; i < retcode; i++) {
//     printf("%02x ", rx_buffer[i]);
//   }
//   printf("\r\n");
// #endif
//   // extract command byte
//   uint8_t cmdByte = rx_buffer[0] >> 4;
//   uint8_t additionalInfo = rx_buffer[0] & 0x0F;

//   switch (cmdByte) {
//   case 0x05: {
// #if DEBUGGING
//     printf("setting uplink interval \r\n");
// #endif
//     uint16_t receivedInterval = (rx_buffer[1] << 8) | rx_buffer[2];
//     if (additionalInfo & 0x01) {
//       // time sent in minutes
//       sleepIntervalMs = receivedInterval * 60 * 1000;
//     } else {
//       sleepIntervalMs = receivedInterval * 1000;
//     }
//     break;
//   }
//   default: {
// #if DEBUGGING
//     printf(" unrecognised command: %02x", cmdByte);
//     printf("\r\n");
// #endif
//   }
//   };

//   memset(rx_buffer, 0, sizeof(rx_buffer)); // clear RX buffer
}

static void enterSleep() {

  // here rx windows have passed and we have latest
  // sleep interval
  thread_sleep_for(sleepIntervalMs);

  // here we wake up
  send_message();
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event) {
  switch (event) {
  case CONNECTED: {
    printf("\r\n Connection - Successful \r\n");

    // we connected, send our first sensor reading or error message
    if (nodeOk) {
      send_message();
    } else {
      send_error_message();
    }
    break;
  }
  case DISCONNECTED:
    ev_queue.break_dispatch();
    printf("\r\n Disconnected Successfully \r\n");
    break;
  case TX_DONE:
    // this is where we land after TX is done
    // printf("\r\n Message Sent to Network Server \r\n");

    // enter sleep
    // TODO: we should call this message only after RX window
    if (nodeOk) {
      ev_queue.call_in(6000, enterSleep); // wait for RX to pass
    } else {
      while (true) {
        // Nothing happens!
      }
    }
    break;
  case TX_TIMEOUT:
  case TX_ERROR:
  case TX_CRYPTO_ERROR:
  case TX_SCHEDULING_ERROR:
    printf("\r\n Transmission Error - EventCode = %d \r\n", event);
    // try again
    if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
      send_message();
    }
    break;
  case RX_DONE:
    // printf("\r\n Received message from Network Server \r\n");
    receive_message();
    break;
  case RX_TIMEOUT:
    printf("RX TIMEOUT");
    break;
  case RX_ERROR:
    printf("\r\n Error in reception - Code = %d \r\n", event);
    break;
  case JOIN_FAILURE:
    printf("\r\n OTAA Failed - Check Keys \r\n");
    break;
  case UPLINK_REQUIRED:
    printf("\r\n Uplink required by NS \r\n");
    if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
      send_message();
    }
    break;
  default:
    MBED_ASSERT("Unknown Event");
  }
}
// EOF
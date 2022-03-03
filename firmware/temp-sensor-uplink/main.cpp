/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "events/EventQueue.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "mbed.h"
#include <cstdint>
#include <stdio.h>

// Application helpers
// #include "trace_helper.h"
#include "helpers.h"

// create  I2C instance
I2C i2c(PA_11, PA_12);
// I2C slave addreses
const uint8_t tempSensorAddr[5] = {0x1A, 0x1C, 0x1B, 0x19, 0x18};

DigitalOut sensorEN(PA_7);
DigitalOut I2CExpanderEN(PB_3);

#include "STM32WL_LoRaRadio.h"
STM32WL_LoRaRadio radio;

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t txBufferPointer = 0;
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when
 * Duty Cycling is off for testing
 */
#define TX_TIMER 10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS 10

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

// I2C low level functions
static uint8_t readReg(uint8_t addr, uint8_t reg) {
  char regAddr[1] = {reg};
  char data[1];

  i2c.write(addr, regAddr, 1, true);

  i2c.read(addr, data, 1, false);

  i2c.stop();

  return data[0];
}

static uint16_t readReg16(uint8_t addr, uint8_t reg) {

  char regAddr[1] = {reg};
  char data[2];

  i2c.write(addr, regAddr, 1, true);

  i2c.read(addr, data, 2, false);

  i2c.stop();

  return (data[0] << 8 | data[1]);
}

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  char msg[2] = {reg, value};
  return i2c.write(addr, msg, 2, false);
}

static bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value) {
  uint8_t MSB = value >> 8;
  uint8_t LSB = value & 0xFF;
  char msg[3] = {reg, MSB, LSB};

  return i2c.write(addr, msg, 3, false);
}

// MCP9808 reading
/**
    sets mode of the sensor
    @param state    true - enter shutdown mode
                    false - sampling mode
*/
static void sensorSleep(uint8_t addr, bool state) {
  const uint8_t placeSHDN = 0x08;
  const uint8_t regCONFIG = 0x01;
  uint16_t currentCONFIG = readReg16(addr, regCONFIG);
  // printf("Current Config: "BYTE_TO_BINARY_PATTERN "
  // "BYTE_TO_BINARY_PATTERN"\n",
  // BYTE_TO_BINARY(currentCONFIG>>8),BYTE_TO_BINARY(currentCONFIG));
  uint16_t CONFIG =
      (currentCONFIG & ~(1UL << placeSHDN)) | (state << placeSHDN);
  // printf("Set CONFIG: "BYTE_TO_BINARY_PATTERN " "BYTE_TO_BINARY_PATTERN"\n",
  // BYTE_TO_BINARY(CONFIG>>8),BYTE_TO_BINARY(CONFIG));
  writeReg16(addr, regCONFIG, CONFIG);
}

static void setResolution(uint8_t addr, uint8_t resolution) {
  const uint8_t regRES = 0x08;

  // no need to read resolution because it is the only setting in register
  writeReg(addr, regRES, resolution);
}

static float getTemp(uint8_t addr) {
  const uint8_t regAMBTEMP = 0x05;

  uint16_t regValue = readReg16(addr, regAMBTEMP);
  printf("ABIENT TEMP reg: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN
         "\n",
         BYTE_TO_BINARY(regValue >> 8), BYTE_TO_BINARY(regValue));

  /**
      temperature registr contains 3 sections
      [0:11]  - Ambient Temp Bits
      [12]    - Sign bit
      [13-15] - Alert cause
  */
  uint16_t maskALRTBits = 0xE000;
  uint16_t maskNegativeBit = 0x1000;
  uint16_t maskTempBits = 0xFFF;

  uint8_t alerCauseBits = regValue & maskALRTBits;
  bool negativeTemp = regValue & maskNegativeBit;

  // remove alert cause bits and negative temp bit
  uint16_t tempBits = regValue & maskTempBits;
  // printf("tempBits: "BYTE_TO_BINARY_PATTERN " "BYTE_TO_BINARY_PATTERN"\n",
  // BYTE_TO_BINARY(tempBits>>8),BYTE_TO_BINARY(tempBits));

  uint8_t tempUpper = tempBits >> 8;
  // printf("upper: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempUpper));
  uint8_t tempLower = tempBits & 0xFF;
  // printf("lower: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempLower));

  float temp;
  if (negativeTemp) {
    temp = (256 - (tempUpper * 16 + ((float)tempLower / 16)));
  } else {
    temp = (tempUpper * 16 + ((float)tempLower / 16));
  }

  return temp;
}

/**
 * Entry point for application
 */
int main(void) {

  // test routine
  // i2c.free();  //release I2C for deep sleep
  // I2CExpanderEN =0; //disable I2C expander
  // sensorEN = 0; //disable power for sensors
  // while(true){
  //     I2CExpanderEN =1;
  //     sensorEN = 1;
  //     i2c.init();
  //     sensorSleep(tempSensorAddr[0] << 1, false);
  //     printf("%2.4f \r\n",getTemp(tempSensorAddr[0] <<1));
  //     sensorSleep(tempSensorAddr[0] << 1, true);
  //     i2c.free();
  //     sensorEN = 0;
  //     I2CExpanderEN = 0;
  //     thread_sleep_for(1000);
  // }

  // setup tracing
  // setup_trace();

  // as per low-power suggestion
  mbed_file_handle(STDIN_FILENO)->enable_input(false);

  // put all sensor in sleep mode
  I2CExpanderEN = 1; // enable I2C expander
  sensorEN = 1;      // enable  power for sensors

  for (uint8_t sensorAddr : tempSensorAddr) {
    printf("Sensor: %d \n", (sensorAddr));
    uint8_t addr8Bit = sensorAddr << 1;
    sensorSleep(addr8Bit, true);
  }

  i2c.free();        // release I2C for deep sleep
  I2CExpanderEN = 0; // disable I2C expander
  sensorEN = 0;      // disable power for sensors

  // stores the status of a call to LoRaWAN protocol
  lorawan_status_t retcode;

  // Initialize LoRaWAN stack
  if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
    printf("\r\n LoRa initialization failed! \r\n");
    return -1;
  }

  printf("\r\n Mbed LoRaWANStack initialized \r\n");

  // prepare application callbacks
  callbacks.events = mbed::callback(lora_event_handler);
  lorawan.add_app_callbacks(&callbacks);

  // Set number of retries in case of CONFIRMED messages
  if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER) !=
      LORAWAN_STATUS_OK) {
    printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
    return -1;
  }

  printf("\r\n CONFIRMED message retries : %d \r\n",
         CONFIRMED_MSG_RETRY_COUNTER);

  // Enable adaptive data rate
  if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
    printf("\r\n enable_adaptive_datarate failed! \r\n");
    return -1;
  }

  printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

  retcode = lorawan.connect();

  if (retcode == LORAWAN_STATUS_OK ||
      retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
  } else {
    printf("\r\n Connection error, code = %d \r\n", retcode);
    return -1;
  }

  printf("\r\n Connection - In Progress ...\r\n");

  // make your event queue dispatching events forever
  ev_queue.dispatch_forever();

  return 0;
}

// /**
//  * Sends a message to the Network Server
//  */
static void send_message() {

  I2CExpanderEN = 1; // enable I2C expander
  sensorEN = 1;      // enable sensor power
  i2c.init();        // init I2C

  // read all sensors
  for (uint8_t sensId = 0; sensId < sizeof(tempSensorAddr); sensId++) {
    printf("Sensor: %d \n", (tempSensorAddr[sensId]));
    uint8_t addr8Bit = tempSensorAddr[sensId] << 1;

    sensorSleep(addr8Bit, false);
    thread_sleep_for(300);
    float temp = getTemp(addr8Bit);
    sensorSleep(addr8Bit, true);

    // this is so that we never have negative values and use as much bits as
    // necessary, not more
    uint8_t tempOffset = 40;                           //(273.15-233.15);
    uint16_t tempOffseted = (temp + tempOffset) * 100; //*100 to have 2 decimals

    printf("%2.4f \r\n", temp);
    printf("%d \r\n", tempOffseted);
    
    uint8_t idByte = (sensId << 4) | 0x00;
    uint8_t tempDataUpper = tempOffseted >> 8;
    uint8_t tempDataLower = tempOffseted & 0xFF;

    printf("id Byte " BYTE_TO_BINARY_PATTERN "\r\n", BYTE_TO_BINARY(idByte));
    printf("temp data " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN "\r\n",
           BYTE_TO_BINARY(tempDataUpper), BYTE_TO_BINARY(tempDataLower));

    //fill tx buffer
    tx_buffer[txBufferPointer] = idByte;
    tx_buffer[txBufferPointer+1] = tempDataUpper;
    tx_buffer[txBufferPointer+2] = tempDataLower;
    txBufferPointer = txBufferPointer + 3; //set ready for the next player. 
  }

  i2c.free();        // release I2C
  sensorEN = 0;      // disable sensor power
  I2CExpanderEN = 0; // disable I2C expander

  /**
  NOTES for encoding
  each sensor node data start with idByte

  bit 0:4 - data type:
      0x00 - soil temperature
      0x01 -
      0x02 -
      0x03 -
      0x04 -
      0x05 -
      0x06 -
      0x07 -
      0x08 -
      0x09 -
      0x0A -
      0x0B -
      0x0C -
      0x0D -
      0x0E -
      0x0F -

  bit 5:7 - sensor level
      0x00 - closes to the bottom
      0x0F - closes to the top
      always start by using 0x00 and then move to the top
      as different sensor can be easly added above the surface.
  **/

  
  int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, txBufferPointer,
                         MSG_UNCONFIRMED_FLAG);

  if (retcode < 0) {
    retcode == LORAWAN_STATUS_WOULD_BLOCK
        ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

    if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
      // retry in 3 seconds
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        ev_queue.call_in(3000, send_message);
      }
    }
    return;
  }

  printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
  memset(tx_buffer, 0, sizeof(tx_buffer)); // clear buffer
  txBufferPointer =0;
}

/**
 * Receive a message from the Network Server
 */
static void receive_message() {
  uint8_t port;
  int flags;
  int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

  if (retcode < 0) {
    printf("\r\n receive() - Error code %d \r\n", retcode);
    return;
  }

  printf(" RX Data on port %u (%d bytes): ", port, retcode);
  for (uint8_t i = 0; i < retcode; i++) {
    printf("%02x ", rx_buffer[i]);
  }
  printf("\r\n");

  memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event) {
  switch (event) {
  case CONNECTED:
    printf("\r\n Connection - Successful \r\n");
    if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
      send_message();
    } else {
      ev_queue.call_every(TX_TIMER, send_message);
    }

    break;
  case DISCONNECTED:
    ev_queue.break_dispatch();
    printf("\r\n Disconnected Successfully \r\n");
    break;
  case TX_DONE:
    printf("\r\n Message Sent to Network Server \r\n");
    printf("Entering sleep\r\n");

    thread_sleep_for(20000);
    printf("Wake-up\r\n");
    send_message();

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
    printf("\r\n Received message from Network Server \r\n");
    receive_message();
    break;
  case RX_TIMEOUT:
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

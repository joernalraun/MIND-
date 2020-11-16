/*
 * firmata.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Administrator
 */
#if 1

#ifndef INC_FIRMATA_H_
#define INC_FIRMATA_H_
#include "stm32f1xx_hal.h"
#include "BMI160.h"
#include "HuskyLens.h"
#include "math.h"

#define     FIRMATA_MAJOR             0x02
#define     FIRMATA_MINOR             0x06

#define     DIGITAL_MESSAGE           0x90
#define     ANALOG_MESSAGE            0xE0
#define     REPORT_ANALOG             0xC0
#define     REPORT_DIGITAL            0xD0

#define     SET_PIN_MODE              0xF4
#define     SET_DIGITAL_PIN_VALUE     0xF5

#define     REPORT_VERSION            0xF9
#define     SYSTEM_RESET              0xFF

#define     START_SYSEX               0xF0
#define     END_SYSEX                 0xF7
#define     SYSEX_I2C_REPLY           0x77 // same as I2C_REPLY


#define     KEEP_ALIVE                0x50 //keep alive message
#define     RU_THERE                  0x51 // Poll Request For Boards Presence
#define     I_AM_HERE                 0x52 // Response to RU_THERE
#define     TONE_DATA                 0x5F // request to play a tone
#define     SERIAL_DATA               0x60 // communicate with serial devices, including other boards
#define     ENCODER_DATA              0x61 // reply with encoders current positions
#define     SONAR_CONFIG              0x62 // sonar configuration request
#define     SONAR_DATA                0x63 // sonar data reply
#define     DHT_CONFIG                0x64
#define     DHT_DATA                  0x65
#define     SERVO_CONFIG              0x70 // set max angle, minPulse, maxPulse, freq
#define     STRING_DATA               0x71 // a string message with 14-bits per char
#define     STEPPER_DATA              0x72 // control a stepper motor
#define     ONEWIRE_DATA              0x73 // send an OneWire read/write/reset/select/skip/search request
#define     SHIFT_DATA                0x75 // a bitstream to/from a shift register
#define     I2C_REQUEST               0x76 // send an I2C read/write request
#define     I2C_REPLY                 0x77 // a reply to an I2C read request
#define     I2C_CONFIG                0x78 // config I2C settings such as delay times and power pins
#define     REPORT_FIRMWARE           0x79 // report name and version of the firmware
#define     EXTENDED_ANALOG           0x6F // analog write (PWM, Servo, etc) to any pin
#define     PIN_STATE_QUERY           0x6D // ask for a pin's current mode and value
#define     PIN_STATE_RESPONSE        0x6E // reply with pin's current mode and value
#define     CAPABILITY_QUERY          0x6B // ask for supported modes and resolution of all pins
#define     CAPABILITY_RESPONSE       0x6C // reply with supported modes and resolution
#define     ANALOG_MAPPING_QUERY      0x69 // ask for mapping of analog to pin numbers
#define     ANALOG_MAPPING_RESPONSE   0x6A // reply with mapping info
#define     SAMPLING_INTERVAL         0x7A // set the poll rate of the main loop
#define     SCHEDULER_DATA            0x7B // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
#define     SYSEX_NON_REALTIME        0x7E // MIDI Reserved for non-realtime messages
#define     SYSEX_REALTIME            0x7F // MIDI Reserved for realtime messages

#define     MAX_DATA_BYTES          64

#define    PIN_MODE_INPUT                  0x00 // same as INPUT defined in Arduino.h
#define    PIN_MODE_OUTPUT                 0x01 // same as OUTPUT defined in Arduino.h
#define    PIN_MODE_ANALOG                 0x02 // analog pin in analogInput mode
#define    PIN_MODE_PWM                    0x03 // digital pin in PWM output mode
#define    PIN_MODE_SERVO                  0x04 // digital pin in Servo output mode
#define    PIN_MODE_SHIFT                  0x05 // shiftIn/shiftOut mode
#define    PIN_MODE_I2C                    0x06 // pin included in I2C setup
#define    PIN_MODE_ONEWIRE                0x07 // pin configured for 1-wire
#define    PIN_MODE_STEPPER                0x08 // pin configured for stepper motor
#define    PIN_MODE_ENCODER                0x09 // pin configured for rotary encoders
#define    PIN_MODE_SERIAL                 0x0A // pin configured for serial communication
#define    PIN_MODE_PULLUP                 0x0B // enable internal pull-up resistor for pin
#define    PIN_MODE_SONAR                  0x0C // pin configured for HC-SR04
#define    PIN_MODE_TONE                   0x0D // pin configured for tone
#define    PIN_MODE_PIXY                   0x0E // pin configured for pixy spi
#define    PIN_MODE_DHT                    0x0F // pin configured for DHT
#define    PIN_MODE_IGNORE                 0x7F // pin configured to be ignored by digitalWrite and capabilityResponse
#define    TOTAL_PIN_MODES                 17

//DFRobot私有协议
#define DFROBOT_MESSAGE             0x0D
#define SUB_MESSAGE_DFROBOT_REPORTS 0x0A
//陀螺仪
#define SUB_MESSAGE_GYROSCOPE       0x00
#define GYROSCOPE_BEGIN 			0x00
#define GYROSCOPE_GET_MESSAGE 		0x01

#define GYROSCOPE_GET_STEP 			0x00
#define GYROSCOPE_GET_X 			0x01
#define GYROSCOPE_GET_Y 			0x02
#define GYROSCOPE_GET_Z 			0x03

//哈士奇
#define SUB_MESSAGE_HUKSYLENS       0x01
#define HUKSYLENS_CHANGE_MODE		0x00
#define HUKSYLENS_REQUEST			0x01
#define HUKSYLENS_GET_TOTAL_ID		0x02
#define HUKSYLENS_ISAPPEARDIRECT	0x03
#define HUKSYLENS_CENTER_ID			0x04
#define HUKSYLENS_ID_LEARN			0x05
#define HUKSYLENS_ISAPPEAR			0x06
#define HUKSYLENS_GET_NUM_PARAMETER 0x07
#define HUKSYLENS_READCOUNT			0x08
#define HUKSYLENS_ID_NUM_PARAMETER  0x09
#define HUKSYLENS_LEARN_ONECE       0x0A
#define HUKSYLENS_FORGET_LEARN      0x0B
#define HUKSYLENS_WRITE_NAME		0x0C
#define HUKSYLENS_WRITEOSD			0x0D
#define HUKSYLENS_CLEAR_SCREEN		0x0E
#define HUKSYLENS_TAKEPHOTOTOSDCARD 0x0F
#define HUKSYLENS_SAVEMODELTOTFCARD 0x10

#define SUB_MESSAGE_MOTOR			0x02

#define SUB_MESSAGE_CHANGE_KEYBOARD 0x03

#define    TOTAL_PINS               2//PA6,PA7
uint8_t pinConfig[TOTAL_PINS];

//变量
uint8_t parsingSysex;
size_t sysexBytesRead;
size_t waitForData;
uint8_t executeMultiByteCommand;
uint8_t multiByteChannel;
uint8_t dataBufferSize;
uint8_t dataBuffer[MAX_DATA_BYTES];
uint8_t allowBufferUpdate;
uint8_t analogInputsToReport = 0;//用来存储哪些模拟口需要上报的，以位判断，每一位代表一个模拟口

char *myname = "DFRobot firmata\0";

uint8_t key_up[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t a_down[8]={0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
uint8_t d_down[8]={0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00};

uint8_t w_down[8]={0x00,0x00,0x1a,0x00,0x00,0x00,0x00,0x00};
uint8_t s_down[8]={0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00};

uint8_t j_down[8]={0x00,0x00,0xD,0x00,0x00,0x00,0x00,0x00};
uint8_t k_down[8]={0x00,0x00,0xE,0x00,0x00,0x00,0x00,0x00};
uint8_t l_down[8]={0x00,0x00,0xF,0x00,0x00,0x00,0x00,0x00};
uint8_t i_down[8]={0x00,0x00,0xC,0x00,0x00,0x00,0x00,0x00};
uint8_t o_down[8]={0x00,0x00,0x12,0x00,0x00,0x00,0x00,0x00};
uint8_t p_down[8]={0x00,0x00,0x13,0x00,0x00,0x00,0x00,0x00};
uint8_t space_down[8]={0x00,0x00,0x2c,0x00,0x00,0x00,0x00,0x00};

typedef void (*dataBufferOverflowCallbackFunction)(void * context);
dataBufferOverflowCallbackFunction currentDataBufferOverflowCallback;
void * currentDataBufferOverflowCallbackContext;

void firmata_processSysexMessage(void);
uint8_t firmata_bufferDataAtPosition(const uint8_t data, const size_t pos);
void firmata_parse(uint8_t* inputData,uint8_t datalen);

#endif /* INC_FIRMATA_H_ */

#endif

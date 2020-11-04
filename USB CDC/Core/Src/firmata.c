/*
 * fiamta.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Administrator
 */
#include "firmata.h"
#if 1
static void reoprt_sysex(uint8_t command, uint8_t argc, uint8_t *argv){
	uint8_t mode = 0;
	switch(command){
		case RU_THERE:
			break;
		case I2C_REQUEST:
			break;
		case DFROBOT_MESSAGE:
			mode = argv[0];
			float acceleration=0.0;
			uint8_t Accbuf[5]={0xf0,0x0d,0x00,0x00,0xf7};
			uint8_t symbol=0;//0:正；1：负
			uint8_t data=0;
			uint8_t stepbuf[7]={0xf0,0x0d,0x00,0x00,0x00,0x00,0xf7};
			uint32_t step=0;
			uint8_t addr=0;
			switch(mode){
				case SUB_MESSAGE_GYROSCOPE:
					if(argv[1]==GYROSCOPE_BEGIN){
						if(argv[3]==0)
							addr=0x68;
						else if(argv[3]==1)
							addr=0x69;
						else
							;
						begin(argv[2],addr);
					}else if(argv[1]==GYROSCOPE_GET_MESSAGE){
						switch(argv[2]){
							case 0:
								acceleration=getAccX();
								if(acceleration<0)
									symbol=1;
								data=(uint8_t)(fabs(acceleration)*100);
								Accbuf[2]=symbol;
								Accbuf[3]=data;
								CDC_Transmit_FS(Accbuf,5);
								break;
							case 1:
								acceleration=getAccY();
								if(acceleration<0)
									symbol=1;
								data=(uint8_t)(fabs(acceleration)*100);
								Accbuf[2]=symbol;
								Accbuf[3]=data;
								CDC_Transmit_FS(Accbuf,5);
								break;
							case 2:
								acceleration=getAccZ();
								if(acceleration<0)
									symbol=1;
								data=(uint8_t)(fabs(acceleration)*100);
								Accbuf[2]=symbol;
								Accbuf[3]=data;
								CDC_Transmit_FS(Accbuf,5);
								break;
							case 3:
								step=getstep();
								stepbuf[3]=(uint8_t)(step>>24);
								stepbuf[4]=(uint8_t)(step>>16);
								stepbuf[5]=(uint8_t)(step>>8);
								stepbuf[2]=(uint8_t)step;
								CDC_Transmit_FS(stepbuf,7);
								break;
							default:
								break;
						}
					}else{

					}
					break;
				case SUB_MESSAGE_HUKSYLENS:
					break;
				case SUB_MESSAGE_I2CSCAN:
					break;
				default:
					break;
			}
			break;
		default :
			break;
	}
}

static void sendFirmwareVersion(uint8_t major, uint8_t minor, size_t bytec, uint8_t *bytev){
	uint8_t msg[5]={START_SYSEX,REPORT_FIRMWARE,major,minor,END_SYSEX};
	CDC_Transmit_FS(msg,5);
}

/**
 * 上报版本信息
 */
static void reoprt_firmware(){
	const size_t major_version_offset = 1;
	const size_t minor_version_offset = 2;
	const size_t string_offset = 3;
	if(sysexBytesRead < 3){/*错误格式，用于Firmata V3.0.0 */
		sendFirmwareVersion(FIRMATA_MAJOR, FIRMATA_MINOR, strlen(myname), (uint8_t *)myname);
	}
}

uint8_t getPinMode(uint8_t pin){
	if((pin<6) || (pin>7))
		return 255;
	return pinConfig[pin-6];
}

/**
 * 根据命令设置IO口的模式
 */
static void firmata_setPinMode(uint8_t pin, int mode){
	if(getPinMode(pin) == PIN_MODE_IGNORE){
		return;
	}
	if((pin<6) || (pin>7))
		return ;
	pinConfig[pin-6]=mode;
}


/************************************************************************/
/*                       数字口相关操作                                  */
/************************************************************************/

/**
 * 解析处理firmata协议包里的数字口的操作
 * Port 0 = pins D0 - D7, port 1 = pins D8 - D15
 */

/**
 * 解析处理firmata协议包里的数字口的操作，对应pymata4的digital_pin_write
 */
static void firmata_setDigitalPinValue(uint8_t pin, int value){
	if(getPinMode(pin) == PIN_MODE_OUTPUT){
		switch(pin){
			case 6:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,value);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,value);
				break;
			default:
				break;
		}

	}
}

/************************************************************************/
/*                          模拟口相关操作                               */
/************************************************************************/



static void testReportAnalog(){
	uart_write(0xe2);
	uart_write(0x17);
	uart_write(0x05);
}


/**
 * 设置哪些模拟口需要上报数据
 */
static void firmata_setReportAnalog(uint8_t analogPin, int value){
}

/**
 * 解析处理firmata协议包里的模拟口的操作
 * pin =(0~5),对应A0~A5
 */
static void firmata_analogWrite(uint8_t pin, uint8_t duty,int freq){
	if(pin < TOTAL_PINS){
		switch(getPinMode(pin)){
			case PIN_MODE_SERVO:
				break;
			case PIN_MODE_PWM:
				break;
			}
		}
}

/**
 * 根据firmata协议打包模拟量数据并上传
 */
void firmata_sendAnalog(uint8_t pin, uint16_t value){
	if((0xF >= pin) && (0x3FFF >= value)){
		uart_write(ANALOG_MESSAGE|pin);
		encodeByteStream(sizeof(value),(uint8_t *)&value,sizeof(value));
	}else{
		uart_write(0xFF);
		uart_write(0xFF);
		uart_write(0xFF);
	}
}

/**
 * 从dataBuffer中解析接收到的完整数据包
 */
void firmata_processSysexMessage(void){
	switch(dataBuffer[0]){ /*第一位是命令位*/
		case REPORT_FIRMWARE:
			reoprt_firmware();
			break;
		case STRING_DATA:
			break;
		default:
			reoprt_sysex(dataBuffer[0], sysexBytesRead - 1, dataBuffer + 1);
			break;
	}
}

void firmata_parse(uint8_t* inputData,uint8_t datalen){
	uint8_t command;
	if(inputData[0] < 0xF0){
		command = inputData[0] & 0xF0;
		multiByteChannel = inputData[0] & 0x0F;
	}else{
		command = inputData[0];
	}
	switch(command){
		case ANALOG_MESSAGE:
		case DIGITAL_MESSAGE:
		case SET_PIN_MODE:
			firmata_setPinMode(inputData[1],inputData[2]);
			break;
		case SET_DIGITAL_PIN_VALUE:
			firmata_setDigitalPinValue(inputData[1],inputData[2]);
			break;
		case REPORT_ANALOG:
		case REPORT_DIGITAL:
			break;
		case START_SYSEX:/*接收到的数据为0xF0,代表协议包头，开始传输*/
			if(END_SYSEX!=inputData[datalen-1])
				return ;
			for(int i=1;i<datalen-1;i++)
				dataBuffer[i-1]=inputData[i];
			sysexBytesRead=datalen-2;
			firmata_processSysexMessage();
			break;
		case SYSTEM_RESET:
			break;
		case REPORT_VERSION:
			break;
	}
}
#endif

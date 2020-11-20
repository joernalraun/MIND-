/*
 * fiamta.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Administrator
 */
#include "firmata.h"
#include "string.h"
#include <DFRobot_usbd_cdc_if.h>
#if 1
static void reoprt_sysex(uint8_t command, uint8_t argc, uint8_t *argv){
	uint8_t mode = 0;
	uint8_t txbuffer[50]={0xf0,0x0d,0};
	int16_t TOTAL_ID;
	uint8_t ISAPPEARDIRECT;
	uint8_t isLearn;
	uint8_t ISAPPEAR;
	uint8_t ID;
	uint8_t count;
	uint8_t color_count;
	char rename[50];
	uint8_t screen_count;
	char screen[50];
	HUSKYLENSBlockDirectInfo CENTER_Block_DATA;
	HUSKYLENSArrowDirectInfo CENTER_Arrow_DATA;
	HUSKYLENSBlockInfo BI;
	HUSKYLENSArrowInfo AI;
	int CT;
	int x ;
	int y ;
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
								DFR_CDC_Transmit_FS(Accbuf,5);
								break;
							case 1:
								acceleration=getAccY();
								if(acceleration<0)
									symbol=1;
								data=(uint8_t)(fabs(acceleration)*100);
								Accbuf[2]=symbol;
								Accbuf[3]=data;
								DFR_CDC_Transmit_FS(Accbuf,5);
								break;
							case 2:
								acceleration=getAccZ();
								if(acceleration<0)
									symbol=1;
								data=(uint8_t)(fabs(acceleration)*100);
								Accbuf[2]=symbol;
								Accbuf[3]=data;
								DFR_CDC_Transmit_FS(Accbuf,5);
								break;
							case 3:
								step=getstep();
								stepbuf[3]=(uint8_t)(step>>24);
								stepbuf[4]=(uint8_t)(step>>16);
								stepbuf[5]=(uint8_t)(step>>8);
								stepbuf[2]=(uint8_t)step;
								DFR_CDC_Transmit_FS(stepbuf,7);
								break;
							default:
								break;
						}
					}else{

					}
					break;
				case SUB_MESSAGE_HUKSYLENS:
					switch(argv[1]){
						case(HUKSYLENS_CHANGE_MODE):
							writeAlgorithm(argv[2]);
							break;
						case(HUKSYLENS_REQUEST):
							request();
							break;
						case(HUKSYLENS_GET_TOTAL_ID):
							TOTAL_ID=readLearnedIDCount();
							txbuffer[2]=0x01;
							txbuffer[3]=0x02;
							txbuffer[4]=TOTAL_ID;
							txbuffer[5]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,6);
							break;
						case(HUKSYLENS_ISAPPEARDIRECT):
							ISAPPEARDIRECT=isAppearDirect(argv[2]);
							txbuffer[2]=0x01;
							txbuffer[3]=0x03;
							if(ISAPPEARDIRECT>0)
								txbuffer[4]=1;
							else
								txbuffer[4]=0;
							txbuffer[5]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,6);
							break;
						case(HUKSYLENS_CENTER_ID):
							if(argv[2]==0){
								CENTER_Block_DATA=readBlockCenterParameterDirect();
							}else if(argv[2]==1){
								readArrowCenterParameterDirect(&CENTER_Arrow_DATA);
							}else{
								;
							}
							txbuffer[2]=0x01;
							txbuffer[3]=0x04;
							switch(argv[3]){
								case 0:
									if(argv[2]==0)
										CT=CENTER_Block_DATA.xCenter;
									else if(argv[2]==1)
										CT=CENTER_Arrow_DATA.xOrigin;
									else
										;
									break;
								case 1:
									if(argv[2]==0)
										CT=CENTER_Block_DATA.yCenter;
									else if(argv[2]==1)
										CT=CENTER_Arrow_DATA.yOrigin;
									else
										;
									break;
								case 2:
									if(argv[2]==0)
										CT=CENTER_Block_DATA.width;
									else if(argv[2]==1)
										CT=CENTER_Arrow_DATA.xTarget;
									else
										;
									break;
								case 3:
									if(argv[2]==0)
										CT=CENTER_Block_DATA.height;
									else if(argv[2]==1)
										CT=CENTER_Arrow_DATA.yTarget;
									else
										;
									break;
								case 4:
									if(argv[2]==0)
										CT=CENTER_Block_DATA.ID;
									else if(argv[2]==1)
										CT=CENTER_Arrow_DATA.ID;
									else
										;
									break;
								default:
									break;
							}
							txbuffer[4]=(uint8_t)(CT>>24);
							txbuffer[5]=(uint8_t)(CT>>16);
							txbuffer[6]=(uint8_t)(CT>>8);
							txbuffer[7]=(uint8_t)(CT);
							txbuffer[8]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,9);
							break;
						case(HUKSYLENS_ID_LEARN):
							isLearn=isLearned(argv[2]);
							txbuffer[2]=0x01;
							txbuffer[3]=0x05;
							if(isLearn>0)
								txbuffer[4]=1;
							else
								txbuffer[4]=0;
							txbuffer[5]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,6);
							break;
						case(HUKSYLENS_ISAPPEAR):
							ISAPPEAR=isAppear(argv[2],argv[3]);
							txbuffer[2]=0x01;
							txbuffer[3]=0x06;
							if(ISAPPEAR>0)
								txbuffer[4]=1;
							else
								txbuffer[4]=0;
							txbuffer[5]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,6);
							break;
						case(HUKSYLENS_GET_NUM_PARAMETER):
							ID=argv[2];
							if(argv[3]==0){
								BI=readBlockParameter(ID,1);
							}else if(argv[3]==1){
								AI=readArrowParameter(ID,1);
							}else{
								;
							}
							txbuffer[2]=0x01;
							txbuffer[3]=0x07;
							switch(argv[4]){
								case 0:
									if(argv[3]==0)
										CT=BI.xCenter;
									else if(argv[2]==1)
										CT=AI.xOrigin;
									else
										;
									break;
								case 1:
									if(argv[3]==0)
										CT=BI.yCenter;
									else if(argv[2]==1)
										CT=AI.yOrigin;
									else
										;
									break;
								case 2:
									if(argv[3]==0)
										CT=BI.yCenter;
									else if(argv[2]==1)
										CT=AI.xTarget;
									else
										;
									break;
								case 3:
									if(argv[3]==0)
										CT=BI.height;
									else if(argv[2]==1)
										CT=AI.yTarget;
									else
										;
									break;
								default:
									break;
							}
							txbuffer[4]=(uint8_t)(CT>>24);
							txbuffer[5]=(uint8_t)(CT>>16);
							txbuffer[6]=(uint8_t)(CT>>8);
							txbuffer[7]=(uint8_t)(CT);
							txbuffer[8]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,9);
							break;
						case(HUKSYLENS_READCOUNT):
							count=(uint8_t)(readALLCount(argv[2]));
							txbuffer[2]=0x01;
							txbuffer[3]=0x08;
							txbuffer[4]=count;
							txbuffer[5]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,6);
							break;
						case(HUKSYLENS_ID_NUM_PARAMETER):
							ID=argv[2];
							if(argv[3]==0)
								BI=readBlockParameter(ID,argv[4]);
							else if(argv[3]==1)
								AI=readArrowParameter(ID,argv[4]);
							txbuffer[2]=0x01;
							txbuffer[3]=0x09;
							switch(argv[5]){
								case 0:
									if(argv[3]==0)
										CT=BI.xCenter;
									else if(argv[2]==1)
										CT=AI.xOrigin;
									else
										;
									break;
								case 1:
									if(argv[3]==0)
										CT=BI.yCenter;
									else if(argv[2]==1)
										CT=AI.yOrigin;
									else
										;
									break;
								case 2:
									if(argv[3]==0)
										CT=BI.yCenter;
									else if(argv[2]==1)
										CT=AI.xTarget;
									else
										;
									break;
								case 3:
									if(argv[3]==0)
										CT=BI.height;
									else if(argv[2]==1)
										CT=AI.yTarget;
									else
										;
									break;
								default:
									break;
							}
							txbuffer[4]=(uint8_t)(CT>>24);
							txbuffer[5]=(uint8_t)(CT>>16);
							txbuffer[6]=(uint8_t)(CT>>8);
							txbuffer[7]=(uint8_t)(CT);
							txbuffer[8]=0xf7;
							DFR_CDC_Transmit_FS(txbuffer,9);
							break;
						case HUKSYLENS_LEARN_ONECE:
							ID=argv[2];
							learnOnece(ID);
							break;
						case HUKSYLENS_FORGET_LEARN:
							forgetLearn();
							break;
						case HUKSYLENS_WRITE_NAME:
							ID=argv[2];
							int i;
							color_count=argv[3];
							for(i=0;i<color_count;i++)
								rename[i]=(char)argv[i+4];
							rename[i+1]='\0';
							writeName(rename,ID);
							break;
						case HUKSYLENS_WRITEOSD:
							x = (argv[2]<<24) | (argv[3]<<16) | (argv[4]<<8) | argv[5];
							y = (argv[6]<<24) | (argv[7]<<16) | (argv[8]<<8) | argv[9];
							screen_count=argv[10];
							for(i=0;i<screen_count;i++){
								screen[i]=(char)argv[i+11];
							}
							screen[i]='\0';
							writeOSD(screen,x,y);
							break;
						case HUKSYLENS_CLEAR_SCREEN:
							clearOSD();
							break;
						case HUKSYLENS_TAKEPHOTOTOSDCARD:
							takePhotoToSDCard();
							break;
						case HUKSYLENS_SAVEMODELTOTFCARD:
							if(argv[2]==0)
								saveModelToTFCard(argv[3]);
							else if(argv[2]==1)
								loadModelFromTFCard(argv[3]);
							else
								;
							break;
						default:
							break;
					}
					break;
				case SUB_MESSAGE_MOTOR:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,argv[1]);
					break;
				case SUB_MESSAGE_CHANGE_KEYBOARD:
					if(argv[1]==0x00){
						a_down[2]=0x04;
						d_down[2]=0x07;
						w_down[2]=0x1a;
						s_down[2]=0x16;
						j_down[2]=0x0d;
						k_down[2]=0x0e;
						l_down[2]=0x0f;
						i_down[2]=0x0c;
						o_down[2]=0x12;
						p_down[2]=0x13;
					}else if(argv[1]==0x01){
						a_down[2]=0x50;
						d_down[2]=0x4f;
						w_down[2]=0x52;
						s_down[2]=0x51;
						j_down[2]=0x27;
						k_down[2]=0x1e;
						l_down[2]=0x1f;
						i_down[2]=0x20;
						o_down[2]=0x21;
						p_down[2]=0x22;
					}else{
						;
					}
					break;
				default:
					break;
			}
			break;
		default :
			break;
	}
}

void sendFirmwareVersion(uint8_t major, uint8_t minor, size_t bytec, uint8_t *bytev){
	uint8_t msg[5]={START_SYSEX,REPORT_FIRMWARE,major,minor,END_SYSEX};
	DFR_CDC_Transmit_FS(msg,5);
}

/**
 * 上报版本信息
 */
static void reoprt_firmware(){
//	const size_t major_version_offset = 1;
//	const size_t minor_version_offset = 2;
//	const size_t string_offset = 3;
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

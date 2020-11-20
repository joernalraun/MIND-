/*
 * HuskyLens.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Administrator
 */
#include "HuskyLens.h"
#include "stdlib.h"
#include "string.h"
Protocol_t protocolInfo;
Protocol_t *protocolPtr = NULL;
static uint8_t send_buffer[FRAME_BUFFER_SIZE];
static uint8_t receive_buffer[FRAME_BUFFER_SIZE];
static short send_index = 0;
static short receive_index = 0;

static uint8_t send_fail = 0;
static uint8_t receive_fail = 0;

static short content_current = 0;
static short content_end = 0;
static uint8_t content_read_end = 0;
HUSKYLENSResult resultDefault;

PROTOCOL_CREATE(Request, Command, COMMAND_REQUEST)
PROTOCOL_CREATE(RequestBlocks, Command, COMMAND_REQUEST_BLOCKS)
PROTOCOL_CREATE(RequestArrows, Command, COMMAND_REQUEST_ARROWS)
PROTOCOL_CREATE(RequestLearned, Command, COMMAND_REQUEST_LEARNED)
PROTOCOL_CREATE(RequestBlocksLearned, Command, COMMAND_REQUEST_BLOCKS_LEARNED)
PROTOCOL_CREATE(RequestArrowsLearned, Command, COMMAND_REQUEST_ARROWS_LEARNED)
PROTOCOL_CREATE(RequestKnock, Command, COMMAND_REQUEST_KNOCK)
PROTOCOL_CREATE(ReturnOK, Command, COMMAND_RETURN_OK)
PROTOCOL_CREATE(RequestClearText, Command, COMMAND_REQUEST_CLEAR_TEXT)
PROTOCOL_CREATE(RequestTakePhotoToSDCard, Command, COMMAND_REQUEST_TAKE_PHOTO_TO_SD_CARD)
PROTOCOL_CREATE(RequestScreenshotToSDCard, Command, COMMAND_REQUEST_SCREENSHOT_TO_SD_CARD)
PROTOCOL_CREATE(RequestForgetLearn, Command, COMMAND_REQUEST_FORGET)
PROTOCOL_CREATE(RequestSaveModelToTFCard, BufferUint8, COMMAND_REQUEST_SAVE_MODEL_TO_SD_CARD)
PROTOCOL_CREATE(RequestLoadModelFromTFCard, BufferUint8, COMMAND_REQUEST_LOAD_MODEL_FROM_SD_CARD)

PROTOCOL_CREATE(RequestByID, OneInt16, COMMAND_REQUEST_BY_ID)
PROTOCOL_CREATE(RequestBlocksByID, OneInt16, COMMAND_REQUEST_BLOCKS_BY_ID)
PROTOCOL_CREATE(RequestArrowsByID, OneInt16, COMMAND_REQUEST_ARROWS_BY_ID)
PROTOCOL_CREATE(RequestAlgorithm, OneInt16, COMMAND_REQUEST_ALGORITHM)

PROTOCOL_CREATE(ReturnInfo, FiveInt16, COMMAND_RETURN_INFO)
PROTOCOL_CREATE(ReturnBlock, FiveInt16, COMMAND_RETURN_BLOCK)
PROTOCOL_CREATE(ReturnArrow, FiveInt16, COMMAND_RETURN_ARROW)

PROTOCOL_CREATE(RequestCustomText, BufferUint8, COMMAND_REQUEST_CUSTOM_TEXT)
PROTOCOL_CREATE(RequestName, BufferUint8, COMMAND_REQUEST_NAME)
PROTOCOL_CREATE(RequestLearnOnece, BufferUint8, COMMAND_REQUEST_LEARN_ONECE)
PROTOCOL_CREATE(RequestFirmwareVersion, BufferUint8, COMMAND_REQUEST_FIRMWARE_VERSION)

uint8_t writeAlgorithm(enum protocolAlgorithm algorithmType){
	Protocol_t protocol;
	protocol.algorithmType = algorithmType ;
	protocolWriteRequestAlgorithm(&protocol);//protocolWriteOneInt16
	//return wait(COMMAND_RETURN_OK);
	return 1;
}

uint8_t request(){
	Protocol_t protocol;
    protocolWriteRequest(&protocol);
	HAL_Delay(100);
    return processReturn();
}

uint8_t processReturn(){
    if (!wait(COMMAND_RETURN_INFO))
        return 0;
    protocolReadReturnInfo(&protocolInfo);
    protocolPtr = (Protocol_t *)realloc(protocolPtr, max(protocolInfo.protocolSize, 1) * sizeof(Protocol_t));
    for (int i = 0; i < protocolInfo.protocolSize; i++){
        if (!wait(0))
            return 0;
        if (protocolReadReturnBlock(&protocolPtr[i]))
            continue;
        else if (protocolReadReturnArrow(&protocolPtr[i]))
            continue;
        else
            return 0;
    }
    return 1;
}

uint8_t wait(uint8_t command){
	//return 1;
    while (1){
        if (protocolAvailable()){
            if (command){
                if (husky_lens_protocol_read_begin(command))
                    return 1;
            }else{
                return 1;
            }
        }
    }
    return 0;
}

uint8_t husky_lens_protocol_read_begin(uint8_t command){
    if (command == receive_buffer[COMMAND_INDEX]){
        content_current = CONTENT_INDEX;
        content_read_end = 0;
        receive_fail = 0;
        return 1;
    }
    return 0;
}

uint8_t protocolAvailable(){
	uint8_t buf[16];
	int i;
	uint8_t result;
	HAL_I2C_Master_Receive(&hi2c2,0x32<<1,buf,16,1000);
	for(i=0;i<16;i++){
		result=buf[i];
		if (husky_lens_protocol_receive(result))
			return 1;
	}
	return 0;
}

uint8_t validateCheckSum(){
    uint8_t stackSumIndex=receive_buffer[CONTENT_SIZE_INDEX] + CONTENT_INDEX;
    uint8_t sum = 0;
    for (uint8_t i=0; i< stackSumIndex; i++){
        sum+=receive_buffer[i];
    }
    return (sum == receive_buffer[stackSumIndex]);
}

uint8_t husky_lens_protocol_receive(uint8_t data){
    switch (receive_index){
    case HEADER_0_INDEX:
        if (data!=0x55) {receive_index = 0; return 0;}
        receive_buffer[HEADER_0_INDEX] = 0x55;
        break;
    case HEADER_1_INDEX:
        if (data!=0xaa) {receive_index = 0; return 0;}
        receive_buffer[HEADER_1_INDEX] = 0xaa;
        break;
    case ADDRESS_INDEX:
        receive_buffer[ADDRESS_INDEX] = data;
        break;
    case CONTENT_SIZE_INDEX:
        if (data >= FRAME_BUFFER_SIZE-PROTOCOL_SIZE) {receive_index = 0; return 0;}
        receive_buffer[CONTENT_SIZE_INDEX] = data;
        break;
    default:
        receive_buffer[receive_index]=data;
        if (receive_index==receive_buffer[CONTENT_SIZE_INDEX]+CONTENT_INDEX) {
            content_end = receive_index;
            receive_index=0;
            return validateCheckSum();
        }
        break;
    }
    receive_index++;
    return 0;
}

void protocolWriteCommand(Protocol_t* protocol, uint8_t command){
     protocol->command = command;
     uint8_t *buffer = husky_lens_protocol_write_begin(protocol->command);
     int length = husky_lens_protocol_write_end();
     protocolWrite(buffer, length);
 }

uint8_t protocolReadCommand(Protocol_t* protocol, uint8_t command){
     if (husky_lens_protocol_read_begin(command))
     {
         protocol->command = command;
         husky_lens_protocol_read_end();
         return 1;
     }
     else
     {
         return 0;
     }
}

void protocolWrite(uint8_t *buffer, int length){
	HAL_I2C_Master_Transmit(&hi2c2,0x32<<1,buffer,length,1000);
}

int husky_lens_protocol_write_end(){
    if(send_fail) {return 0;}
    if(send_index + 1 >= FRAME_BUFFER_SIZE) {return 0;}
    send_buffer[CONTENT_SIZE_INDEX] = send_index - CONTENT_INDEX;
    uint8_t sum = 0;
    for (int i = 0; i < send_index; i++)
    {
        sum += send_buffer[i];
    }
    send_buffer[send_index] = sum;
    send_index ++;
    return send_index;
}

uint8_t* husky_lens_protocol_write_begin(uint8_t command){
     send_fail = 0;
     send_buffer[HEADER_0_INDEX] = 0x55;
     send_buffer[HEADER_1_INDEX] = 0xAA;
     send_buffer[ADDRESS_INDEX] = 0x11;
     send_buffer[COMMAND_INDEX] = command;
     send_index = CONTENT_INDEX;
     return send_buffer;
 }

 void protocolWriteFiveInt16(Protocol_t* protocol, uint8_t command){
     protocol->command = command;
     uint8_t *buffer = husky_lens_protocol_write_begin(protocol->command);
     husky_lens_protocol_write_int16(protocol->first);
     husky_lens_protocol_write_int16(protocol->second);
     husky_lens_protocol_write_int16(protocol->third);
     husky_lens_protocol_write_int16(protocol->fourth);
     husky_lens_protocol_write_int16(protocol->fifth);
     int length = husky_lens_protocol_write_end();
     protocolWrite(buffer, length);
 }

 uint8_t protocolReadFiveInt16(Protocol_t* protocol, uint8_t command){
     if (husky_lens_protocol_read_begin(command))
     {
         protocol->command = command;
         protocol->first = husky_lens_protocol_read_int16();
         protocol->second = husky_lens_protocol_read_int16();
         protocol->third = husky_lens_protocol_read_int16();
         protocol->fourth = husky_lens_protocol_read_int16();
         protocol->fifth = husky_lens_protocol_read_int16();
         husky_lens_protocol_read_end();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 uint8_t husky_lens_protocol_read_end(){
     if (receive_fail)
     {
         receive_fail = 0;
         return 0;
     }
     return content_current == content_end;
 }

 int16_t husky_lens_protocol_read_int16(){
     if (content_current >= content_end || content_read_end){receive_fail = 1; return 0;}
     int16_t result;
     memcpy(&result, receive_buffer + content_current, sizeof(result));
//     if (IS_BIG_ENDIAN()){__builtin_bswap16(result);}
     content_current += sizeof(result);
     return result;
 }

 void protocolWriteOneInt16(Protocol_t* protocol, uint8_t command){
     protocol->command = command;
     uint8_t *buffer = husky_lens_protocol_write_begin(protocol->command);
     husky_lens_protocol_write_int16(protocol->first);
     int length = husky_lens_protocol_write_end();
     protocolWrite(buffer, length);
 }

 void husky_lens_protocol_write_int16(int16_t content){
     if(send_index + sizeof(content) >= FRAME_BUFFER_SIZE) {send_fail = 1; return;}
//     if (IS_BIG_ENDIAN()){__builtin_bswap16(content);}
     memcpy(send_buffer + send_index, &content, sizeof(content));
     send_index += sizeof(content);
 }

 uint8_t protocolReadOneInt16(Protocol_t* protocol, uint8_t command){
     if (husky_lens_protocol_read_begin(command))
     {
         protocol->command = command;
         protocol->first = husky_lens_protocol_read_int16();
         husky_lens_protocol_read_end();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 void protocolWriteBufferUint8(Protocol_t* protocol, uint8_t command){
     protocol->command = command;
     uint8_t *buffer = husky_lens_protocol_write_begin(protocol->command);
     husky_lens_protocol_write_buffer_uint8(protocol->data, protocol->length);
     int length = husky_lens_protocol_write_end();
     protocolWrite(buffer, length);
 }

 void husky_lens_protocol_write_buffer_uint8(uint8_t *content, uint32_t length){
     if(send_index + sizeof(uint8_t) * length >= FRAME_BUFFER_SIZE) {send_fail = 1; return;}
     memcpy(send_buffer + send_index, content, sizeof(uint8_t) * length);
     send_index += sizeof(uint8_t) * length;
 }

 uint8_t protocolReadBufferUint8(Protocol_t* protocol, uint8_t command){
     if (husky_lens_protocol_read_begin(command))
     {
         protocol->command = command;
         protocol->length = husky_lens_protocol_read_buffer_uint8(protocol->data, protocol->length);
         husky_lens_protocol_read_end();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 uint32_t husky_lens_protocol_read_buffer_uint8(uint8_t *content, uint32_t length){
     if (content_current >= content_end || content_read_end){receive_fail = 1; return 0;}
     memcpy(content, receive_buffer + content_current, sizeof(uint8_t)*length);
     content_current += sizeof(uint8_t)*length;
     return length;
 }

 //从结果中获取已学习的ID数
 int16_t readLearnedIDCount(){
	 return protocolInfo.knowledgeSize;
 }

//屏幕叠加显示文字
 uint8_t writeOSD(const char* text, int x, int y){
     Protocol_t protocol;
     uint8_t textLength = strlen(text);
     uint8_t data[20]={textLength, 0, x, y};
     //uint8_t data[textLength + 5] = {textLength, 0, x, y};
     if (x > 255){
         data[1] = 0xff;
         data[2] = x % 256;
     }
     for(int i=0;i<textLength + 1;i++){
    	 data[i+4]=(int)text[i];
     }
     protocol.length = textLength + 4;
     protocol.data = data;
     protocolWriteRequestCustomText(&protocol);
     return wait(COMMAND_RETURN_OK);
 }

 //清除现在屏幕上显示的东西
 uint8_t clearOSD(){
	 Protocol_t protocol;
     protocolWriteRequestClearText(&protocol);
     return wait(COMMAND_RETURN_OK);
 }

//从结果中获取（方框，箭头）是否在画面中
uint8_t isAppearDirect(HUSKYLENSResultType type){
	 switch (type){
	 case HUSKYLENSResultBlock:
		 return countBlocks();
	 case HUSKYLENSResultArrow:
		 return countArrows();
	 default:
		 return 0;
	 }
}

//哈士奇获取靠近中心的参数
HUSKYLENSBlockDirectInfo readBlockCenterParameterDirect(){
    int32_t distanceMin = INT32_MAX;
    int16_t distanceMinIndex = -1;
    for (int i = 0; i < blocksavailable(); i++){
        HUSKYLENSResult resultBuffer = blocksreadDirect(i);
        int32_t distance = sq(resultBuffer.xCenter - 320 / 2) + sq(resultBuffer.yCenter - 240 / 2);
        if (distance < distanceMin)
        {
            distanceMin = distance;
            distanceMinIndex = i;
        }
    }

    HUSKYLENSResult result = blocksreadDirect(distanceMinIndex);
    HUSKYLENSBlockDirectInfo block;
    block.xCenter = result.xCenter;
    block.yCenter = result.yCenter;
    block.width = result.width;
    block.height = result.height;
    block.ID = result.ID;
    return block;
}

void readArrowCenterParameterDirect(HUSKYLENSArrowDirectInfo*arrow){
    int32_t distanceMin = INT32_MAX;
    int16_t distanceMinIndex = -1;
    for (int i = 0; i <available(); i++){
        HUSKYLENSResult resultBuffer = readDirect(i);
        int32_t distance = sq((resultBuffer.xOrigin + resultBuffer.xTarget) / 2 - 320 / 2) + sq((resultBuffer.yOrigin + resultBuffer.yTarget) / 2 - 240 / 2);
        if (distance < distanceMin)
        {
            distanceMin = distance;
            distanceMinIndex = i;
        }
    }
    HUSKYLENSResult result = readDirect(distanceMinIndex);
//    HUSKYLENSArrowDirectInfo arrow;
    arrow->xOrigin = result.xOrigin;
    arrow->yOrigin = result.yOrigin;
    arrow->xTarget = result.xTarget;
    arrow->yTarget = result.yTarget;
    arrow->ID = result.ID;
//    return arrow;
}

int available(){
    return countArrows();
}


HUSKYLENSResult blocksreadDirect(int index){
	return readBlockDirect(index);
}

HUSKYLENSResult readBlockDirect(int index){
    Protocol_t *protocol = readBlockProtocol(index);
    return protocol ? *protocol : resultDefault;
}

Protocol_t *readBlockProtocol(int16_t index){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++)
    {
        if (protocolPtr[i].command == COMMAND_RETURN_BLOCK)
            if (index == counter++)
                return protocolPtr + i;
    }
    return NULL;
}

int blocksavailable(){
	return countBlocks();
}

int16_t countBlocks(){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++){
        if (protocolPtr[i].command == COMMAND_RETURN_BLOCK)
            counter++;
    }
    return counter;
}

//哈士奇从结果中获取ID。。是否已学习
uint8_t isLearned(int ID){

    return (ID <= countLearnedIDs()) && ID > 0;
}

int16_t countLearnedIDs(){
    return protocolInfo.knowledgeSize;
}

//哈士奇从结果中获取ID是否在画面中

uint8_t isAppear(int ID, HUSKYLENSResultType type){
    switch (type){
    case HUSKYLENSResultBlock:
        return countIDBlocks(ID);
    case HUSKYLENSResultArrow:
        return countIDArrows(ID);
    default:
        return 0;
    }
}

int16_t countArrows(){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++){
        if (protocolPtr[i].command == COMMAND_RETURN_ARROW)
            counter++;
    }
    return counter;
}

//哈士奇从结果中获取ID的参数

HUSKYLENSBlockInfo readBlockParameter(int ID, int index){
    HUSKYLENSResult result = read(ID, index - 1);
    HUSKYLENSBlockInfo block;
    block.xCenter = result.xCenter;
    block.yCenter = result.yCenter;
    block.width = result.width;
    block.height = result.height;
    return block;
}
HUSKYLENSResult read(int ID, int index){
    return readBlock(ID, index);
}

HUSKYLENSResult readBlock(int ID , int index){
    Protocol_t *protocol = readBlockByIDProtocol(ID, index);
    return protocol ? *protocol : resultDefault;
}

Protocol_t *readBlockByIDProtocol(int16_t ID, int16_t index){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++)
    {
        if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID == ID)
            if (index == counter++)
                return protocolPtr + i;
    }
    return NULL;
}

HUSKYLENSArrowInfo readArrowParameter(int ID, int index){
    HUSKYLENSResult result = arrowsread(ID, index - 1);
    HUSKYLENSArrowInfo arrow;
    arrow.xOrigin = result.xOrigin;
    arrow.yOrigin = result.yOrigin;
    arrow.xTarget = result.xTarget;
    arrow.yTarget = result.yTarget;
    return arrow;
}

HUSKYLENSResult arrowsread(int ID, int index){
    Protocol_t *protocol = readBlockByIDProtocol(ID, index);
    return protocol ? *protocol : resultDefault;
}

//哈士奇从结果中获取方框总数
float readALLCount(HUSKYLENSResultType type){
    switch (type){
    case HUSKYLENSResultBlock:
        return countBlocks();
    case HUSKYLENSResultArrow:
        return countArrows();
    default:
        return -1.0f;
    }
}

//哈士奇从结果中获取第。。个方框的参数
HUSKYLENSBlockDirectInfo readBlockParameterDirect(int index){
    HUSKYLENSResult result = readDirect(index - 1);
    HUSKYLENSBlockDirectInfo block;
    block.xCenter = result.xCenter;
    block.yCenter = result.yCenter;
    block.width = result.width;
    block.height = result.height;
    block.ID = result.ID;
    return block;
}

HUSKYLENSResult readDirect(int index){
    return readBlockDirect(index);
}

//哈士奇获取ID（）的方框总数
float readCount(int ID, HUSKYLENSResultType type){
    switch (type){
		case HUSKYLENSResultBlock:
			return countIDBlocks(ID);
		case HUSKYLENSResultArrow:
			return countIDArrows(ID);
		default:
			return -1.0f;
    }
}
int16_t countIDBlocks(int16_t ID){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++)
    {
        if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID == ID)
            counter++;
    }
    return counter;
}
int16_t countIDArrows(int16_t ID){
    int16_t counter = 0;
    for (int i = 0; i < protocolInfo.protocolSize; i++)
    {
        if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID == ID)
            counter++;
    }
    return counter;
}

//遗忘现在已经学习的全部数据
uint8_t forgetLearn(){
	Protocol_t protocol;
    protocolWriteRequestForgetLearn(&protocol);
    return wait(COMMAND_RETURN_OK);
}

//哈士奇自动学习一次
uint8_t learnOnece(uint16_t id){
    uint8_t data[] = {id & 0xff, (id >> 8) & 0xff};
    Protocol_t protocol;
    protocol.length = 2;
    protocol.data = data;
    protocolWriteRequestLearnOnece(&protocol);
    return wait(COMMAND_RETURN_OK);
}

//给当前算法的当前ID改名字
uint8_t writeName(const char*name, uint8_t id){
    Protocol_t protocol;
    uint8_t nameLength = strlen(name);
    uint8_t data[100] = {id, (nameLength + 1) * 2};
    for(int i=0;i<nameLength;i++){
    	data[i+2]=name[i];
    }
    protocol.length = nameLength + 3;
    protocol.data = data;
    protocolWriteRequestName(&protocol);
    return wait(COMMAND_RETURN_OK);
}
//拍照
uint8_t takePhotoToSDCard(){
	Protocol_t protocol;
    protocolWriteRequestTakePhotoToSDCard(&protocol);
    return wait(COMMAND_RETURN_OK);
}

//保存/加载到SD卡
uint8_t saveModelToTFCard(uint16_t index){
    uint8_t data[] = {index & 0xff, (index >> 8) & 0xff};
    Protocol_t protocol;
    protocol.length = 2;
    protocol.data = data;
    protocolWriteRequestSaveModelToTFCard(&protocol);
    return wait(COMMAND_RETURN_OK);
}

uint8_t loadModelFromTFCard(uint16_t index){
    uint8_t data[] = {index & 0xff, (index >> 8) & 0xff};
    Protocol_t protocol;
    protocol.length = 2;
    protocol.data = data;
    protocolWriteRequestLoadModelFromTFCard(&protocol);
    return 0;
}

//通信部分
void readreg(uint8_t iic_addr,uint8_t reg_addr, uint8_t *data, uint16_t len){
	  HAL_I2C_Master_Transmit(&hi2c2,iic_addr<<1,&reg_addr,1,1000);
	  HAL_Delay(10);
	  HAL_I2C_Master_Receive(&hi2c2,iic_addr<<1,data,len,1000);
}

void writereg(uint8_t iic_addr,uint8_t reg_addr, uint8_t *data, uint16_t len){
	  uint8_t buf[255]={0};
	  buf[0]=reg_addr;
	  for(int i=1;i<len+1;i++)
		  buf[i]=data[i-1];
	  HAL_I2C_Master_Transmit(&hi2c2,iic_addr<<1,buf,len+1,1000);
}

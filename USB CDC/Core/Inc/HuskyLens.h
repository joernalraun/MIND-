/*
 * HuskyLens.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Administrator
 */

#ifndef INC_HUSKYLENS_H_
#define INC_HUSKYLENS_H_

#include "stm32f1xx_hal.h"
#include "math.h"

#define FRAME_BUFFER_SIZE 128
#define HEADER_0_INDEX      0
#define HEADER_1_INDEX      1
#define ADDRESS_INDEX       2
#define CONTENT_SIZE_INDEX  3
#define COMMAND_INDEX       4
#define CONTENT_INDEX       5
#define PROTOCOL_SIZE       6

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define sq(x) ((x)*(x))

#define IS_BIG_ENDIAN() (!*(uint8_t *)&(uint16_t){1})

static uint8_t send_buffer[FRAME_BUFFER_SIZE];
static uint8_t receive_buffer[FRAME_BUFFER_SIZE];

static short send_index = 0;
static short receive_index = 0;

static uint8_t send_fail = 0;
static uint8_t receive_fail = 0;

static short content_current = 0;
static short content_end = 0;
static uint8_t content_read_end = 0;

I2C_HandleTypeDef hi2c2;

#define PROTOCOL_CREATE(function, type, command)       \
    void protocolWrite##function(Protocol_t* protocol) \
    {                                                  \
        protocolWrite##type(protocol, command);        \
    }                                                  \
    uint8_t protocolRead##function(Protocol_t*protocol)  \
    {                                                  \
        return protocolRead##type(protocol, command);  \
    }

static uint8_t send_buffer[FRAME_BUFFER_SIZE];
static uint8_t receive_buffer[FRAME_BUFFER_SIZE];

enum protocolAlgorithm{
    ALGORITHM_FACE_RECOGNITION=0,
    ALGORITHM_OBJECT_TRACKING,
    ALGORITHM_OBJECT_RECOGNITION,
    ALGORITHM_LINE_TRACKING,
    ALGORITHM_COLOR_RECOGNITION,
    ALGORITHM_TAG_RECOGNITION,
    ALGORITHM_OBJECT_CLASSIFICATION,
    ALGORITHM_QR_RECOGNITION,
    ALGORITHM_BARCODE_RECOGNITION
};

enum protocolCommand
{
    COMMAND_REQUEST = 0x20,
    COMMAND_REQUEST_BLOCKS,
    COMMAND_REQUEST_ARROWS,
    COMMAND_REQUEST_LEARNED,
    COMMAND_REQUEST_BLOCKS_LEARNED,
    COMMAND_REQUEST_ARROWS_LEARNED,
    COMMAND_REQUEST_BY_ID,
    COMMAND_REQUEST_BLOCKS_BY_ID,
    COMMAND_REQUEST_ARROWS_BY_ID,
    COMMAND_RETURN_INFO,
    COMMAND_RETURN_BLOCK,
    COMMAND_RETURN_ARROW,
    COMMAND_REQUEST_KNOCK,
    COMMAND_REQUEST_ALGORITHM,
    COMMAND_RETURN_OK,
    COMMAND_REQUEST_NAME,
    COMMAND_REQUEST_TAKE_PHOTO_TO_SD_CARD = 0x30,
    COMMAND_REQUEST_SAVE_MODEL_TO_SD_CARD = 0x32,
    COMMAND_REQUEST_LOAD_MODEL_FROM_SD_CARD = 0x33,
    COMMAND_REQUEST_CUSTOM_TEXT = 0x34,
    COMMAND_REQUEST_CLEAR_TEXT = 0x35,
    COMMAND_REQUEST_LEARN_ONECE = 0x36,
    COMMAND_REQUEST_FORGET = 0x37,
    COMMAND_REQUEST_SCREENSHOT_TO_SD_CARD = 0x39,
    COMMAND_REQUEST_FIRMWARE_VERSION = 0x3C
};

typedef struct{
    uint8_t command;
    union {
        int16_t first;
        int16_t xCenter;
        int16_t xOrigin;
        int16_t protocolSize;
        int16_t algorithmType;
        int16_t requestID;
    };
    union {
        int16_t second;
        int16_t yCenter;
        int16_t yOrigin;
        int16_t knowledgeSize;
    };
    union {
        int16_t third;
        int16_t width;
        int16_t xTarget;
        int16_t frameNum;
    };
    union {
        int16_t fourth;
        int16_t height;
        int16_t yTarget;
    };
    union {
        int16_t fifth;
        int16_t ID;
        int16_t length;
    };
    union {
        uint8_t *data;
    };
} Protocol_t;
typedef Protocol_t HUSKYLENSResult;

typedef enum HUSKYLENSResultType_t
{
    HUSKYLENSResultBlock,
    HUSKYLENSResultArrow,
} HUSKYLENSResultType;

typedef struct HUSKYLENSBlockDirectInfo_t
{
    int32_t xCenter;
    int32_t yCenter;
    int32_t width;
    int32_t height;
    int32_t ID;
} HUSKYLENSBlockDirectInfo;

typedef struct HUSKYLENSArrowDirectInfo_t
{
    int32_t xOrigin;
    int32_t yOrigin;
    int32_t xTarget;
    int32_t yTarget;
    int32_t ID;
} HUSKYLENSArrowDirectInfo;

typedef struct HUSKYLENSBlockInfo_t
{
    int32_t xCenter;
    int32_t yCenter;
    int32_t width;
    int32_t height;
} HUSKYLENSBlockInfo;

typedef struct HUSKYLENSArrowInfo_t
{
    int32_t xOrigin;
    int32_t yOrigin;
    int32_t xTarget;
    int32_t yTarget;
} HUSKYLENSArrowInfo;
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

uint8_t writeAlgorithm(enum protocolAlgorithm algorithmType);
uint8_t wait(uint8_t command);
uint8_t protocolAvailable();
uint8_t validateCheckSum();
uint8_t husky_lens_protocol_receive(uint8_t data);

void protocolWriteCommand(Protocol_t* protocol, uint8_t command);
uint8_t protocolReadCommand(Protocol_t* protocol, uint8_t command);
void protocolWriteFiveInt16(Protocol_t* protocol, uint8_t command);
uint8_t protocolReadFiveInt16(Protocol_t* protocol, uint8_t command);
void protocolWriteOneInt16(Protocol_t* protocol, uint8_t command);
uint8_t protocolReadOneInt16(Protocol_t* protocol, uint8_t command);
void protocolWriteBufferUint8(Protocol_t* protocol, uint8_t command);
uint8_t protocolReadBufferUint8(Protocol_t* protocol, uint8_t command);
uint8_t husky_lens_protocol_read_begin(uint8_t command);
uint8_t processReturn();
uint8_t* husky_lens_protocol_write_begin(uint8_t command);
uint8_t protocolReadCommand(Protocol_t* protocol, uint8_t command);
void protocolWrite(uint8_t *buffer, int length);
uint8_t* husky_lens_protocol_write_begin(uint8_t command);
void protocolWriteFiveInt16(Protocol_t* protocol, uint8_t command);
int16_t husky_lens_protocol_read_int16();
uint8_t husky_lens_protocol_read_end();
void husky_lens_protocol_write_buffer_uint8(uint8_t *content, uint32_t length);
uint8_t isAppearDirect(HUSKYLENSResultType type);
int16_t countBlocks();
int16_t countArrows();
HUSKYLENSBlockDirectInfo readBlockCenterParameterDirect();
HUSKYLENSResult blocksreadDirect(int index);
Protocol_t *readBlockProtocol(int16_t index);
HUSKYLENSResult readBlockDirect(int index);
HUSKYLENSBlockDirectInfo readBlockCenterParameterDirect();
int blocksavailable();
uint8_t isLearned(int ID);
int16_t countLearnedIDs();
uint8_t isAppear(int ID, HUSKYLENSResultType type);
HUSKYLENSBlockDirectInfo readBlockParameterDirect(int index);
HUSKYLENSResult readDirect(int index);
HUSKYLENSBlockInfo readBlockParameter(int ID, int index);
HUSKYLENSResult read(int ID, int index);
HUSKYLENSResult readBlock(int ID , int index);
Protocol_t *readBlockByIDProtocol(int16_t ID, int16_t index);
float readALLCount(HUSKYLENSResultType type);
float readCount(int ID, HUSKYLENSResultType type);
int16_t countIDBlocks(int16_t ID);
int16_t countIDArrows(int16_t ID);
uint8_t forgetLearn();
uint8_t learnOnece(uint16_t id);
uint8_t writeName(const char*name, uint8_t id);
uint8_t takePhotoToSDCard();
uint8_t saveModelToTFCard(uint16_t index);
HUSKYLENSArrowInfo readArrowParameter(int ID, int index);
HUSKYLENSResult arrowsread(int ID, int index);
uint8_t loadModelFromTFCard(uint16_t index);

void readreg(uint8_t iic_addr,uint8_t reg_addr, uint8_t *data, uint16_t len);
void writereg(uint8_t iic_addr,uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif /* INC_HUSKYLENS_H_ */

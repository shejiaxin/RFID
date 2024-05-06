#ifndef __RF_CTR_DECODE_H
#define __RF_CTR_DECODE_H

#include "includes.h"
#include "stm32f10x.h"

#define RFDATA_BIT_NUM_MAX                 24      //RF一帧数据包含的数据位数
#define RFDATA_BYTE_NUM_MAX                 3      //RF一帧数据包含的数据字节数

//遥控器数据结构
typedef struct tagRemKey_DataStruct
{   
    uint8_t  RfBufData[RFDATA_BYTE_NUM_MAX+1];
    #ifdef RCV_DATA_1527_MSB
    uint16_t KeyAddress;  //四个按键的地址
    #else
    uint32_t KeyAddress;  //五个按键的地址
    #endif
    
    uint8_t  PairingMode;  //用于配对模式 
    uint8_t  PairingFlag;  //是否配对标志，配对成功为1，否则为0
    
    uint8_t  lockValue;    //设防键键值
    uint8_t  unlockValue;  //解防键键值
    uint8_t  startValue;   //一键启动键键值
    uint8_t  hornValue;    //寻车键键值
}RemKeyDataStruct;

extern RemKeyDataStruct  RemKeyData;

/*extern uint8_t lockOkCnt;
extern uint16_t lockOkDelay;
extern uint8_t lockFlag;*/

extern uint16_t StartKeyOkDelay;
extern uint8_t StartKeyFlag;

void RF_Decode_Init(void);
void RF_Recv_IO_Config(void);
void RemoteControlProc(uint8_t *pMsg);
void RF_Decode(void);
void SwitchSpkMode(void);
void RemoteOpenSeatLock(void);

#endif /* __RF_CTR_DECODE_H */


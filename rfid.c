
#include "includes.h"

#ifdef USE_RF_REMOTE

uint8_t CurRfData[RFDATA_BYTE_NUM_MAX];
uint8_t BufRfData[RFDATA_BYTE_NUM_MAX];

RemKeyDataStruct  RemKeyData;


#ifdef ENABLE_MEM_MANAGEMENT
extern OS_MEM UserBufPoolMem;
#endif
uint8_t *p_rfRx_blk = NULL;

extern OS_TCB    RFDecodeTaskTCB;

uint16_t dat[]={0,0,0,0,0,0,0,0,0};  //存放高低电平时间
uint8_t  rec_count; 
uint8_t  lo_buf; 
uint8_t  hi_buf;  
uint8_t  tempdata;

//RF接收标志 
uint8_t  rfstart1; 
uint8_t  rfstart; 
uint8_t  rf_er;
uint8_t  rfok;
uint8_t  rfokcnt;
 #ifdef RCV_DATA_1527_MSB
uint16_t keyadr;
#else
uint32_t keyadr=0;
#endif

#ifdef SPK_NV040_VOICE
/*uint8_t lockOkCnt = 0;  //设防键接收成功计数
uint16_t lockOkDelay = TWO_SECOND;
uint8_t lockFlag = FALSE;*/

uint8_t StartKeyOkCnt = 0;  //启动键接收成功计数
uint16_t StartKeyOkDelay = FIVE_SECOND;
uint8_t StartKeyFlag = FALSE;
#endif



/*
*********************************************************************************************************
*   函 数 名: RF_Decode_Init
*   功能说明: 遥控解码初始化
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
void RF_Decode_Init(void)
{
    TIM2_Config();         //TIM2初始化
    RF_Recv_IO_Config();   //遥控接收IO
    //Remote_Init();
    
    lo_buf   = 0;          //计数低电平持续时间
    hi_buf   = 0;          //计数高电平持续时间
    tempdata = 0;          //存放接收到的临时数据，一个字节
    rfstart  = 0;          //接收到同步位标志，=1表示接收到同步位
    rfstart1 = 0;          //开始解码标志，=1表示已经接收到同步位，可以开始解码地址位和数据位
    rfok     = 0;          //解码成功计数
    rf_er    = 0;          //解码错误标志
    rfokcnt  = 0;
    
    keyadr   = 0;
//    lockOkCnt = 0; 
    
    RemKeyData.PairingMode = FALSE; //默认非配对模式
    RemKeyData.PairingFlag = FALSE; 
}

/*
*********************************************************************************************************
*   函 数 名: RF_Recv_Config
*   功能说明: 配置PB1为遥控器接收
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
void RF_Recv_IO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /*开启GPIOC的外设时钟*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
    
    /* 配置PC1为浮空输入 PB1-->PC1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*   函 数 名: RemoteControlProc
*   功能说明: 遥控器接收处理程序，完成状态上报平台
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
//void RemoteControlProc(uint8_t *pMsg,uint8_t len)
void RemoteControlProc(uint8_t *pMsg)
{
    //static uint8_t startKeyCnt = 0;
    uint8_t * rxbuf = pMsg; 

    if(rxbuf == NULL) 
        return;
    
    /*if(CompareLeaseTime() == FALSE)
    {
        DEBUG_PRINT("=====[REM]Lease Time Compare Faile!=====\r\n");
        return;
    }*/
        
    #ifdef DEBUG_OUTPUT0 
    DEBUG_UART("\r\n====接收成功====");
    DEBUG_UART("\r\nAddress:0x%05X",rxbuf[3]<<16|rxbuf[2]<<8|rxbuf[1]);
    DEBUG_UART("\r\nRF data:0x%02X,qd=%02X",rxbuf[0],RemKeyData.startValue); 
    DEBUG_UART("\r\nhornValue:%02X",RemKeyData.hornValue);
    #endif
    
    if((RemKeyData.lockValue == rxbuf[0]) && 
        ((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) == 0))
    {
        LockCar(REM_CMD);
        //SaveCarFunctionState(); 
    }
    else if((RemKeyData.lockValue == rxbuf[0]) && (AlartLockedFlag == TRUE)
           && ((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0))
    {
        Disable_G_INT();
        OneWriteSendCmd(0xFE); //停止喇叭声音
        Enable_G_INT();
    }
    else if((RemKeyData.unlockValue == rxbuf[0])&&
           ((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0))
    {
        unLockCar(REM_CMD);
        //SaveCarFunctionState(); 
    }
    else if((RemKeyData.unlockValue == rxbuf[0])&&
           ((gCarFunctionState & FUNC_START_STOP_BIT) != 0))
    {
        OneKeyStop(REM_CMD);
        //SaveCarFunctionState(); 
    }
    else if((RemKeyData.startValue == rxbuf[0]) &&
           ((gCarFunctionState & FUNC_START_STOP_BIT) == 0))
    {
        //startKeyCnt++;
        //if(startKeyCnt >= 1)
        //{
        //    startKeyCnt = 0;
            
            OneKeyStart(REM_CMD);
            //SaveCarFunctionState(); 
        //}
    }
    else if((RemKeyData.hornValue == rxbuf[0]) 
         && ((gCarFunctionState & FUNC_START_STOP_BIT) == 0))
    {            
        HornCar(REM_CMD);
    }
}

/****************************************************
RF遥控器解码程序
1、支持315M、433M遥控器
2、实测过支持以下编码IC的遥控器
   a、HS2240(SOP8 1.8或2M振荡电阻)
   b、SCT1527(SOP8 270K振荡电阻)
   c、HS1528P(SOP8 OTP一次可编程设置参数及地址)
3、接收端采用超再生或超外差都可以，只要频率对应(315M或433M)
4、定时器78us中断一次调用此函数
*****************************************************/
void RF_Decode(void)
{   
    OS_ERR   err;
    
    if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1) //GPIOB-->GPIOC
    {  
        if(rfstart1 == 1)//是否找到同步头,如果找到同步头就由低到高开始接收 
        { 
            if(lo_buf > hi_buf)//判断是否低电平 
            {    
                //低电平范围:8*78=624us 20*78=1560us 高电平范围:78*3=234us  9*78=702us  
                if((lo_buf >= 8) && (lo_buf <= 20) && (hi_buf >= 3) && (hi_buf <= 9)) //8,16 | 3,9
                {  
                    dat[1] = lo_buf;
                    dat[2] = hi_buf;        
                    rec_count += 1;
                    
                    #ifdef RCV_DATA_1527_MSB
                    tempdata <<= 1;
                    #else
                    tempdata >>= 1;
                    #endif
                } 
                else
                {
                    rf_er = 1;//接收失败    
                }                   
            }            
            else if(lo_buf < hi_buf)//判断是否高电平 
            { 
                //低电平范围:78*3=234us  9*78=702us  高电平范围:8*78=624us 20*78=1560us
                if((lo_buf >= 3) && (lo_buf <= 9) && (hi_buf >= 8) && (hi_buf <= 20)) //3,9 | 8,16
                {                   
                    dat[3] = lo_buf;
                    dat[4] = hi_buf;    
                    rec_count += 1;//是高电平   
                    
                    #ifdef RCV_DATA_1527_MSB
                    tempdata <<= 1;//左移一位   从高bit往低bit 
                    tempdata += 1;
                    #else 
                    tempdata >>= 1;//左移一位   CMT2150A 从低bit往高bit 
                    tempdata |= 0x80;
                    #endif
                }  
                else
                {                   
                    rf_er = 1;
                }                   
             } 
             else if(lo_buf == hi_buf) 
             {
                rf_er = 1;  //高低电平周期相等，接收错误
             }
               
             if(rf_er == 1)
             { 
                rfstart = 0; 
                rec_count = 0; 
                rfok = 0; 
             } 
             else
             { 
                #ifdef RCV_DATA_1527_MSB
                if(rec_count == 8)
                    CurRfData[2] = tempdata;
                else if(rec_count == 16)
                    CurRfData[1] = tempdata;
                #else
                if(rec_count == 8)
                    CurRfData[1] = tempdata;
                else if(rec_count == 16)
                    CurRfData[2] = tempdata;
                #endif
             } 
             if(rec_count == 24)
             {
                  rfok = 1; 
                  rec_count = 0;
                  //TEST_IO_TOGGLE;             
                  
                  if(p_rfRx_blk != NULL)
                  {
                    OSMemPut ((OS_MEM  *)&UserBufPoolMem,
                              (void    *)p_rfRx_blk,
                              (OS_ERR  *)&err);
                  }
                  p_rfRx_blk = OSMemGet((OS_MEM      *)&UserBufPoolMem,
                                        (OS_ERR      *)&err);
                  
                  if((p_rfRx_blk == NULL) && (err==OS_ERR_MEM_NO_FREE_BLKS))
                  {
                      return ;
                  }
                  #ifdef RCV_DATA_1527_MSB  
                  p_rfRx_blk[0] = tempdata & 0x0F;
                  #else  
                  p_rfRx_blk[3] = tempdata & 0x0F;
                  p_rfRx_blk[0] = (tempdata>>4) & 0x0F;
                  #endif
                  p_rfRx_blk[1] = CurRfData[1];
                  p_rfRx_blk[2] = CurRfData[2];
                  
               #ifdef DEBUG_OUTPUT0 
                  //printf("\r\n");
                  DEBUG_PRINT("\r\n====Receive Success====");
                #ifdef RCV_DATA_1527_MSB
                  DEBUG_PRINT("\r\nAddress:0x%04X",p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
                #else
                  DEBUG_PRINT("\r\nAddress:0x%05X",p_rfRx_blk[3]<<16|p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
                #endif
                  DEBUG_PRINT("\r\nRF data:0x%02X",p_rfRx_blk[0]); 
               #endif
                  
                 #ifdef RCV_DATA_1527_MSB
                  keyadr = (p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
                 #else
                  keyadr = (p_rfRx_blk[3]<<16|p_rfRx_blk[2]<<8|p_rfRx_blk[1])& 0x000FFFFF;
                 #endif
                 
                 if(RemKeyData.PairingMode == TRUE) //配对模式
                 {
                      RemKeyData.PairingFlag = TRUE;
                      RemKeyData.KeyAddress = keyadr;
                 }
                  
                 if(RemKeyData.KeyAddress == keyadr)
                 {  
                      rfokcnt++;

                      /*if((RemKeyData.lockValue == p_rfRx_blk[0]) && ((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0))
                      {
                          if(lockOkCnt < 20)
                            lockOkCnt++;
                          
                          if(lockFlag == FALSE)
                          {
                              lockOkCnt = 0;
                              lockOkDelay = TWO_SECOND;  
                              lockFlag = TRUE;
                          }
                      }
                      else
                      {
                          lockFlag = FALSE;
                          lockOkCnt = 0;
                      }*/
     
                      if((RemKeyData.startValue == p_rfRx_blk[0]) && ((gCarFunctionState & FUNC_START_STOP_BIT) != 0))
                      {
                          if(StartKeyOkCnt < 50)
                            StartKeyOkCnt++;
                          
                          if(StartKeyFlag == FALSE)
                          {
                              StartKeyFlag = TRUE;
                              StartKeyOkCnt = 0;
                              StartKeyOkDelay = FIVE_SECOND;
                          }
                      }
                      else
                      {
                          StartKeyFlag = FALSE;
                          StartKeyOkCnt = 0;
                      }
                      #ifdef DEBUG_OUTPUT0
                      DEBUG_PRINT("\r\nrfokcnt:%d,lockOkCnt:%d",rfokcnt,lockOkCnt); 
                      #endif
                  }
                  else
                  {
                      rfokcnt = 0;
                  }
                  
                  if(rfokcnt >= 1) 
                  {
                      rfokcnt = 0;

                      #ifdef  BEEP_SOUND_PLAY
                      if(RemKeyData.lockValue == p_rfRx_blk[0])
                       //&&((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0))
                      {
                          Disable_G_INT();
                          OneWriteSendCmd(0xFE);
                          SoundPlayFlag = FALSE;
                          Enable_G_INT();
                      }
                      #else 
                      OSTaskQPost ((OS_TCB      *)&RFDecodeTaskTCB,
                                   (void        *)p_rfRx_blk,
                                   (OS_MSG_SIZE  )4,
                                   (OS_OPT       )OS_OPT_POST_FIFO,
                                   (OS_ERR      *)&err);
                      #endif //#ifdef  BEEP_SOUND_PLAY
                  }
             }
             hi_buf = 0;
                          
        } //if(rfstart1==1)函数结尾                  
        else if((lo_buf >= 100) && (lo_buf <= 190))
        {  
             dat[0] = lo_buf;
             rfstart = 1;                       
             rec_count = 0; 
        }
        
        hi_buf += 1;                                                                             
        lo_buf = 0; 
        rfstart1 = 0; 
        rf_er = 0;          //接收失败标志 
    } 
    else 
    {     
        //输入端口为低电
        lo_buf += 1;         
        if(rfstart == 1)   //是否接收到同步位
        {
            rfstart1 = 1;  //开始接收地址位+数据位 
        }
        else 
        {
            hi_buf = 0;
        }           
    } 

}

//=================================输入捕获方式接收解码=======================
//遥控器接收状态
//[7]:收到了引导码标志
//[6]:保留
//[5]:保留  
//[4]:标记上升沿是否已经被捕获                                 
//[3:0]:溢出计时器
//uint8_t  RmtSta = 0;       //接收状态  

//uint32_t High_cnt = 0;     //下降沿时计数器的值，保存的是高电平时长，单位us
//uint32_t Low_cnt = 0;      //上升沿时计数器的值，保存的是低电平时长，单位us
//uint32_t RmtRec = 0;       //接收到的数据(24bit)

//uint8_t  num = 0;
//uint32_t address = 0;      //地址码(20bit)
//uint8_t key_value = 0;     //按键值(1、2、4、8)

/*****************************************************************************
 Prototype    : key_rmote
 Description  : 处理遥控解码
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : Billy
    Modification : Created function

*****************************************************************************/
//uint8_t key_rmote(void)
//{
//    OS_ERR   err;
//    
//    #ifdef RCV_DATA_1527_MSB  
//    address = RmtRec >> 4;  //取得遥控码地址bit0--bit20
//    key_value = RmtRec & 0x0000000F;
//    #else
//    address = RmtRec>>8;
//    key_value = (address>>20) & 0x0F;
//    #endif
//    RmtRec = 0;
//    
//    //printf("Rmt_addr=%X,key=%d",address,key_value);  //Rmt_addr=112345,key=1
//    
//    if(p_rfRx_blk != NULL)
//    {
//        OSMemPut ((OS_MEM  *)&UserBufPoolMem,
//                  (void    *)p_rfRx_blk,
//                  (OS_ERR  *)&err);
//    }
//    p_rfRx_blk = OSMemGet((OS_MEM      *)&UserBufPoolMem,
//                          (OS_ERR      *)&err);
//    
//    if((p_rfRx_blk == NULL) && (err==OS_ERR_MEM_NO_FREE_BLKS))
//    {
//        return 0;
//    }
//                  
//    #ifdef RCV_DATA_1527_MSB  
//    p_rfRx_blk[0] = key_value;  //遥控键值  
//    p_rfRx_blk[1] = (address>>4) & 0xFF;  //地址码 FA8E7
//    p_rfRx_blk[2] = (address>>12) &0xFF;
//    #else  
//    p_rfRx_blk[0] = key_value;
//    p_rfRx_blk[1] = address & 0x0000FF;        //45
//    p_rfRx_blk[2] = (address>>8) & 0x0000FF;   //23
//    p_rfRx_blk[3] = (address>>16) & 0x00000F;  //0x12345中的01
//    #endif
//    

//    #ifdef DEBUG_OUTPUT0
//    //printf("\r\n");
//    DEBUG_PRINT("\r\n====Receive Success====");  //Address:0x123405
//    #ifdef RCV_DATA_1527_MSB
//    DEBUG_PRINT("\r\nAddress:0x%04X",p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
//    #else
//    DEBUG_PRINT("\r\nAddress:0x%05X",p_rfRx_blk[3]<<16|p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
//    #endif
//    DEBUG_PRINT("\r\nRF data:0x%02X",p_rfRx_blk[0]); 
//    #endif

//    #ifdef RCV_DATA_1527_MSB
//    keyadr = (p_rfRx_blk[2]<<8|p_rfRx_blk[1]);
//    #else
//    keyadr = (p_rfRx_blk[3]<<16|p_rfRx_blk[2]<<8|p_rfRx_blk[1])& 0x000FFFFF;
//    #endif
//    
//    
//    //printf("\r\nRmt_addr=%X,key=%d",keyadr,key_value);
//    if(RemKeyData.KeyAddress == keyadr)
//    {  
//        rfokcnt++;      
//        
//        if((RemKeyData.lockValue == p_rfRx_blk[0]) && ((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0))
//        {
//            lockOkCnt++;
//            if(lockFlag == FALSE)
//            {
//                lockOkCnt = 0;
//                lockOkDelay = TWO_SECOND;  
//                lockFlag = TRUE;
//            }
//        }
//        else
//            lockOkCnt = 0;

//        #ifdef DEBUG_OUTPUT0
//        DEBUG_PRINT("\r\nrfokcnt:%d,lockOkCnt:%d",rfokcnt,lockOkCnt); 
//        #endif
//    }
//    else
//    {
//        rfokcnt = 0;
//    }

//    if(rfokcnt >= 1)
//    {
//        rfokcnt = 0;
//        rfokflag = 1;
//        
//        #ifdef  BEEP_SOUND_PLAY
//        if(RemKeyData.lockValue == key_value) 
//        {
//          Disable_G_INT();
//          OneWriteSendCmd(0xFE);
//          SoundPlayFlag = FALSE;
//          Enable_G_INT();
//        }
//        #else
//        OSTaskQPost ((OS_TCB      *)&RFDecodeTaskTCB,
//                    (void        *)p_rfRx_blk,
//                    (OS_MSG_SIZE  )4,
//                    (OS_OPT       )OS_OPT_POST_FIFO,
//                    (OS_ERR      *)&err);
//        #endif //#ifdef  BEEP_SOUND_PLAY
//    }

//    return 0;
//}

/*****************************************************************************
 Prototype    : TIM3_IRQHandler
 Description  : 定时器3输入捕获中断，用于采集遥控码
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : Billy
    Modification : Created function

*****************************************************************************/
//void TIM3_IRQHandler(void) 
//{         
//    OSIntEnter();
//    
//    if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)  
//    {  
//        if(RmtSta & 0x80)    //上次有数据被接收到了  
//        {     
//            RmtSta &=~ 0X10; //取消上升沿已经被捕获标记  
//            if((RmtSta&0X0F) == 0X00)RmtSta|=1<<6;//标记已经完成一次按键的键值信息采集  
//            if((RmtSta&0X0F) < 14)RmtSta++;  
//            else  
//            {  
//                RmtSta&=~(1<<7);//清空引导标识  
//                RmtSta&=0XF0;   //清空计数器   
//            }                                 
//        }                                 
//    }
//    
//    if(TIM_GetITStatus(TIM3,TIM_IT_CC4) != RESET)
//    {     
//        if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)) //上升沿捕获
//        {
//            TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling); //设置为下降沿捕获 
//            Low_cnt = TIM_GetCapture4(TIM3);                    //读取CCR4也可以清CC4IF标志位  //读取低电平的时间
//            TIM_SetCounter(TIM3,0);                             //清空定时器值
//            RmtSta|=0X10;                                       //标记上升沿已经被捕获
//        }else //下降沿捕获
//        {
//            High_cnt = TIM_GetCapture4(TIM3);                   //读取CCR4也可以清CC4IF标志位   //高电平的时间
//            TIM_SetCounter(TIM3,0);                             //清空定时器值
//            TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);  //设置为上升沿捕获 
//            RmtSta&=~(1<<4);
//        }   
//        if(RmtSta&0X10)                         //完成一次高低电平捕获 
//        {
//            if(RmtSta&0X80)//接收到了引导码
//            {
//                if(Low_cnt>230&&Low_cnt<700&&High_cnt>600&&High_cnt<1600)      //收到1
//                {//低电平范围:230--702us  高电平范围:600us--1600us
//                    #ifdef RCV_DATA_1527_MSB
//                    RmtRec<<=1;        //1527系列从MSB-->LSB接收         
//                    RmtRec|=1;                  
//                    #else 
//                    RmtRec >>= 1;      //CMT2150A从LSB-->MSB接收
//                    RmtRec |= 0x80000000;
//                    #endif
//                    
//                    num ++;
//                }
//                else if(Low_cnt>600&&Low_cnt<1600&&High_cnt>230&&High_cnt<700) //收到0
//                {//低电平范围:600--1600us 高电平范围:230--700us 
//                    
//                    #ifdef RCV_DATA_1527_MSB
//                    RmtRec<<=1;                 
//                    RmtRec|=0;                  
//                    #else
//                    RmtRec >>= 1;     
//                    #endif
//                    num ++;
//                }
//                else if(High_cnt>2000||Low_cnt>2000)  //数据错误
//                {
//                    num = 0;
//                    RmtRec = 0;
//                    RmtSta &= 0XF0;             //清空计时器
//                    RmtSta &= ~1<<7;            //取消标记接收到引导码                      
//                }
//                
//                if(num == 24)
//                {
//                    num = 0;
//                    RmtSta &= ~1<<7;            //取消标记接收到引导码
//                    key_rmote();                //处理解码
//                }
//            }
//            else 
//            {
//                if((Low_cnt > 8000) && (Low_cnt < 15000))       
//                {
//                    RmtSta |= 1<<7;             //标记成功接收到了引导码
//                    RmtRec = 0;
//                    num = 0;
//                }
//            }       
//        }   
//    }
//    
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4|TIM_IT_Update); 
//    
//    OSIntExit();
//}

/*****************************************************************************
 Prototype    : SwitchSpkMode
 Description  : 语音模式切换
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : Billy
    Modification : Created function

*****************************************************************************/
/*void SwitchSpkMode(void)
{
    if((lockOkCnt >= 15)&&(lockOkDelay == 0)&&(lockFlag == TRUE))
    {
      lockFlag = FALSE;
      
      #ifdef DEBUG_OUTPUT
      DEBUG_PRINT("\r\n===DeviceInfo.SoundMode:%d",DeviceInfo.SoundMode);
      DEBUG_PRINT("\r\n===lockOkCnt:%d",lockOkCnt); 
      #endif
      
      lockOkCnt = 0;
      lockOkDelay = TWO_SECOND;
      
      if(DeviceInfo.SoundMode == VOICE_MODE)
      {
          OneWriteSendCmd(VOICE_CLOSE);
          DeviceInfo.SoundMode = BELL_MODE;
      }
      else if(DeviceInfo.SoundMode == BELL_MODE)
      {
          OneWriteSendCmd(VOICE_OPEN);
          DeviceInfo.SoundMode = VOICE_MODE;
      }
      DeviceData.DeviceSoundMode = DeviceInfo.SoundMode;    
    }
}*/

/*****************************************************************************
 Prototype    : RemoteOpenSeatLock
 Description  : 遥控开启座桶锁
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : Billy
    Modification : Created function

*****************************************************************************/
/*void RemoteOpenSeatLock(void)
{
    OS_ERR         err;
    
    if(StartKeyFlag != TRUE)
        return;
    
    if(StartKeyOkDelay != 0)
        return;
    
    if((StartKeyFlag == TRUE)&&(StartKeyOkCnt >= 30)&&(StartKeyOkDelay == 0))
    {
        StartKeyFlag = FALSE;
        StartKeyOkCnt = 0;
        StartKeyOkDelay = FIVE_SECOND;

        DEBUG_PRINT("\r\n===Open Seat Lock!!!===\r\n");
        
        OpenSeatLock();
        OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_DLY, &err);
        CloseSeatLock();
    }
}*/
#endif //#ifdef USE_RF_REMOTE

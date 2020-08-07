//***********************************************************************************
// Copyright 2019, zksiot Ltd.
// Created by younger.fu @ 2019.03.18
// File name: interbow.c
// Description: wireless communication mac layer routine.
//***********************************************************************************

#include "sx1276-Hal.h"
#include "sx1276-Base.h"
#include "interBow.h"

#ifdef SUPPORT_RADIO

//#define DEBUG
#ifdef DEBUG
#define DEBUG_RX //used for rx ack from gateway
#ifdef DEBUG_RX
#define PACKET_MCU_TIME   30
#else
#define PACKET_MCU_TIME   30//120
#endif
#else
#undef  System_printf
#define System_printf(...)   //
#undef  System_flush
#define System_flush()   //

#define  PACKET_MCU_TIME   30
#endif

typedef enum{

   CONTROL_SET_END = 0x01,
   CONTROL_LINK_STOP,
}t_GatewayControlCmd;


typedef enum{

   CHNO_NONE = 0x00,
   CHNO_ACTIVED,
   CHNO_IS_UNACTIVED,
}t_ChnoState;

enum  IB_MSG_ID_V20{
//node send msg
    IMI_TX_SGN  = 0x10,//信号侦测命令                     ACK:IMI_RX_ACK
    IMI_TX_REG  = 0x11,//节点注册命令                     ACK:IMI_RX_NTP 
    IMI_TX_DATA = 0x12,//数据发送命令                     ACK:IMI_RX_ACK
    IMI_TX_ACK  = 0x13,//节点应答命令                     CMD:IMI_TX_SET
//gate send msg
    IMI_RX_ACK = 0x14,//网关应答命令                      CMD: IMI_TX_SGN,IMI_TX_DATA
    IMI_RX_NTP = 0x15,//注册应答命令                      CMD: IMI_TX_REG
    IMI_TX_SET = 0x16,//网关发送命令                      ACK: IMI_TX_ACK
    IMI_TX_CTL = 0x17,//网关控制命令                      ACK: NO
} ;

#pragma pack (1)

typedef struct {
    uint16_t crc;
    uint8_t  length;            //length     
    union {        
        struct{                      
            uint8_t  cmd:5;
            uint8_t  encrypt:1;
            uint8_t  busy:1;    //占用状态
            uint8_t  paraEx:1;
        };
        uint8_t command;        
    };
    uint16_t chnNo;             //channel no：0 is used for register，others are used for send data.
    uint16_t custID;            //network id
    uint32_t dstAddr;           //destination address
    uint32_t srcAddr;           //source address
    uint8_t  serialNum;         //流水号     
    uint8_t  payload[TXDATA_MAX_SIZE];//Payload
}IB_Packet;

//chno Manager struct
typedef struct {
    uint32_t DevID;
    uint8_t  active_flag;
}ChnnoManage_t;

typedef struct 
{
    union{
        struct 
        {
            uint8_t ack:1;
            uint8_t cmd:5;
            uint8_t netTime:1;
            uint8_t syn:1;
        };
        uint8_t status;
    };
}IB_Ack_Status;


IB_config_t IBConfig;
IB_hw_config_t IBHwConfig;
uint8_t InterBow_CurState;

uint8_t radioSerialNum;
uint8_t radioSerialNumAck;

bool gatewaySetRequest = 0;

#pragma pack ()

static InterBow_Gateway_Cb_t interBowGatewayCb;
static InterBow_Node_Cb_t interBowNodeCb;

#define SUPPORT_AES
#define USE_CHECKCODE 
#define CHECKCODE_LEN  2
#define SUPPORT_MUTI_GATEWAY   //支持多个网关跳频组网

#define REGISTER_FIAL_MAX_TIMES         5

#define  TX_PACKET_HEAD_LEN     (sizeof(IB_Packet) - TXDATA_MAX_SIZE)
#define  RX_PACKET_HEAD_LEN     TX_PACKET_HEAD_LEN

#define DevChnNUM   500
static ChnnoManage_t DevChnList[DevChnNUM] ={0,};


const Interbow_Hw_FxnTable InterHwFxn =
{   
    MODULE_LORA,
    SX1276_SleepMode,
    SX1276_StandbyMode,
    Lora_Reset,
    Lora_RxMode,
    Lora_Init,
    Lora_GetDefaultHwConfig,
    Lora_SetFreq,
    Lora_RateModeSet,
    Lora_SetRfPower,
    SX1276_RxPacket,
    Lora_SendPacket,
    SX1276_GetPacketRssi,
};


//***********************************************************************************
//
//Lora  event define.
//
//***********************************************************************************
#define INTERBOW_EVT_NONE                	Event_Id_NONE
#define INTERBOW_EVT_TX_DONE             	Event_Id_00
#define INTERBOW_EVT_RX_DONE             	Event_Id_01
#define INTERBOW_EVT_RX_TIMEOUT         Event_Id_02
#define INTERBOW_EVT_TX_DO               	Event_Id_03
#define INTERBOW_EVT_RX_DO               	Event_Id_04
#define INTERBOW_EVT_POWEROFF            Event_Id_05
#define INTERBOW_EVT_POWERON             Event_Id_06
#define INTERBOW_EVT_CAD_DONE            Event_Id_07
#define INTERBOW_EVT_CHANGEMODE      Event_Id_08
#define INTERBOW_EVT_CONFIG_MODE      Event_Id_09
#define INTERBOW_EVT_EXIT_CONFIG         Event_Id_10
#define INTERBOW_EVT_SET_CONFIG          Event_Id_11
#define INTERBOW_EVT_UPGRADE             	Event_Id_12

#define INTERBOW_EVT_ALL                 		0xffff
#define INTERBOWTASKSTACKSIZE           	1280




static Clock_Struct InterBowSend_ClkStru;
static Clock_Handle InterBowSend_ClkHandle;

//register manager timer
static Clock_Struct InterBowRegManager_ClkStruct;
static Clock_Handle InterBowRegManager_ClkHandle;

static Event_Struct InterBowLoraEvtStruct;
static Event_Handle InterBowEvtHandle;

static Task_Struct InterBowTaskStruct;
static uint8_t InterBowTaskStack[INTERBOWTASKSTACKSIZE];

static bool InterBowIsPowerOn = false;  //LORA是否打开
static int16_t last_rssi = -200;    //最后一包数据的rssi
static int16_t last_pwr = -200;    //最后一次网关接收采集器的信号强度反馈
static uint8_t ntp = 0;             //是否已注册/时间同步

static uint16_t packettime;         //一个数据包占用的最大时间ms。
static uint16_t maxdevnum;          //网络支持的最大设备数。

static uint32_t ntp_ms;             //网关当前参考时间点ms
static uint16_t devchnno = 0;       //网关分配的设备通道编号/网关下一个可分配的通道号

static uint32_t curdstAddr = 0;   //当前目标采集器ID/网关ID
static bool curisbusy = 0;         //当前是否处于占用模式
static uint8_t isOccupyBusy = 0;      //在主动发送是否下一包继续占用
static uint8_t  curTxcmd;          //当前发送命令
static uint8_t AckState = 0;        //应答状态
static bool RxAckOK = 0;        //是否收到过网关正确的应答包

static bool nodeLastBusyState = 0;
static uint8_t  interBowVer;

#define   INTERBOW_V10      1
#define   INTERBOW_V20      2


#ifdef LORA_OOK_TEST
void Lora_mode_event_post()
{
    if(InterBowEvtHandle)
        Event_post(InterBowEvtHandle, INTERBOW_EVT_CHANGEMODE);
}
#endif

void InterBowGatewayRequestSet(void)
{
    gatewaySetRequest = 1;
}

void InterBowGatewayRequestClear(void)
{
    gatewaySetRequest = 0;
}


void InterBowTxDoneEvtSet(void)
{
    if(InterBowEvtHandle)
    {
        InterBow_CurState = INTERBOW_TX_DONE;
        Event_post(InterBowEvtHandle, INTERBOW_EVT_TX_DONE);
    }
}

void InterBowRxDoneEvtSet(void)
{
    if(InterBowEvtHandle)
    {
        InterBow_CurState = INTERBOW_RX_DONE;
        Event_post(InterBowEvtHandle, INTERBOW_EVT_RX_DONE);
    }
}

void InterBowCadDoneEvtSet(void)
{
    if(InterBowEvtHandle)
    {
        InterBow_CurState = INTERBOW_CAD_DONE;
        Event_post(InterBowEvtHandle, INTERBOW_EVT_CAD_DONE);
    }
}


static uint16_t GetDevChnno(uint32_t devid)
{

    uint16_t i;
    uint16_t cursor_unactived = 0;

    
    for ( i = 1; i< DevChnNUM; ++i){
    
        if((DevChnList[i].DevID == 0))
            break;

        if(DevChnList[i].DevID == devid){

            DevChnList[i].active_flag = CHNO_ACTIVED;
            return i;
        }
        else if (DevChnList[i].active_flag == CHNO_IS_UNACTIVED && cursor_unactived == 0){

            cursor_unactived = i;
        }        
    }

    if(cursor_unactived != 0){
        DevChnList[cursor_unactived].DevID                = devid;
        DevChnList[cursor_unactived].active_flag          = CHNO_ACTIVED;
        return cursor_unactived;
    }
    
    if(i == DevChnNUM){//没有空闲的通道号，清除所有通道信息，重新分配
        memset(DevChnList, 0, sizeof(DevChnList));
        i = 1;
    }
    

    DevChnList[i].DevID                = devid;
    DevChnList[i].active_flag          = CHNO_ACTIVED;
    devchnno = i + 1;
    return i;    
}

uint16_t InterBow_CheckNode(uint32_t devid)
{
    uint16_t i;

    for ( i = 1; i< DevChnNUM; ++i){
    
        if((DevChnList[i].DevID == 0))
            return INTERBOW_INVALID_CHN;

        if((DevChnList[i].DevID == devid) && (DevChnList[i].active_flag != CHNO_IS_UNACTIVED)){
            return i;
        }
    }
    return INTERBOW_INVALID_CHN;
}

static void  IB_SetUpFreq(void)
{
    InterHwFxn.FreqSet(IBConfig.curFreq+IBHwConfig.offsetUpFreq);
}

static void  IB_SetDownFreq(void)
{
    InterHwFxn.FreqSet(IBConfig.curFreq+IBHwConfig.offsetDownFreq);
}


static uint8_t  IB_group_package(uint8_t msgId, IB_Packet* txPacket)
{
    uint8_t *rxbuffer;
    Calendar rRtcCalendar;
    uint32_t ms;    
    uint16_t devchn;
    uint8_t  index = 0;
    uint8_t userDataLen = 0;
    IB_Ack_Status *ackStateP;

#ifdef SUPPORT_AES
#ifdef S_C
    struct AES_ctx ctx;
#endif
#endif //SUPPORT_AES

    rxbuffer = txPacket->payload;


    ackStateP      = (IB_Ack_Status *)rxbuffer;
    txPacket->paraEx = 0;

    txPacket->dstAddr = curdstAddr;//如果目标地址为0，表示需要广播
    txPacket->custID = *(uint16_t*)IBConfig.customId;

    radioSerialNum++;
    txPacket->serialNum = radioSerialNum;

    #ifdef S_C
    if (*(uint32_t*)IBConfig.BindGateway  != 0 ){// 绑定到设置的网关
        txPacket->dstAddr = *(uint32_t*)IBConfig.BindGateway;
    }
    #endif

    txPacket->srcAddr = *(uint32_t*)IBConfig.DeviceId;//deviceid

    txPacket->encrypt = 0;
    txPacket->cmd = msgId;
    switch(msgId){
#ifdef S_C
    case IMI_TX_DATA:
        if(interBowNodeCb.SendData)
            userDataLen = interBowNodeCb.SendData(rxbuffer, TXDATA_MAX_SIZE, &isOccupyBusy);
        if(userDataLen == 0)
            return 0;
        txPacket->busy = isOccupyBusy;
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen;//length
        txPacket->chnNo = devchnno;
#ifdef SUPPORT_AES
    if(g_rSysConfigInfo.rfStatus & STATUS_AES128)
    {
        txPacket->encrypt = 1;

        AES_init_ctx_iv(&ctx, aes128Key, aes128Iv);
        AES_CBC_encrypt_buffer(&ctx, txPacket->payload, txPacket->length-RX_PACKET_HEAD_LEN);
        if(txPacket->length%16 != 0)
        {
            txPacket->length += 16 - txPacket->length%16;
        }
        
    }
#endif //SUPPORT_AES
        break;
#endif
    case IMI_TX_REG:
        index = 0;
        rxbuffer[index++] = PROTOCAL_VERSION; //protocal version
        rxbuffer[index++] = 0;    //0:node X:relay

        txPacket->chnNo = 0;//
        if(interBowNodeCb.SendRegistering)
            userDataLen = interBowNodeCb.SendRegistering(rxbuffer+index, TXDATA_MAX_SIZE-index);
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen + index;//length
        break;

    case IMI_TX_SGN:
        if(interBowNodeCb.SendSingleTest)
            userDataLen = interBowNodeCb.SendSingleTest(rxbuffer, TXDATA_MAX_SIZE);
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen;//length
        txPacket->chnNo = 0;//
        break;

    case IMI_TX_ACK:
        index = 0;
        rxbuffer[index++] = 0;    //success flag

        ackStateP->cmd = curTxcmd;

        rxbuffer[index++] = radioSerialNumAck;    //serial num
        rxbuffer[index++] = (int8_t)(last_rssi +164);    //success flag
        if(interBowNodeCb.SendSingleTest)
            userDataLen = interBowNodeCb.SendSingleTest(rxbuffer+index, TXDATA_MAX_SIZE-index);
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen + index;//length
        break;  
          
        
    case IMI_RX_ACK:
        if(isOccupyBusy == 1){
            txPacket->paraEx = 1;
            txPacket->busy = 1;
            txPacket->payload[index++] = 0x01;// 切换为网关主动发送，等待
        }
        ackStateP      = (IB_Ack_Status *)(rxbuffer+index);
        rxbuffer[index++] = 0;    //success flag
        ackStateP->cmd = curTxcmd;
        ackStateP->ack = RxAckOK;
        rxbuffer[index++] = radioSerialNumAck;    //success ack
        rxbuffer[index++] = (int8_t)(last_rssi +164);    //success flag

        if(curTxcmd == IMI_TX_SGN){
            if(interBowGatewayCb.SendSingleTestAck)
                userDataLen = interBowGatewayCb.SendSingleTestAck(rxbuffer+index, TXDATA_MAX_SIZE-index);
            txPacket->length = TX_PACKET_HEAD_LEN + userDataLen + index;//length
            break;
        }
        else if(curTxcmd == IMI_TX_DATA){
            if(!curisbusy){//最后一包数据发送同步时间及通道号
                ackStateP->syn = 1;

                ms = Clock_getTicks();
                ms = (ms*Clock_tickPeriod/1000);

                rxbuffer[index++] = HIBYTE(HIWORD(ms));//ntp_ms
                rxbuffer[index++] = LOBYTE(HIWORD(ms)); 
                rxbuffer[index++] = HIBYTE(LOWORD(ms));
                rxbuffer[index++] = LOBYTE(LOWORD(ms));                  
                
                ackStateP->netTime = 1;
                rRtcCalendar = Rtc_get_calendar();
                rxbuffer[index++] = rRtcCalendar.Year;
                rxbuffer[index++] = rRtcCalendar.Month;
                rxbuffer[index++] = rRtcCalendar.DayOfMonth;
                rxbuffer[index++] = rRtcCalendar.Hours;
                rxbuffer[index++] = rRtcCalendar.Minutes;
                rxbuffer[index++] = rRtcCalendar.Seconds;
            }
            if(interBowGatewayCb.SendDataAck)
                userDataLen = interBowGatewayCb.SendDataAck(rxbuffer+index, TXDATA_MAX_SIZE-index);
            txPacket->length = TX_PACKET_HEAD_LEN + userDataLen + index;//length
        }
    break;


    case IMI_RX_NTP:  //每次开机申请授时，同时分配同步发送参考时间,设备通道,设备上传周期(使用网关的采集周期)
        rxbuffer[index++] = 0;    //success flag
        ackStateP->cmd = curTxcmd;

        ackStateP->syn = 1;
        ms = Clock_getTicks();
        ms = (ms*Clock_tickPeriod/1000);

        rxbuffer[index++] = HIBYTE(HIWORD(ms));//ntp_ms
        rxbuffer[index++] = LOBYTE(HIWORD(ms)); 
        rxbuffer[index++] = HIBYTE(LOWORD(ms));
        rxbuffer[index++] = LOBYTE(LOWORD(ms));                  
        
        ackStateP->netTime = 1;
        rRtcCalendar = Rtc_get_calendar();
        rxbuffer[index] = rRtcCalendar.Year;
        if(Nwk_Is_Ntp()==0){//when gateway is in ntp state, send invalid time.  add by younger@20181127
            rxbuffer[index] |= 0x80; //invalid flag
        }
        index++;
        rxbuffer[index++] = rRtcCalendar.Month;
        rxbuffer[index++] = rRtcCalendar.DayOfMonth;
        rxbuffer[index++] = rRtcCalendar.Hours;
        rxbuffer[index++] = rRtcCalendar.Minutes;
        rxbuffer[index++] = rRtcCalendar.Seconds;
        devchn = GetDevChnno(txPacket->dstAddr);

        rxbuffer[index++] = HIBYTE((devchn));//chnno
        rxbuffer[index++] = LOBYTE((devchn));

        rxbuffer[index++] = HIBYTE(HIWORD(IBConfig.uploadPeriod));//uploadperiod
        rxbuffer[index++] = LOBYTE(HIWORD(IBConfig.uploadPeriod));
        rxbuffer[index++] = HIBYTE(LOWORD(IBConfig.uploadPeriod));
        rxbuffer[index++] = LOBYTE(LOWORD(IBConfig.uploadPeriod));

        if(interBowGatewayCb.SendRegisteringAck)
            userDataLen = interBowGatewayCb.SendRegisteringAck(rxbuffer+index, TXDATA_MAX_SIZE-index);
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen + index;//length
        break;
    
        
    case IMI_TX_SET:
        if(interBowGatewayCb.SendData)
            userDataLen = interBowGatewayCb.SendData(rxbuffer, TXDATA_MAX_SIZE, curdstAddr);
        txPacket->length = TX_PACKET_HEAD_LEN + userDataLen;//length
        txPacket->chnNo = devchnno;
        break;
    case IMI_TX_CTL:
        rxbuffer[0] = 0;//success flag
        index = 1;
        if(interBowGatewayCb.SendControlData)
            userDataLen = interBowGatewayCb.SendControlData(rxbuffer + index, TXDATA_MAX_SIZE - index);
        txPacket->length = RX_PACKET_HEAD_LEN + index;//length       
        break;

    }

    curTxcmd  = msgId;
    
#ifdef USE_CHECKCODE 
    txPacket->crc = CRC16((uint8_t*)txPacket + CHECKCODE_LEN, txPacket->length - CHECKCODE_LEN); 
#endif
    return txPacket->length;
}


#ifdef S_C
static void InterBow_SendFxn(UArg arg0)
{   
    System_printf("InterBow_SendFxn tick = %08ld\n",Clock_getTicks());
    System_flush();
    if(InterBowEvtHandle)
        Event_post(InterBowEvtHandle, INTERBOW_EVT_TX_DO);    
}
#endif


#ifdef S_G
static void InterBow_RegManagerFxn(UArg arg0)
{
    uint16_t i =0;
    for(i = 1; i < DevChnNUM ; i ++){

        if(DevChnList[i].DevID == 0)break;

        if( DevChnList[i].active_flag == CHNO_NONE){

            DevChnList[i].active_flag = CHNO_IS_UNACTIVED;
        }else if (DevChnList[i].active_flag == CHNO_ACTIVED ){

            DevChnList[i].active_flag = CHNO_NONE;
        }
    }
}
#endif

xdc_UInt InterBow_pend( ti_sysbios_knl_Event_Handle __inst, xdc_UInt andMask, xdc_UInt orMask, xdc_UInt32 timeout )
{
    #if 0
        return Event_pend(__inst, andMask, orMask, timeout );
    #else
    uint32_t  times;
    xdc_UInt eventId;

    if(timeout == BIOS_WAIT_FOREVER || timeout == BIOS_NO_WAIT)
        return Event_pend(__inst, andMask, orMask, timeout);

    times = (timeout + 9) / (10*CLOCK_UNIT_MS);
    
    while(times--){

        eventId = Event_pend(__inst, andMask, orMask, 10*CLOCK_UNIT_MS);
        
        if(eventId == 0) 
        {           
            if(GPIO_read(LORA_DIO0_PIN)){
                return orMask;
            }            
        }
        else{
            return eventId;
        }  
    }    
    
    return 0;  

    #endif

}

void InterBow_SyncSendTime()
{
    uint32_t passtime,sendtime;
    System_printf("ntp_ms = %08ld  tick= %08ld \n", ntp_ms, Clock_getTicks());
    System_flush();
        
    passtime = ntp_ms % ((uint32_t)IBConfig.uploadPeriod*1000);
    sendtime = devchnno*packettime;
    if(sendtime <= passtime)
        sendtime += (IBConfig.uploadPeriod*1000);
      
    sendtime -= passtime;

    //if (sendtime < IBConfig.uploadPeriod*1000/2)//当多条数据发送导致占用时间过长
    //    sendtime+=IBConfig.uploadPeriod*1000;//延迟到下一周期再发。
    
    Clock_stop(InterBowSend_ClkHandle);
    Clock_setPeriod(InterBowSend_ClkHandle, IBConfig.uploadPeriod*CLOCK_UNIT_S);
    Clock_setTimeout(InterBowSend_ClkHandle,  sendtime*CLOCK_UNIT_MS);
    //Clock_setTimeout(InterBowSend_ClkHandle,  IBConfig.uploadPeriod*CLOCK_UNIT_MS);//以当前时间及周期来确定下次发送时间。
    Clock_start(InterBowSend_ClkHandle);            

}

static uint32_t InterBow_GetPassTime(uint32_t starttime)
{
    uint32_t curtime;

    curtime = Clock_getTicks();

    if(curtime >= starttime)
       curtime -= starttime;
    else
       curtime += (0xffffffff-starttime+1); 

    return curtime;    
}

static void InterBow_CalcPacketTime()
{
    packettime = SX1276_GetPacketTime(TX_PACKET_HEAD_LEN+TXDATA_MAX_SIZE);//send time    
    packettime += packettime;//ack time
    packettime += PACKET_MCU_TIME;
   
    maxdevnum = (IBConfig.uploadPeriod*1000)/(packettime);

    if(maxdevnum >100)//取整到100的倍数
        maxdevnum = (maxdevnum / 50) * 50;
    else if(maxdevnum >10)//取整到10的倍数
        maxdevnum = (maxdevnum / 5) * 5;

    packettime = (IBConfig.uploadPeriod*1000)/ maxdevnum;

    //maxdevnum = maxdevnum*8/10;//只使用80%的容量，20%用来处理出错的情况。

}
static bool InterBow_SendData(IB_Packet *curTxPacket, uint32_t timeout)
{
    uint8_t send_retrys = 0;
    UInt eventId;
    uint8_t acklen,CmdPos;
    Calendar rRtcCalendar;
    bool ret = FALSE;    
    IB_Packet RxPacket,*curRxPacket;
    uint16_t tmpdevchnno;
    bool  firstActive = true;
    uint32_t starttime;
    bool HavedSend = false;//记录超时或收到应答之前的上一次是否发送过数据。
    bool SendAndRecv = false;//记录是否刚发送过数据后顺序进入接收状态。
    //uint8_t i,SendCount = 0;//记录已经发送的次数
    uint8_t  index = 0, *rxbuffer;
    IB_Ack_Status ackStateTemp;
    uint8_t userDataIndex;

    RxAckOK = FALSE;
    tmpdevchnno = devchnno;
   
    starttime = Clock_getTicks();

    if(!curisbusy)
        goto RX_RETRY;//进行接收侦听
    
TX_RETRY:

    #ifdef DEBUG_RX
        goto RX_RETRY;
    #endif
    
    if(!InterBowIsPowerOn){      
        InterBow_CurState = INTERBOW_IDLE;
        InterHwFxn.Sleep();
        return FALSE;//关机，退出
    } 
#if 0
    if(!curisbusy){        
        i = SendCount++;
        while(i--)
            delay_ms(2);//根据发送次数做相应延时，避免重试的采集器与其它重试或原始位置的采集器刚好同时发送。        
    }
#endif
    if(InterBow_GetPassTime(starttime) >= timeout){//发送时间超过一个周期，认为网关可能关闭或其他异常。
        System_printf("TX TIMEOUT EXIT\n");
        System_flush();
        Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 3);  
        return FALSE;//退出。
    }

#ifdef SUPPORT_LORA_APC    
    if(IBConfig.apcEnable) {//auto power control        
        if(last_pwr > -100) {
            if(IBConfig.powerConfig > 2) {
                IBConfig.powerConfig--;
                InterHwFxn.RfPowerSet(IBConfig.powerConfig);
                //========================================================
#ifdef DEBUG
                   //sprintf((char*)buffer, "%03d\n",IBConfig.powerConfig);
                   // Flash_log(buffer);
                   System_printf("APC Dec last_pwr= %d TxPower %d \n",last_pwr, IBConfig.powerConfig);
                   System_flush();
#endif
               //==========================================================
            }
        }
        else if(last_pwr < -120) {
            if(IBConfig.powerConfig < IBHwConfig.powerConfigMax) {
                IBConfig.powerConfig++;
                InterHwFxn.RfPowerSet(IBConfig.powerConfig);
                //========================================================
#ifdef DEBUG
                    //sprintf((char*)buffer, "%03d\n",IBConfig.powerConfig);
                   // Flash_log(buffer);
                System_printf("APC Inc last_pwr= %d TxPower %d \n",last_pwr, IBConfig.curFreq);
                System_flush();
#endif
                //==========================================================
            }
        }

        last_pwr = -200;
   }
#endif

    Led_ctrl(LED_G, 1, 20* CLOCK_UNIT_MS, 1);

    //0.检测通道是否空闲
   #if 0
    while(InterBowIsPowerOn) {
        
        InterBow_CurState = INTERBOW_CAD_DOING;   
        IB_SetUpFreq()
        Lora_CADMode();
        eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_CAD_DONE, 3*G_TsXms*CLOCK_UNIT_MS);

        if(eventId == INTERBOW_EVT_CAD_DONE) {
            if(GPIO_read(LORA_DIO1_PIN) == 0){//idle
                break;
            }
            else{ //busy
                InterHwFxn.Sleep();
                Task_sleep((packettime + acktime) * CLOCK_UNIT_MS);
            }
        }        
    }
   #endif
   
    send_retrys = 0;    
   
SEND_RETRY:

    #ifdef DEBUG        
        System_printf("SEND ...curisbusy = %d Freq = %ld tick = %08ld\n", curisbusy, IBConfig.curFreq,Clock_getTicks());
        System_flush();
    #endif   

    //1.发送数据包             
    InterBow_CurState = INTERBOW_TX_DOING;    
    IB_SetUpFreq();
    InterHwFxn.SendPacket((uint8_t*)curTxPacket,curTxPacket->length);

    
    //2.等待发送完成
    eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_TX_DONE, packettime*CLOCK_UNIT_MS);
    if(eventId != INTERBOW_EVT_TX_DONE) 
    {   
        
        System_printf("SEND TIMEOUT\n");
        System_flush();
        Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 4);

        if(++send_retrys > 3){
            InterHwFxn.Reset();
            return FALSE;//硬件发送3次错误，退出。
        }
        goto SEND_RETRY;
    }

    HavedSend = true;  //已经发送标志置位
    SendAndRecv = true;//刚发送过数据后顺序进入接收状态。
    
    //3..等待应答
RX_RETRY:     

    if(SendAndRecv){//刚发送过数据后顺序进入接收状态。
        SendAndRecv = false;
    }
    else{//
        HavedSend = false;  //已经发送标志清除
    }
    
    if(!InterBowIsPowerOn){      
        InterBow_CurState = INTERBOW_IDLE;
        InterHwFxn.Sleep();   
        return FALSE;//关机，退出
    } 

    if(InterBow_GetPassTime(starttime) >= (timeout))//接收监听时间超过一个周期，认为网关可能忙碌或其他异常。
    {
        System_printf("RX TIMEOUT EXIT\n");
        System_flush();
        Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 3);  
        return FALSE;//
    }


    InterBow_CurState = INTERBOW_RX_DOING;
    IB_SetDownFreq();
    InterHwFxn.RxMode();                  //进入接收状态,


    //4.收到应答或超时
    eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_RX_DONE, (1*packettime)*CLOCK_UNIT_MS);
    if(eventId != INTERBOW_EVT_RX_DONE) 
    {
        System_printf("ACK TIMEOUT tmpdevchnno = %d\n", tmpdevchnno);
        System_flush();

        //将临时通道向前推进1步
        if(tmpdevchnno > 0)
            tmpdevchnno--;
        else
            tmpdevchnno = (maxdevnum + tmpdevchnno -1);

        if (curisbusy){            
            goto TX_RETRY;//busy模式，开始发送            
        }
        
        if(firstActive){                                
            firstActive = false;
            tmpdevchnno = 0;//第一次即超时，将临时通道号置0，避免失败后与后面通道号冲突，开始发送。
        }
        
        if(curTxPacket->cmd == IMI_TX_SGN){
            goto TX_RETRY;////信号侦测，开始发送
        }
        
        if(curTxPacket->cmd == IMI_TX_REG){//注册
            if (tmpdevchnno % 10 == 0 ){
                goto TX_RETRY;//排队时间到，开始发送
            }
            else{
                 uint16_t tmpchnno;
                 tmpchnno = tmpdevchnno % 10;
                 InterHwFxn.Sleep();
                 Task_sleep((tmpchnno-1)*packettime);
                 tmpdevchnno = tmpdevchnno - (tmpchnno -1);
                 goto RX_RETRY;
            }
        }

        if(tmpdevchnno == 0){//
            //tmpdevchnno = maxdevnum*2;//将设备放置队尾，本周期不再发送。
            tmpdevchnno = maxdevnum /2;//将设备放置1/3队列处，，本周期有3次发送机会。
            index = curTxPacket->chnNo % 3;
            while(index--)
                delay_ms(5);//根据通道号做相应延时，避免超时的采集器与其它超时或原始位置的采集器刚好同时发送。
                
            goto TX_RETRY;//排队时间到，开始发送
        }
        else{
            if(RxAckOK){//if not recv sign from netgate, rx 
               if(tmpdevchnno > 1){
                   InterHwFxn.Sleep();
                   Task_sleep((tmpdevchnno-1)*packettime);
                   tmpdevchnno = 1;
               }
            }
            goto RX_RETRY;//排队时间未到，继续监听
        }        
    }

    //5.接收并分析应答数据
    last_rssi = InterHwFxn.GetRssi();

    System_printf("last_rssi = %d \n", last_rssi);
    System_flush();

    
    acklen = InterHwFxn.ReadRxPacket((uint8_t*)&RxPacket,sizeof(RxPacket));
    if(acklen == 0)
    {
        System_printf("CRC ERR\n");
        System_flush();        
        if(curisbusy){
            Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 2);  
            goto TX_RETRY;
        }
        else
            goto RX_RETRY;
    }

    curRxPacket = &RxPacket;
    CmdPos = 0;
    
    if (curRxPacket->length < RX_PACKET_HEAD_LEN || acklen < curRxPacket->length){
        System_printf("ACK LEN ERR acklen=%d  curRxPacket->length=%d\n", acklen, curRxPacket->length);
        System_flush();        
        if(curisbusy){
            Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 2);
            goto TX_RETRY;
        }
        else
            goto RX_RETRY;
    }

#ifdef USE_CHECKCODE 
    //软件检测校验码是否正常
    if(curRxPacket->crc != CRC16((uint8_t*)curRxPacket + CHECKCODE_LEN,curRxPacket->length - CHECKCODE_LEN))
    {
        System_printf("CHK CODE ERR\n");
        System_flush();        
        if(curisbusy){
            Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 2);  
            goto TX_RETRY;
        }
        else
            goto RX_RETRY;
    }  

#endif

#ifdef SUPPORT_AES
    if(curRxPacket->encrypt)
    {
        struct AES_ctx ctx;

        AES_init_ctx_iv(&ctx, aes128Key, aes128Iv);
        AES_CBC_decrypt_buffer(&ctx, curRxPacket->payload, curRxPacket->length-RX_PACKET_HEAD_LEN);
    }
#endif //SUPPORT_AES

    //5.1判断数据异常    
    
    radioSerialNumAck = curRxPacket->serialNum;
    if( curRxPacket->custID != curTxPacket->custID ){ //客户码必须一致，否则继续监视网关状态。    
    
        System_printf("ERR custID = %08lx\n", curRxPacket->custID);
        System_flush();  

        if(curTxPacket->cmd == IMI_TX_SGN){
               return FALSE;//信号侦测，立即返回FALSE
        }

        if (curTxPacket->cmd == IMI_TX_REG){//判断客户码,发现非目标网关，则立即退出不向该网关申请注册
            return FALSE;
        }      

        if(curisbusy){
            Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 2);  
            goto TX_RETRY;
        }
        else
            goto RX_RETRY;                   
    }


    if(curTxPacket->cmd == IMI_TX_SGN){
        userDataIndex = 2;
        ackStateTemp.status = curRxPacket->payload[0];
        if(ackStateTemp.syn)
            userDataIndex += 10;
        if(interBowNodeCb.RecvSingleTestAck)
            interBowNodeCb.RecvSingleTestAck(curRxPacket->payload+userDataIndex, curRxPacket->length-RX_PACKET_HEAD_LEN-userDataIndex);
        return TRUE;//信号侦测，立即返回FALSE
     }


    if ( curRxPacket->srcAddr == curTxPacket->dstAddr ){
        RxAckOK = TRUE;//收到了目标网关的正确数据应答包。
    }
    
    if(curRxPacket->dstAddr == *(uint32_t*)IBConfig.DeviceId){//是对本设备的应答

        //收到了对本设备的应答，目标地址为0或者与网关地址一致，否则有异常。
        if ((curTxPacket->dstAddr != 0 && curTxPacket->dstAddr != curRxPacket->srcAddr)){

            System_printf("GATE ERR srcAddr = %08lx\n", curRxPacket->srcAddr);
            System_flush();  
            if(curisbusy){
                Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 1);  
                goto TX_RETRY;
            }
            else {
                goto RX_RETRY; //继续监听
            }
        }
    }    
    else {//不是对本设备的应答       
            
        if(curisbusy){//处于busy 模式，却收到了非本设备的应答，有异常。
            System_printf("IN BUSY BUT ACK dstAddr = %08lx\n", curRxPacket->dstAddr);
            System_flush();  
            Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 1);                
            goto TX_RETRY;
        }
        else {
            
            if(curRxPacket->busy){//网关处于非空闲状态。
                System_printf("GATE BUSY dstAddr = %08lx   chnNo = %d\n", curRxPacket->dstAddr, curRxPacket->chnNo);
                System_flush();  
                goto RX_RETRY;//继续监视网关状态。
            }
            else{//网关处于空闲状态                

                System_printf("GATE IDLE dstAddr = %08lx  chnNo = %d  tmpdevchnno = %d\n", curRxPacket->dstAddr, curRxPacket->chnNo,tmpdevchnno);
                System_flush();   
                        
                if(curTxPacket->chnNo == 0){//对注册来说，不管是其它采集器的抢占注册成功；还是其它采集器发送数据成功，空闲就是机会，按注册流程处理。
                                    
                     //将临时通道向前推进1步
                    tmpdevchnno = (maxdevnum + tmpdevchnno -1) % maxdevnum;      

                    if(firstActive){                                     
                        firstActive = false;   
                        goto TX_RETRY;//排队时间到，开始发送                           
                    }

                    if (tmpdevchnno % 10 == 0 ){
                        goto TX_RETRY;//排队时间到，开始发送
                    }
                    else{
                          uint16_t tmpchnno;
                          tmpchnno = tmpdevchnno % 10;
                          InterHwFxn.Sleep();
                          Task_sleep((tmpchnno-1)*packettime);
                          tmpdevchnno = tmpdevchnno - (tmpchnno -1);
                          goto RX_RETRY;//排队时间未到，继续监听
                    }
                }
                else{//发送数据

                    if(curRxPacket->chnNo == 0){//对发送数据来说，被注册抢占成功是意外事件，不需要处理而继续侦听，当前已发送数据的采集器例外。
                     
                        if(HavedSend == true){
                            goto TX_RETRY;//采集器已经向网关发送过数据的再次发送。
                        }
                        else{
                            goto RX_RETRY;//继续监听    
                        }                         
                    }
                    else{//其它采集器发送数据成功完成。                        
                    
                        //一起发送会导致网关CRC错误，根据sensor临时通道编号顺序发送。
                        //将临时通道向前推进1步
                        if(tmpdevchnno > 0) 
                            tmpdevchnno--;
                        else                        
                            tmpdevchnno = (maxdevnum + tmpdevchnno -1) ;    
                        
                                    
                        if(firstActive){                                     
                            firstActive = false;                                                                
                            if(tmpdevchnno >= curRxPacket->chnNo){
                                tmpdevchnno = (tmpdevchnno - curRxPacket->chnNo);
                            }
                            else{
                                //tmpdevchnno = (maxdevnum + tmpdevchnno - curRxPacket->chnNo);
                                tmpdevchnno = 0;//首次唤醒发现后面的采集器在发送，有异常，直接发送。
                            }                                
                        }
                                                    
                        if(tmpdevchnno == 0){
                            //tmpdevchnno = maxdevnum*2;//将设备放置队尾，本周期不再发送。
                            tmpdevchnno = maxdevnum /2;//将设备放置1/3队列处，，本周期有3次发送机会。

                            index = curTxPacket->chnNo % 10;
                            while(index--)
                                delay_ms(5);//根据通道号做相应延时，避免采集器与其它采集器刚好同时发送。
                                
                            goto TX_RETRY;//排队时间到，开始发送
                        }
                        else{
                            if(RxAckOK){//if not recv sign from netgate, rx 
                            if(tmpdevchnno > 1){
                                  InterHwFxn.Sleep();
                                  Task_sleep((tmpdevchnno-1)*packettime);
                                  tmpdevchnno = 1;
                            }
                            }
                            goto RX_RETRY;//排队时间未到，继续监听
                        }
                    }
                    
                }                
            }
        }
    }


    curisbusy = curRxPacket->busy;//记录网关返回的busy状态。

    //netgate  send: length(1BYTE) cmd(1BYTE)  srcadd (4BYTE)   dstadd (4BYTE) ...
    while((curRxPacket->length >= RX_PACKET_HEAD_LEN) && (CmdPos + curRxPacket->length <= acklen)) {//  多条应答命令时分别处理
        
        index = 0;
        if(curRxPacket->paraEx)
        {
            index++;
            while(curRxPacket->payload[index-1] & 0x80){
                index++;
            }
        }
        rxbuffer = curRxPacket->payload + index;

        switch(curRxPacket->cmd) {//ACK CMD

        case IMI_TX_CTL://网关主动断开。
            System_printf("GATE TO IDLE\n");
            System_flush();
            goto RX_RETRY;//发送中出现意外，被网关主动断开，开始侦听。

        case IMI_RX_ACK:
            ackStateTemp.status = rxbuffer[0];

            if(ackStateTemp.ack == 0 && rxbuffer[1] == radioSerialNum)
                AckState = 0; //一般都是0，表示成功，但可能发生网关保存数据处理不过来导致不为0，表示处理第几条数据失败。
            else
                AckState = 1;
            last_pwr =  rxbuffer[2] - 164; //获取网关反馈的RSSI值

            index = 3;

            if(ackStateTemp.syn){
                //由于每个设备的timer有偏差，需要每次发送完数据后与网关同步发送参考时间
                HIBYTE(HIWORD(ntp_ms)) = rxbuffer[index++];
                LOBYTE(HIWORD(ntp_ms)) = rxbuffer[index++];                    
                HIBYTE(LOWORD(ntp_ms)) = rxbuffer[index++];
                LOBYTE(LOWORD(ntp_ms)) = rxbuffer[index++];
            }

            if(ackStateTemp.syn){
                rRtcCalendar.Year       = rxbuffer[index++] + CALENDAR_BASE_YEAR;
                rRtcCalendar.Month      = rxbuffer[index++]; 
                rRtcCalendar.DayOfMonth = rxbuffer[index++];
                rRtcCalendar.Hours      = rxbuffer[index++];
                rRtcCalendar.Minutes    = rxbuffer[index++]; 
                rRtcCalendar.Seconds    = rxbuffer[index++];                
                Rtc_set_calendar(rRtcCalendar); 
            }
            last_pwr =  rxbuffer[index++] - 164; //获取网关反馈的RSSI值

            if(AckState == 0)
            {
                if(interBowNodeCb.RecvDataAck)
                    interBowNodeCb.RecvDataAck(rxbuffer+index, curRxPacket->length-RX_PACKET_HEAD_LEN-index);
                ret = TRUE;
            }
            if((curRxPacket->paraEx == 1) && (curRxPacket->payload[0] & 0x01)){ // 网关有数据需要下发，优先下发，转换为接收状态
                nodeLastBusyState = isOccupyBusy;
                isOccupyBusy = 0;       //清除繁忙状态，等接收完网关后再继续发送数据
                curisbusy = 1;          //设置为繁忙状态，让采集器切换至接收状态
            }
            break;
            
        case IMI_RX_NTP:
            if(curTxcmd == IMI_TX_REG) {      

                curdstAddr = curRxPacket->srcAddr;//注册成功，记住网关地址
                
                ackStateTemp.status = rxbuffer[0];
                AckState = ackStateTemp.ack; //一般都是0，表示成功，但可能发生网关保存数据处理不过来导致不为0，表示处理第几条数据失败。
                index = 1;

                //由于每个设备的timer有偏差，需要每次发送完数据后与网关同步发送参考时间
                HIBYTE(HIWORD(ntp_ms)) = rxbuffer[index++];
                LOBYTE(HIWORD(ntp_ms)) = rxbuffer[index++];                    
                HIBYTE(LOWORD(ntp_ms)) = rxbuffer[index++];
                LOBYTE(LOWORD(ntp_ms)) = rxbuffer[index++];

                if(rxbuffer[index]&0x80 == 0){//when gateway is in ntp state, send invalid time, not use . add by younger@20181127
                    rRtcCalendar.Year       = (rxbuffer[index++] & 0x7f) + CALENDAR_BASE_YEAR;
                    rRtcCalendar.Month      = rxbuffer[index++]; 
                    rRtcCalendar.DayOfMonth = rxbuffer[index++];
                    rRtcCalendar.Hours      = rxbuffer[index++];
                    rRtcCalendar.Minutes    = rxbuffer[index++]; 
                    rRtcCalendar.Seconds    = rxbuffer[index++];              
                    Rtc_set_calendar(rRtcCalendar);                
                }
                else {
                    index += 6;
                }

                HIBYTE((devchnno)) = rxbuffer[index++];
                LOBYTE((devchnno)) = rxbuffer[index++];                
                HIBYTE(HIWORD(IBConfig.uploadPeriod)) = rxbuffer[index++];
                LOBYTE(HIWORD(IBConfig.uploadPeriod)) = rxbuffer[index++];
                HIBYTE(LOWORD(IBConfig.uploadPeriod)) = rxbuffer[index++];
                LOBYTE(LOWORD(IBConfig.uploadPeriod)) = rxbuffer[index++];
                    
                InterBow_CalcPacketTime();//uploadPeriod更新，更新时间片                    
                ntp = 1;
                ret = TRUE;

                if(interBowNodeCb.RecvRegisteringAck)
                    interBowNodeCb.RecvRegisteringAck(rxbuffer+index, curRxPacket->length-RX_PACKET_HEAD_LEN-index);
            }
            break;
        
        case IMI_TX_SET:

            break;
        }

        
        CmdPos += curRxPacket->length;
        curRxPacket = (IB_Packet*)((uint8_t*)&RxPacket + CmdPos);       
        
    }
    
    //6.发送完成
    Led_ctrl(LED_B, 1, 20* CLOCK_UNIT_MS, 1);
    InterBow_CurState = INTERBOW_IDLE;

#ifdef DEBUG        
    System_printf("SEND Finish ret= %d\n", ret);
    System_flush();
#endif    

    return ret;
}

static void InterBow_detect_and_fix_freq(IB_Packet *curTxPacket)
{
    uint8_t i;
    int8_t minrssi = 0;
    //int16_t FreqList[FREQ_DOT_MAX]={0};//存放所有频点对应的rssi    
    int16_t minFreq = 0;

    ntp = 0;//网关处于注册状态
    devchnno = 0;//reset

    if(*(uint32_t*)IBConfig.DeviceId == 0)
        return;        
    
    if (IBConfig.rateFixEnable){//fix freq by custid

        IBConfig.curFreq = IBHwConfig.baseFreq + IBHwConfig.baseFreqOffset * IBConfig.curFreqchannel;
        ntp = 1;//网关处于非注册状态     
        return;
    }        

    for( i = 0; i < IBHwConfig.freqMaxchannel; ++i){
    
        if(!InterBowIsPowerOn)return;
    
        IBConfig.curFreq = IBHwConfig.baseFreq + IBHwConfig.baseFreqOffset * i;
        
        curdstAddr = 0;//信号检测采用广播。          
        curTxPacket->busy = 0; //0 ：表示只是信号检测，没有其它数据需要处理。
        IB_group_package(IMI_TX_SGN,curTxPacket);//发送SGN, 用来搜索存在的网关及信号。
        last_rssi = -200;
        InterBow_SendData(curTxPacket, 10*packettime*CLOCK_UNIT_MS);//只需获取rssi，不需要结果。
        //FreqList[i] = last_rssi;
        if(last_rssi < minFreq){
            minFreq = last_rssi;
            minrssi = i;                               
            if(minFreq == -200)
                break;
        }
    }

    IBConfig.curFreq = IBHwConfig.baseFreq + IBHwConfig.baseFreqOffset * minrssi;
    ntp = 1;//网关处于非注册状态
}

#ifdef S_C
static bool InterBow_master_register(IB_Packet *curTxPacket)
{
#ifdef SUPPORT_RADIO_LOWPOWER
    uint8_t regfails = 0;  
#endif //SUPPORT_RADIO_LOWPOWER  
    bool Ntpresult;
    uint8_t currate = 0;
    uint32_t starttime;
    starttime = Clock_getTicks();
    
#ifdef SUPPORT_MUTI_GATEWAY       
    int8_t freqdot = 0;                   
#endif

    if(ntp != 0) //同步系统时间，即采集器开始注册过程。     
        return false;

    //devchnno = 0; //reset 
    if(*(uint32_t*)IBConfig.DeviceId == 0)
        return true;
        
    
SEND_NTP:     
    
    if(InterBow_GetPassTime(starttime) >= IBConfig.uploadPeriod*CLOCK_UNIT_S){// add by younger@201904012 for 注册时间超过一个周期，可能lora模块异常,重新初始化。
        InterHwFxn.Reset();
        starttime = Clock_getTicks();
    }

#ifdef SUPPORT_MUTI_GATEWAY   
    if (IBConfig.rateFixEnable){//fix freq by custid
        IBConfig.curFreq = IBHwConfig.baseFreq + IBHwConfig.baseFreqOffset * IBConfig.curFreqchannel;

#ifdef SUPPORT_RADIO_LOWPOWER
        if(regfails > REGISTER_FIAL_MAX_TIMES)
        {
            if(Clock_isActive(InterBowSend_ClkHandle))
                Clock_stop(InterBowSend_ClkHandle);
            Clock_setPeriod(InterBowSend_ClkHandle, uploadPeriod*CLOCK_UNIT_S);
            Clock_setTimeout(InterBowSend_ClkHandle,  uploadPeriod*CLOCK_UNIT_S);
            Clock_start(InterBowSend_ClkHandle);
            InterHwFxn.Reset();
            return true; 
        } 
#endif // SUPPORT_RADIO_LOWPOWER


    }   
    else{

        IBConfig.curFreq = IBHwConfig.baseFreq + freqdot*(IBHwConfig.baseFreqOffset);//fix freq
        freqdot++;

        if(freqdot >= IBHwConfig.freqMaxchannel){
        
            //switch to next rate.
            currate++;
            if(currate >= IBHwConfig.rateMax)
                currate = 0;

            InterHwFxn.RateModeSet(currate);
            InterHwFxn.GetHwConfig(&IBHwConfig);
            InterBow_CalcPacketTime();//BW & SF更新，更新时间片          
            freqdot = 0;  
            //regfails++;
            //if(regfails > 10);
            //    regfails = 10;//最多等待10分钟后重新注册。
            //Task_sleep(regfails*60*CLOCK_UNIT_S);
            goto SEND_NTP;//注册失败，待下个周期重新注册。      
        }

    }
#endif

    if(!InterBowIsPowerOn)return false;

    //先检测当前频点是否存在网关。
    curdstAddr = 0;//信号检测采用广播。          
    curTxPacket->busy = 0; //0 ：表示只是信号检测，没有其它数据需要处理。
    IB_group_package(IMI_TX_SGN,curTxPacket);//发送SGN, 用来搜索存在的网关及信号。
    last_rssi = -200;
    if (InterBow_SendData(curTxPacket, 2*packettime*CLOCK_UNIT_MS)==FALSE){  //如果返回FALSE则 RSSI设置为无效的
        last_rssi = -200;
    }
    if(last_rssi == -200){
        goto SEND_NTP;//没有网关，更换频点重新注册。
    }
    
    curdstAddr = 0;//注册采用广播。    
    curTxPacket->busy = 0; //0 ：表示只是注册，没有其它数据需要处理。
    IB_group_package(IMI_TX_REG,curTxPacket);    
    Ntpresult = InterBow_SendData(curTxPacket, IBConfig.uploadPeriod*CLOCK_UNIT_S);
        
    // if((g_rSysConfigInfo.rfStatus & STATUS_LORA_TEST)) {//测试模式，一直需要发送数据。
    //     freqdot = 0;
    //     Task_sleep(10*CLOCK_UNIT_MS);//必须延时，否则lora工作不正常。
    //     System_printf("TEST\n");
    //     System_flush();
    //     goto SEND_NTP;
    // }
    
    if (Ntpresult){//success
        InterBow_SyncSendTime();                      
        ntp = 1;     
        Sys_event_post(SYS_EVT_DISP);//update time           
        return true;//注册完后不发送数据，等待时间通道发送。            
    }          
    else{
        
      System_printf("NTP FAIL \n");
      System_flush();  
     
#ifdef SUPPORT_MUTI_GATEWAY       
      goto SEND_NTP;//注册失败，更换频点重新注册。          
#else          
      Task_sleep(IBConfig.uploadPeriod*CLOCK_UNIT_S);
      goto SEND_NTP;//注册失败，待下个周期重新注册。
#endif      
    }
   
}
#endif

#ifdef LORA_OOK_TEST    
static void InterBow_master_test_ook(IB_Packet *curTxPacket)
{
    extern Event_Handle sysEvtHandle;
    bool  changmode = 1, isook = 1;
    
#ifdef OOK_TEST_AUTHENTICATION
    #define TEST_FREQ  471000000 //天线认证测试
    //#define TEST_FREQ  509000000
#else
    #define   TEST_FREQ  480000000 //天线匹配测试
#endif

    while(1){   

        if(!InterBowIsPowerOn)return;
        
        if(isook){ 
            Led_ctrl(LED_B, 1, 500* CLOCK_UNIT_MS, 5);
            if(changmode){
                changmode = 0;
                OOK_test(TEST_FREQ);   
            }
        }    
        else{
            IBConfig.curFreq = TEST_FREQ;///LORA_FREQ;
            IBConfig.powerConfig = 14;
            InterHwFxn.RfPowerSet(IBConfig.powerConfig);
            curdstAddr = 0;//测试采用广播。          
            curTxPacket->busy = 0; //0 ：表示只是测试，没有其它数据需要处理。
            IB_group_package(IMI_TX_REG,curTxPacket);
            InterBow_SendData(curTxPacket, 1*packettime*CLOCK_UNIT_MS);
        }            
        
        if(INTERBOW_EVT_CHANGEMODE == Event_pend(InterBowEvtHandle, 0, INTERBOW_EVT_CHANGEMODE, 100)){
            changmode = 1;
            isook =!isook;
        }           
        
    } 
}
#endif


#ifdef S_C
//***********************************************************************************
// InterBow_MasterKeepConnect
//
//**************************************************************************************
static void InterBow_MasterKeepConnect(uint32_t timeout){
    uint8_t retrys;
    uint32_t starttime;
    UInt eventId;
    uint8_t acklen;
    IB_Packet RxPacket,*curRxPacket,curTxPacket;
    IB_Ack_Status ackStateTemp;



    retrys = 0;
    starttime = Clock_getTicks();
RX_ENTRY:
    if(!InterBowIsPowerOn){
        InterBow_CurState = INTERBOW_IDLE;
        InterHwFxn.Sleep();
        return ;//关机，退出
    }

    if(InterBow_GetPassTime(starttime) >= (timeout))//接收监听时间超过一个周期，认为网关可能忙碌或其他异常。
    {
        System_printf("RX TIMEOUT EXIT\n");
        System_flush();
        Led_ctrl(LED_R, 1, 20* CLOCK_UNIT_MS, 3);
        return ;//
    }


    InterBow_CurState = INTERBOW_RX_DOING;
    IB_SetDownFreq();
    InterHwFxn.RxMode();                  //进入接收状态,
    //4.收到应答或超时
    eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_RX_DONE, (10*packettime)*CLOCK_UNIT_MS);
    if(eventId != INTERBOW_EVT_RX_DONE)
    {
        System_printf("ACK TIMEOUT tmpdevchnno = %d\n", tmpdevchnno);
        System_flush();
        goto RX_ENTRY;
    }

    //5.接收并分析应答数据
    acklen = InterHwFxn.ReadRxPacket((uint8_t*)&RxPacket,sizeof(RxPacket));
    if(acklen == 0)
        return;

    curRxPacket = &RxPacket;
    //CmdPos = 0;

    if (curRxPacket->length < RX_PACKET_HEAD_LEN || acklen < curRxPacket->length){
        System_printf("ACK LEN ERR acklen=%d  curRxPacket->length=%d\n", acklen, curRxPacket->length);
        System_flush();
        return;
    }

#ifdef USE_CHECKCODE
    //软件检测校验码是否正常
    if(curRxPacket->crc != CRC16((uint8_t*)curRxPacket + CHECKCODE_LEN,curRxPacket->length - CHECKCODE_LEN))
    {
        System_printf("CHK CODE ERR\n");
        System_flush();
        return;
    }
#endif

#ifdef SUPPORT_AES
    if(curRxPacket->encrypt)
    {
        struct AES_ctx ctx;

        AES_init_ctx_iv(&ctx, aes128Key, aes128Iv);
        AES_CBC_decrypt_buffer(&ctx, curRxPacket->payload, curRxPacket->length-RX_PACKET_HEAD_LEN);
    }
#endif //SUPPORT_AES

    if( curRxPacket->custID != *(uint16_t*)IBConfig.customId){
        System_printf("ERR custID = %08lx\n", curRxPacket->custID);
        System_flush();
        return;
    }

    if(curRxPacket->dstAddr != *(uint32_t*)IBConfig.DeviceId){//是发射给本设备的数据包
        System_printf("ERR dstAddr = %08lx\n", curRxPacket->dstAddr);
        System_flush();
        return;
    }

    if((curRxPacket->length >= RX_PACKET_HEAD_LEN) && (curRxPacket->length <= acklen)){

        if(curRxPacket->cmd==IMI_TX_SET){
            ackStateTemp.status = 0;
            if(interBowNodeCb.RecvData)
                ackStateTemp.ack = interBowNodeCb.RecvData(curRxPacket->payload, curRxPacket->length-RX_PACKET_HEAD_LEN);

            //=======================================================================
            //ready for send ack
            curTxPacket.payload[0] = ackStateTemp.status;// succ state flag
            curdstAddr = curRxPacket->srcAddr;
            IB_group_package(IMI_TX_ACK, &curTxPacket);
        }
        else if(curRxPacket->cmd==IMI_TX_CTL){
            curRxPacket->busy = 0;// 网关设置完成，终止下发数据
        }
        else{
            System_printf("Not Setting CMD\n");
            System_flush();
            return;
        }
    }
    else{
       return;
    }

TX_RETRY:
    if(retrys++ < 3)
    {
        InterBow_CurState = INTERBOW_TX_DOING;
        IB_SetUpFreq();
        InterHwFxn.SendPacket((uint8_t*)&curTxPacket,curTxPacket.length);

        //等待发送完成
        eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_TX_DONE, packettime*CLOCK_UNIT_MS);
        if(eventId != INTERBOW_EVT_TX_DONE)
        {
            System_printf("SEND ERR\n");
            System_flush();
            InterHwFxn.Reset();
            Led_ctrl(LED_R, 1, 10* CLOCK_UNIT_MS, 1);
            goto TX_RETRY;
        }
    }

    Led_ctrl(LED_B, 1, 10* CLOCK_UNIT_MS, 1);
    if(curRxPacket->busy)
        goto RX_ENTRY;
}

#endif //S_C

void InterBow_SetBusy(void)
{
    isOccupyBusy = 1;
}

bool InterBow_GetBusyState(void)
{
    return curisbusy;
}

void InterBow_EnterConfigMode(void)
{
    if(InterBowEvtHandle)
        Event_post(InterBowEvtHandle, INTERBOW_EVT_CONFIG_MODE);
}


void InterBow_ExitConfigMode(void)
{
    if(InterBowEvtHandle)
        Event_post(InterBowEvtHandle, INTERBOW_EVT_EXIT_CONFIG);
}
//***********************************************************************************
//
//InterBow_MasterThreadFxn  function, for master send task.
//
//***********************************************************************************
#ifdef S_C
static void InterBow_MasterThreadFxn()
{
    UInt eventId;
    IB_Packet curTxPacket;
    static uint8_t sendfails = 0;   
    

    eventId = Event_pend(InterBowEvtHandle, 0, INTERBOW_EVT_POWERON | INTERBOW_EVT_POWEROFF | INTERBOW_EVT_TX_DO, BIOS_WAIT_FOREVER);

    //0.判断是否开关机
    if(eventId & INTERBOW_EVT_POWEROFF)
    {
        //if(!InterBowIsPowerOn)return;
        
        InterBowIsPowerOn = false;
        
        InterHwFxn.Sleep();
        
        Clock_stop(InterBowSend_ClkHandle); 

        return;
    }
    
    if(eventId & INTERBOW_EVT_POWERON)
    {
        if(InterBowIsPowerOn)return;
        
        InterBowIsPowerOn = true;
        
        //Clock_start(InterBowSend_ClkHandle);          
        ntp = 0;//开始注册
    }
    
#ifdef LORA_OOK_TEST    
    InterBow_master_test_ook(&curTxPacket);
#endif

    curisbusy = 0;

    if(InterBow_master_register(&curTxPacket))
        return;
        
SEND_NEXT:  
    if(!InterBowIsPowerOn)return;

    isOccupyBusy = 0;

    
    if(IB_group_package(IMI_TX_DATA,&curTxPacket) == 0)
        return;

    if (InterBow_SendData(&curTxPacket, IBConfig.uploadPeriod*CLOCK_UNIT_S)){

        sendfails = 0;//reset
        
        //由于每个设备的timer有偏差，需要每次发送数据后同步与网关的时间重设timer
        if ( isOccupyBusy == 0){//最后一条，同步下时间

            InterBow_SyncSendTime();  
            if((curisbusy != 0)){ //但网关仍然返回busy状态
                 InterBow_MasterKeepConnect(60*CLOCK_UNIT_S);
            }
            if(nodeLastBusyState)
                goto SEND_NEXT;//采用抢占模式，一直发送到完成。
            return;//最后一条已通知网关没有数据，不再发送。
        } 
        
        goto SEND_NEXT;//采用抢占模式，一直发送到完成。
        
    }
    else {     
        System_printf("SEND DATA ERR sendfails=%d\n", sendfails);
        System_flush();        
        sendfails++;
        if( !RxAckOK || sendfails >= 60){//1个周期没有收到任何正确的应答或通讯失败60分钟，可能网关出现问题，重新注册。      
            ntp = 0;
            Clock_stop(InterBowSend_ClkHandle);
            InterBow_master_register(&curTxPacket);
            sendfails = 0;//reset
            return;//注册完后不发送数据，等待时间通道发送。      
        }
        goto SEND_NEXT;//失败后立即发送。      
        //return;//失败后不发送数据，等待时间通道发送。              
    }

}
#endif
bool keepConnect=0;
//***********************************************************************************
//
// InterBow_SlaverThreadFxn   function, for slaver receive and ack task..
//
//***********************************************************************************
static void InterBow_SlaverThreadFxn()
{
    uint8_t retrys=0;
    UInt eventId;
    uint8_t acklen;
    IB_Packet curTxPacket;
    bool FailFlag = 0;
    uint8_t index = 0, *rxbuffer;




    static int8_t  busy_errs = 0;
    IB_Ack_Status* ackStateP;

START_RX:
    eventId = Event_pend(InterBowEvtHandle, 0, INTERBOW_EVT_POWERON | INTERBOW_EVT_POWEROFF | INTERBOW_EVT_RX_DONE | INTERBOW_EVT_CONFIG_MODE | INTERBOW_EVT_EXIT_CONFIG, curisbusy?(10*packettime*CLOCK_UNIT_MS):(BIOS_WAIT_FOREVER));

    if (curisbusy == 0 && eventId == 0){// add by younger@201904012 for 接收时间超过10个周期，可能lora模块异常,重新初始化。
        if(rSysTask.state == SYS_STATE_POWERON)
        {
            InterHwFxn.Reset();
            InterHwFxn.RxMode();//进入接收状态,
        }
        else
        Task_sleep(1000*CLOCK_UNIT_MS);

        goto START_RX;
    }


#ifdef SUPROT_GATE_SET
    Txset = 0;
#endif
#ifdef DEBUG
    //System_printf("RX...\n");
    //System_flush();
#endif
    
    if(eventId == 0){//超时，表示处于busy状态时一定时间片里没有收到数据，关闭busy状态,并通知busy的采集器断开。

        System_printf("BUSY TIMEOUT, curdstAddr = %08lx\n", curdstAddr);
        System_flush();
        keepConnect = 0;
        isOccupyBusy = 0;
        curTxPacket.busy = curisbusy = 0; 
        if(interBowVer == INTERBOW_V20)
            IB_group_package(IMI_TX_CTL, &curTxPacket);
        curdstAddr = 0;
        retrys = 0;        
        
        goto FAIL_RETRY;
    }

    //0.判断是否开关机
    if(eventId & INTERBOW_EVT_POWEROFF)
    {
        //if(!InterBowIsPowerOn)return;
        
        InterBowIsPowerOn = false;
        
        InterHwFxn.Sleep();
        
        return;
    }

    if(eventId & INTERBOW_EVT_POWERON)
    {
        if(InterBowIsPowerOn)return;

        InterBowIsPowerOn = true;
    

        Clock_start(InterBowRegManager_ClkHandle);

#ifdef SUPPORT_MUTI_GATEWAY
        //搜索存在的网关，将本网关注册到不存在或信号最弱的频点
        InterBow_detect_and_fix_freq(&curTxPacket);
#endif

        InterBow_CurState = INTERBOW_RX_DOING;
        IB_SetUpFreq();
        InterHwFxn.RxMode();                  //进入接收状态,

        return;
    }

    if(eventId & INTERBOW_EVT_CONFIG_MODE){
        IBConfig.curFreq      = LORA_FREQ_CONFIG;
        InterHwFxn.RateModeSet(LORA_RATE_0);
        InterHwFxn.GetHwConfig(&IBHwConfig);
        InterBow_CalcPacketTime();//BW & SF更新，更新时间片     

        InterBow_CurState = INTERBOW_RX_DOING;
        IB_SetUpFreq();
        InterHwFxn.RxMode();                  //进入接收状态,
        return;
    }


    if(eventId & INTERBOW_EVT_EXIT_CONFIG){
        InterHwFxn.RateModeSet(LORA_RATE_USER);
        InterHwFxn.GetHwConfig(&IBHwConfig);
        SX1276_SF_BW_Set();
        InterBow_CalcPacketTime();//BW & SF更新，更新时间片 
        InterHwFxn.Reset(); 

#ifdef SUPPORT_MUTI_GATEWAY
        //搜索存在的网关，将本网关注册到不存在或信号最弱的频点
        InterBow_detect_and_fix_freq(&curTxPacket);
#endif

        InterBow_CurState = INTERBOW_RX_DOING;
        IB_SetUpFreq();
        InterHwFxn.RxMode();                  //进入接收状态,
        return;
    }

    if(!InterBowIsPowerOn)return;


#ifdef LORA_FSK_TEST
    //Receive_OOK();
    Receive_FSK();
    while(1){
        Event_pend(InterBowEvtHandle, 0, INTERBOW_EVT_RX_DONE , (BIOS_WAIT_FOREVER));
        LoraFskStandBy();
        if(FSK_Rx_PayloadCrcOk()){

        }
        ReGotoFskRx();
    }
    return;
#endif



    //1.收到数据    
    //Led_ctrl(LED_G, 1, 100* CLOCK_UNIT_MS, 1);    
    InterHwFxn.Standby();
    
    last_rssi = InterHwFxn.GetRssi(); 
    
    //2.接收并处理数据
    acklen = InterHwFxn.ReadRxPacket((uint8_t*)&curTxPacket,sizeof(curTxPacket));

    if(acklen == 0){//crc error
        System_printf("CRC ERR\n");
        System_flush();
        FailFlag = 1;
        goto FINISH;
    }
          
    if(acklen != curTxPacket.length ||//data length error  当前只考虑sensor每次只发一个命令情况。        
       curTxPacket.length  < TX_PACKET_HEAD_LEN ) {//sensor send: length(1BYTE) cmd(1BYTE) srcddr (4BYTE) dstddr(4BYTE) ...
        System_printf("LEN ERR acklen=%d Txlen=%d\n",acklen, curTxPacket.length);
        System_flush();        
        FailFlag = 1;
        goto FINISH;
    }
    

#ifdef USE_CHECKCODE 
    //软件检测校验码是否正常    
    if(curTxPacket.crc != CRC16((uint8_t*)&curTxPacket + CHECKCODE_LEN,curTxPacket.length - CHECKCODE_LEN))
    {
        System_printf("CHK CODE ERR\n");
        System_flush();        
        FailFlag = 1;
        goto FINISH;
    } 

#endif


#ifdef SUPPORT_AES
    if(curTxPacket.encrypt)
    {
        struct AES_ctx ctx;

        AES_init_ctx_iv(&ctx, aes128Key, aes128Iv);
        AES_CBC_decrypt_buffer(&ctx, curTxPacket.payload, curTxPacket.length-RX_PACKET_HEAD_LEN);
    }
#endif //SUPPORT_AES

#ifdef DEBUG_RX
    static uint32_t  lasttick,curtick;
    curtick = Clock_getTicks();
    
    System_printf("tick=%08ld diff=%08ld srcAdd = %08lx  dstAddr = %08lx  busy = %d chnNo=%d\n", curtick, curtick - lasttick,curTxPacket.srcAddr, curTxPacket.dstAddr, curTxPacket.busy, curTxPacket.chnNo);
    System_flush();  
    lasttick = Clock_getTicks();
    goto FINISH;
#endif

    radioSerialNumAck = curTxPacket.serialNum;
    //目标地址必须为网关地址或0 ,客户码必须一致,否则有误
    if(curTxPacket.cmd != IMI_TX_SGN){  //不是信号检测命令，才需要检查客户码是否一致

        if ((curTxPacket.custID  != *(uint16_t*)IBConfig.customId) ||
              ((curTxPacket.dstAddr != *(uint32_t*)IBConfig.DeviceId) &&(curTxPacket.dstAddr != 0)))
          {
              System_printf("ADD ERR dstAddr = %08lx srcAddr = %08lx\n", curTxPacket.dstAddr, curTxPacket.srcAddr);
              System_flush();
              FailFlag = 1;
              goto FINISH;
          }

    }
    else if( (curTxPacket.dstAddr != 0) && (curTxPacket.dstAddr != *(uint32_t*)IBConfig.DeviceId) ){ //信号检测命令  一定是广播或者绑定的网关地址，如果不是则错误
        System_printf("ADD ERR3 dstAddr = %08lx \n", curTxPacket.dstAddr);
        System_flush();
        FailFlag = 1;
        goto FINISH;

    }


    //收到正确数据，开始处理

    //广播消息只能用于注册或信号检测,否则异常
    if ((curTxPacket.dstAddr == 0 ) && (curTxPacket.cmd != IMI_TX_REG) && (curTxPacket.cmd != IMI_TX_SGN)){
        System_printf("ADD ERR2 cmd=%d \n", curTxPacket.cmd);
        System_flush();        
        FailFlag = 1;
        goto FINISH;
    }

    //only register 80%
    if( (curTxPacket.dstAddr != 0) && (curTxPacket.cmd != IMI_TX_SGN)){
        if(GetDevChnno(curTxPacket.srcAddr) > (maxdevnum *8/10))
          {
            System_printf("node is over 80%% maxdevnum:%d nowNum:%d",maxdevnum,GetDevChnno(curTxPacket.srcAddr));
            System_flush();
            FailFlag = 1;
            goto FINISH;
          }
    }

    if(curisbusy){//如果通道处于占用状态，只接收当前采集器的数据。
        if( curdstAddr != curTxPacket.srcAddr){

            if(++busy_errs > 10){//如果连续占用错误出现10次，认为异常，关闭busy状态,并通知busy的采集器断开。
            
                System_printf("BUSY 10 TIMES, curdstAddr=%08lx\n", curdstAddr);
                System_flush();

                curTxPacket.busy = curisbusy = 0;
                if(interBowVer == INTERBOW_V20)
                    IB_group_package(IMI_TX_CTL, &curTxPacket);

                curdstAddr = 0;
                retrys = 0;                
                FailFlag = 1;
                goto FAIL_RETRY;
            }
            else{
                System_printf("ADD BUSY srcAddr=%08lx\n", curTxPacket.srcAddr);
                System_flush();        
                FailFlag = 1;
                goto FINISH;
            }
        }
    }

    
    //记录当前采集器的ID及通道占用状态。
    curdstAddr = curTxPacket.srcAddr;
    curisbusy  = curTxPacket.busy;
    curTxcmd   = curTxPacket.cmd;
    busy_errs  = 0;//reset busyerrs。
    ackStateP  = (IB_Ack_Status*)curTxPacket.payload;
#ifdef DEBUG
    System_printf("curdstAddr =%08lx  busy = %d chnNo=%d\n", curdstAddr, curisbusy, curTxPacket.chnNo);
    System_flush();  
#endif

    isOccupyBusy = 0;

    index = 0;
    if(curTxPacket.paraEx)
    {
        index++;
        while(curTxPacket.payload[index-1] & 0x80){
            index++;
        }
    }
    rxbuffer = curTxPacket.payload + index;

    switch(curTxcmd){        
    case IMI_TX_DATA:
        index = 0;
        nodeLastBusyState = curTxPacket.busy;
        //ready for send ack
        if(interBowGatewayCb.RecvData)
            RxAckOK = interBowGatewayCb.RecvData(rxbuffer, curTxPacket.length-TX_PACKET_HEAD_LEN, last_rssi, curdstAddr, curisbusy, &isOccupyBusy);

        keepConnect = 0;
        interBowVer = INTERBOW_V20;
        IB_group_package(IMI_RX_ACK, &curTxPacket);
    break;


    case IMI_TX_REG:
        if(interBowGatewayCb.RecvRegistering)
            interBowGatewayCb.RecvRegistering(rxbuffer, curTxPacket.length-TX_PACKET_HEAD_LEN, last_rssi, curdstAddr);
        //ready for send ack                
        interBowVer = INTERBOW_V20;
        if(rxbuffer[0] == PROTOCAL_VERSION && rxbuffer[1] == 0)  {
            IB_group_package(IMI_RX_NTP, &curTxPacket);
        }
        else {
            FailFlag = 1;
            goto FINISH;
        }
        //ready for send ack             
        break;

    case IMI_TX_SGN:
        keepConnect = 0;
        if(interBowGatewayCb.RecvSingleTest)
            interBowGatewayCb.RecvSingleTest(rxbuffer, curTxPacket.length-TX_PACKET_HEAD_LEN, last_rssi, curdstAddr);
        //ready for send ack
        interBowVer = INTERBOW_V20;
        curTxPacket.payload[0] = 0;
        ackStateP      = (IB_Ack_Status *)curTxPacket.payload;
        ackStateP->cmd = curTxcmd;
        ackStateP->ack = 0;
        curTxPacket.payload[1] = curTxPacket.serialNum;// serial num
        curTxPacket.payload[2] = (int8_t)(last_rssi +164);//ACK时把接收采集器的发射信号强度返回,以便采集器了解发射信号强度进行APC 
        IB_group_package(IMI_RX_ACK, &curTxPacket);
        break;

   case IMI_TX_ACK:
        if(interBowGatewayCb.RecvDataAck)
            interBowGatewayCb.RecvDataAck(rxbuffer, curTxPacket.length-TX_PACKET_HEAD_LEN, last_rssi, curdstAddr);
        FailFlag = 0;
        break;
    default:
        //Led_ctrl(LED_R, 1, 100* CLOCK_UNIT_MS, 1);
        FailFlag = 1;
        goto FINISH;
    }

    delay_us(2000);

    //3.发送ACK       
#ifdef S_G
    if(keepConnect){
        goto KeepConnectProcess;
    }
#endif //S_G
FAIL_RETRY:
    if(retrys++ < 3)
    {   
        InterBow_CurState = INTERBOW_TX_DOING;
        IB_SetDownFreq();
        InterHwFxn.SendPacket((uint8_t*)&curTxPacket,curTxPacket.length);

        //等待发送完成
        eventId = InterBow_pend(InterBowEvtHandle, 0, INTERBOW_EVT_TX_DONE, packettime*CLOCK_UNIT_MS);
        if(eventId != INTERBOW_EVT_TX_DONE)
        {   
            System_printf("SEND ERR\n");
            System_flush();
            InterHwFxn.Reset();
            Led_ctrl(LED_R, 1, 10* CLOCK_UNIT_MS, 1);
            goto FAIL_RETRY;
        }    
    }
    
    Led_ctrl(LED_B, 1, 10* CLOCK_UNIT_MS, 1);
    if(keepConnect){    //在网关设置（主动发送）时，跑到该位置已经发送完成可以返回了
        FailFlag = 0;
        goto FINISH;
    }

#ifdef S_G

KeepConnectProcess:



    if(isOccupyBusy && keepConnect == 0){
        keepConnect = 1;
        Task_sleep(500*CLOCK_UNIT_MS);//延时500ms 让采集器进入RX准备好
    }

    if(keepConnect){   //网关占用通道资源，继续与该节点保持连接

        if(isOccupyBusy) {
            IB_group_package(IMI_TX_SET, &curTxPacket);
            curisbusy = 1;
            retrys = 0;
        }
        else{
            keepConnect = 0;
            curisbusy   = nodeLastBusyState;
            IB_group_package(IMI_TX_SET, &curTxPacket);
            retrys = 0;
        }
        goto FAIL_RETRY;

    }
#endif //S_G
    //4.等待数据    

FINISH:

    if(FailFlag){
        Led_ctrl(LED_R, 1, 10* CLOCK_UNIT_MS, 1);
    }

    
    InterBow_CurState = INTERBOW_RX_DOING;
    IB_SetUpFreq();
    InterHwFxn.RxMode();                  //进入接收状态,
    
}


static void InterBow_RecvSendFxn()
{
    #ifdef LORA_FSK_TEST
    //Receive_OOK();
    Receive_FSK();
    while(1){
        Event_pend(InterBowEvtHandle, 0, INTERBOW_EVT_RX_DONE , (BIOS_WAIT_FOREVER));
        LoraFskStandBy();
        if(FSK_Rx_PayloadCrcOk()){

        }
        ReGotoFskRx();
    }
    return;
#endif

#ifdef S_C
    if(IBConfig.asMaster)
    {
        while(1){
            InterBow_MasterThreadFxn();
            InterHwFxn.Sleep();//发送完成，进入sleep模式省电。
            Event_pend(InterBowEvtHandle, 0,  INTERBOW_EVT_TX_DO, BIOS_NO_WAIT);//clear event
        }
    }
    else
#endif        
    {
        while(1){
            InterBow_SlaverThreadFxn();
        }
    }

}


void InterBow_Isr(void)
{
    if(InterBow_CurState == INTERBOW_TX_DOING)
    {
        InterBowTxDoneEvtSet();
    }
    if(InterBow_CurState == INTERBOW_RX_DOING)
    {           
        InterBowRxDoneEvtSet();
    }
    if(InterBow_CurState == INTERBOW_CAD_DOING)
    {           
        InterBowCadDoneEvtSet();
    }
}


void InterBow_Init(IB_config_t *configPara)
{
    Error_Block eb;

    Error_init(&eb);

    InterHwFxn.RateModeSet(LORA_RATE_USER);
    InterHwFxn.GetHwConfig(&IBHwConfig);

    IBConfig = *configPara;
    
    IBConfig.curFreqchannel = IBConfig.curFreqchannel % IBHwConfig.freqMaxchannel;

    if(IBConfig.rateMode >= IBHwConfig.rateMax)
        IBConfig.rateMode = IBHwConfig.rateMax - 1;

    if(IBConfig.powerConfig >= IBHwConfig.powerConfigMax)
        IBConfig.powerConfig = IBHwConfig.powerConfigMax;

    IBConfig.curFreq = IBHwConfig.baseFreq + IBConfig.curFreqchannel * IBHwConfig.baseFreqOffset;


    InterHwFxn.Init(InterBow_Isr);


    InterBow_CurState = INTERBOW_IDLE;

    /* Construct lora  process Event */
    Event_construct(&InterBowLoraEvtStruct, NULL);
    /* Obtain event instance handle */
    InterBowEvtHandle = Event_handle(&InterBowLoraEvtStruct);

    /* Construct main system process Task threads */
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = INTERBOWTASKSTACKSIZE;
    taskParams.stack = &InterBowTaskStack;
    taskParams.priority = 2;//lora优先级必须比较高，否则可能影响通讯。
    Task_construct(&InterBowTaskStruct, (Task_FuncPtr)InterBow_RecvSendFxn, &taskParams, &eb);
    

#ifdef S_C
    if(IBConfig.asMaster)
    {
        /* Construct a 1 mintue periodic Clock Instance  to lora send*/
        Clock_Params clkParams;
        Clock_Params_init(&clkParams);
        clkParams.period = 0;//60 * CLOCK_UNIT_S;
        clkParams.startFlag = FALSE;
        //use collectPeriod as lora send Period
        Clock_construct(&InterBowSend_ClkStru, (Clock_FuncPtr)InterBow_SendFxn, 
            IBConfig.uploadPeriod*CLOCK_UNIT_S, &clkParams);
        
        InterBowSend_ClkHandle = Clock_handle(&InterBowSend_ClkStru);
    }
#else
    {
        //Construct a 10 mintue periodic Clock Instance  to lora register manager*/
        Clock_Params clkParams;
        Clock_Params_init(&clkParams);
        clkParams.period = 3 * IBConfig.uploadPeriod * CLOCK_UNIT_S;
        clkParams.startFlag = true;
        //use collectPeriod as lora send Period
        Clock_construct(&InterBowRegManager_ClkStruct, (Clock_FuncPtr)InterBow_RegManagerFxn,
                        30 * 60 * CLOCK_UNIT_S, &clkParams);

        InterBowRegManager_ClkHandle = Clock_handle(&InterBowRegManager_ClkStruct);
    }

#endif  
    
    InterBow_CalcPacketTime();//计算时间片

    devchnno = 0;
    memset(DevChnList,0x00,sizeof(ChnnoManage_t)*DevChnNUM);
}


void InterBow_PowerOn()
{
    if(InterBowEvtHandle)
        Event_post(InterBowEvtHandle, INTERBOW_EVT_POWERON);    
}


void InterBow_PowerOff()
{
    if(InterBowIsPowerOn){
        InterBowIsPowerOn = false;
        if(InterBowEvtHandle)
            Event_post(InterBowEvtHandle, INTERBOW_EVT_POWEROFF);   
    }
}

uint8_t InterBow_GetNTP()
{
    return ntp;
}

uint8_t* InterBow_GetGateID()
{
    return (uint8_t*)&curdstAddr;
}

uint16_t InterBow_GetChnNo()
{
    return devchnno;
}

uint16_t InterBow_GetLinkNo()
{
    uint16_t regdevCount = 0,i = 0;

    for(i = 1; i < DevChnNUM; i++){
        if( DevChnList[i].DevID == 0 )break;

        if(DevChnList[i].active_flag != CHNO_IS_UNACTIVED){
            regdevCount ++;
        }
    }
    return regdevCount;
}

uint32_t InterBow_GetFreq()
{
    return IBConfig.curFreq;
}

uint8_t InterBow_GetSF()
{
    return G_LoRaConfig.SpreadingFactor;
}

uint8_t InterBow_GetBandWidth()
{
    return (uint8_t)G_LoRaConfig.BandWidth;
}

int16_t InterBow_GetLastRssi()
{
    return last_rssi;
}

void InterBow_GateCbRegister(InterBow_Gateway_Cb_t *gatewayCb)
{
    interBowGatewayCb = *gatewayCb;
}

void InterBow_NodeCbRegister(InterBow_Node_Cb_t *nodeCb)
{
    interBowNodeCb = *nodeCb;
}

#endif //SUPPORT_RADIO


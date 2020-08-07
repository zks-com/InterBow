//***********************************************************************************
// Copyright 2017, Zksiot Development Ltd.
// Created by younger.fu, 2017.09.25
// MCU: MSP430F5529
// OS: TI-RTOS
// Project:
// File name: lora.c
// Description: lora function routine header.
//***********************************************************************************
#ifndef INTERBOW_H
#define INTERBOW_H


#define  TXDATA_MAX_SIZE (128-16)
#define  ACK_MAX_SIZE                32

#define     INTERBOW_USER_RATE      0xff

#define     INTERBOW_INVALID_CHN    0xffff


enum  INTERBOW_STATE{
    INTERBOW_IDLE,
    INTERBOW_TX_DOING,
    INTERBOW_TX_DONE,   
    INTERBOW_RX_DOING,
    INTERBOW_RX_DONE,
    INTERBOW_CAD_DOING, 
    INTERBOW_CAD_DONE,  
};

#pragma pack (1)
typedef struct 
{
    uint32_t curFreq;           //当前频率

    uint8_t  curFreqchannel;    //当前频率通道

    uint8_t  DeviceId[4];       //
    uint8_t  customId[2];       //
    uint32_t uploadPeriod;      //数据上传周期，采用网关的采集周期。
 
    uint8_t  powerConfig;       //当前无线发射功率   
    uint8_t  rateMode;          //速率模式
    uint8_t  apcEnable;         //自动功率控制
    uint8_t  rateFixEnable;     //rateMode和频点固定不变
    uint8_t  asMaster;          //
    uint8_t  BindGateway[4];    //绑定的网关，不绑定网关赋0
}IB_config_t;


typedef struct 
{
    int32_t  baseFreq;          //基础频率
    int32_t  baseFreqOffset;    //相邻频点偏差
    int32_t  offsetUpFreq;      //上行频率偏移
    int32_t  offsetDownFreq;    //下行频率偏移
    uint8_t  freqMaxchannel;    //频率通道最大数

    uint8_t  powerConfigMax;    //最大无线发射功率

    uint8_t  rateMax;           //速率模式最大值
}IB_hw_config_t;

#pragma pack ()


// ****************************************************************PORT**********************************************
typedef struct 
{
    //节点接收到信号反馈命令，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // 返回：    无    
    void (*RecvSingleTestAck)(uint8_t * data, uint8_t len);

    //节点接收到注册反馈命令，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // 返回：    无
    void (*RecvRegisteringAck)(uint8_t * data, uint8_t len);     

    //节点接收到网关的用户数据接收情况反馈，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // 返回：    无
    void (*RecvDataAck)(uint8_t * data, uint8_t len);

    //节点接收到网关发送的用户数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // 返回:      0x00: 数据解析正确
    //            0x01: 数据解析异常
    uint8_t (*RecvData)(uint8_t * data, uint8_t len);

    //节点接收到网关发送控制命令，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // 返回：    无
    void (*RecvControlData)(uint8_t * data, uint8_t len);


    //节点发送信号检测命令时，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendSingleTest)(uint8_t * data, uint8_t maxLen);

    //节点发送信号检测命令时，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendRegistering)(uint8_t * data, uint8_t maxLen);

    //节点打包用户层数据进行发送
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // *pending： 节点如需保持连接，将*pending 赋值为1
    // 返回：    打包的长度
    uint8_t (*SendData)(uint8_t * data, uint8_t maxLen, uint8_t *pending);

    //节点收到网关发送的用户数据命令，进行反馈，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendDataAck)(uint8_t * data, uint8_t maxLen);

    //节点收到网关发送的控制命令，进行反馈，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendControlAck)(uint8_t * data, uint8_t maxLen);
}InterBow_Node_Cb_t;


typedef struct 
{
    //网关接收到信号检测命令，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // rssi:    无线信号强度
    // srcAddr: 节点地址
    // 返回：    无  
    void (*RecvSingleTest)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //网关接收到注册命令，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // rssi:    无线信号强度
    // srcAddr: 节点地址
    // 返回：    无  
    void (*RecvRegistering)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //网关接收到用户层数据命令
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // rssi:    无线信号强度
    // srcAddr: 节点地址
    // busy:    节点是否还要继续发送数据，仅在busy为0是，网关才可赋值pending为1,进行保持
    //          连接，网关可持续下发数据给采集器
    // 返回:      0x00: 数据解析正确
    //            0x01: 数据解析异常
    uint8_t (*RecvData)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr, uint8_t busy, uint8_t *pending);

    //兼容V10版本协议网关接收到用户层数据命令
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // rssi:    无线信号强度
    // 返回:      0x00: 数据解析正确
    //            0x01: 数据解析异常
    uint8_t (*RecvData_V10)(uint8_t * data, uint8_t len, int16_t rssi);

    //网关接收到节点的用户数据接收情况反馈，附带的用户层数据
    // data:    附带的用户数据指针
    // len:     用户数据长度
    // rssi:    无线信号强度
    // srcAddr: 节点地址
    // 返回：    无  
    void (*RecvDataAck)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //网关反馈信号检测命令时，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendSingleTestAck)(uint8_t * data, uint8_t maxLen);

    //网关反馈注册命令时，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendRegisteringAck)(uint8_t * data, uint8_t maxLen);

    //网关反馈数据发送命令时，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendDataAck)(uint8_t * data, uint8_t maxLen);

    //网关发送数据命令，打包用户层数据，网关在打包该包数据时，可调用InterBow_SetBusy()使连接一直保持
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendData)(uint8_t * data, uint8_t maxLen, uint32_t dstDevId);

    //网关发送控制命令，可附带打包用户层数据
    // data:    附带的用户数据指针
    // maxLen:  可打包的最大长度
    // 返回：    打包的长度
    uint8_t (*SendControlData)(uint8_t * data, uint8_t maxLen);
}InterBow_Gateway_Cb_t;




typedef struct 
{
// 硬件射频：MODULE_LORA, MODULE_CC1310
    uint32_t radioType;
// 射频硬件休眠函数
    void (*Sleep)(void);
// 射频硬件standby函数
    void (*Standby)(void);
// 射频硬件初始化函数
    void (*Reset)(void);
// 射频硬件进入接收模式
    bool (*RxMode)(void);
// 射频硬件硬件初始化
    void (*Init)(void (*Cb)(void));
// 获取射频硬件参数
    void (*GetHwConfig)(IB_hw_config_t* config);
// 射频硬件频点设置
    void (*FreqSet)(uint32_t freq);
// 射频硬件速率模式设置
    void (*RateModeSet)(uint8_t rateMode);
// 射频硬件设置无线发射功率
    void (*RfPowerSet)(uint8_t rfPower);
// 射频硬件读取无线数据
    uint8_t (*ReadRxPacket)(uint8_t* buf, uint8_t len);
// 射频硬件发送无线数据
    bool (*SendPacket)(uint8_t* buf, uint8_t len);
// 射频硬件读取无线信号强度
    int16_t (*GetRssi)(void);
}Interbow_Hw_FxnTable; 



extern void InterBowTxDoneEvtSet(void);
extern void InterBowRxDoneEvtSet(void);
extern void InterBowCadDoneEvtSet(void);
extern void InterBow_Init();
extern void InterBow_PowerOn();
extern void InterBow_PowerOff();
extern uint8_t InterBow_GetNTP();
extern uint8_t* InterBow_GetGateID();
extern uint32_t InterBow_GetFreq();
extern uint8_t InterBow_GetSF();
extern uint8_t InterBow_GetBandWidth();

extern uint16_t InterBow_GetChnNo();
extern uint16_t InterBow_GetLinkNo();
extern void InterBow_SyncSendTime();
extern void InterBow_SetBusy(void);
extern void InterBow_GateCbRegister(InterBow_Gateway_Cb_t *gatewayCb);
extern void InterBow_NodeCbRegister(InterBow_Node_Cb_t *nodeCb);
extern void InterBow_EnterConfigMode(void);
extern void InterBow_ExitConfigMode(void);

extern uint16_t InterBow_CheckNode(uint32_t devid);


// ****************************************************************PORT**********************************************

#endif

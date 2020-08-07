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
    uint32_t curFreq;           //��ǰƵ��

    uint8_t  curFreqchannel;    //��ǰƵ��ͨ��

    uint8_t  DeviceId[4];       //
    uint8_t  customId[2];       //
    uint32_t uploadPeriod;      //�����ϴ����ڣ��������صĲɼ����ڡ�
 
    uint8_t  powerConfig;       //��ǰ���߷��书��   
    uint8_t  rateMode;          //����ģʽ
    uint8_t  apcEnable;         //�Զ����ʿ���
    uint8_t  rateFixEnable;     //rateMode��Ƶ��̶�����
    uint8_t  asMaster;          //
    uint8_t  BindGateway[4];    //�󶨵����أ��������ظ�0
}IB_config_t;


typedef struct 
{
    int32_t  baseFreq;          //����Ƶ��
    int32_t  baseFreqOffset;    //����Ƶ��ƫ��
    int32_t  offsetUpFreq;      //����Ƶ��ƫ��
    int32_t  offsetDownFreq;    //����Ƶ��ƫ��
    uint8_t  freqMaxchannel;    //Ƶ��ͨ�������

    uint8_t  powerConfigMax;    //������߷��书��

    uint8_t  rateMax;           //����ģʽ���ֵ
}IB_hw_config_t;

#pragma pack ()


// ****************************************************************PORT**********************************************
typedef struct 
{
    //�ڵ���յ��źŷ�������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // ���أ�    ��    
    void (*RecvSingleTestAck)(uint8_t * data, uint8_t len);

    //�ڵ���յ�ע�ᷴ������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // ���أ�    ��
    void (*RecvRegisteringAck)(uint8_t * data, uint8_t len);     

    //�ڵ���յ����ص��û����ݽ�������������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // ���أ�    ��
    void (*RecvDataAck)(uint8_t * data, uint8_t len);

    //�ڵ���յ����ط��͵��û�����
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // ����:      0x00: ���ݽ�����ȷ
    //            0x01: ���ݽ����쳣
    uint8_t (*RecvData)(uint8_t * data, uint8_t len);

    //�ڵ���յ����ط��Ϳ�������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // ���أ�    ��
    void (*RecvControlData)(uint8_t * data, uint8_t len);


    //�ڵ㷢���źż������ʱ���ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendSingleTest)(uint8_t * data, uint8_t maxLen);

    //�ڵ㷢���źż������ʱ���ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendRegistering)(uint8_t * data, uint8_t maxLen);

    //�ڵ����û������ݽ��з���
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // *pending�� �ڵ����豣�����ӣ���*pending ��ֵΪ1
    // ���أ�    ����ĳ���
    uint8_t (*SendData)(uint8_t * data, uint8_t maxLen, uint8_t *pending);

    //�ڵ��յ����ط��͵��û�����������з������ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendDataAck)(uint8_t * data, uint8_t maxLen);

    //�ڵ��յ����ط��͵Ŀ���������з������ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendControlAck)(uint8_t * data, uint8_t maxLen);
}InterBow_Node_Cb_t;


typedef struct 
{
    //���ؽ��յ��źż������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // rssi:    �����ź�ǿ��
    // srcAddr: �ڵ��ַ
    // ���أ�    ��  
    void (*RecvSingleTest)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //���ؽ��յ�ע������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // rssi:    �����ź�ǿ��
    // srcAddr: �ڵ��ַ
    // ���أ�    ��  
    void (*RecvRegistering)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //���ؽ��յ��û�����������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // rssi:    �����ź�ǿ��
    // srcAddr: �ڵ��ַ
    // busy:    �ڵ��Ƿ�Ҫ�����������ݣ�����busyΪ0�ǣ����زſɸ�ֵpendingΪ1,���б���
    //          ���ӣ����ؿɳ����·����ݸ��ɼ���
    // ����:      0x00: ���ݽ�����ȷ
    //            0x01: ���ݽ����쳣
    uint8_t (*RecvData)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr, uint8_t busy, uint8_t *pending);

    //����V10�汾Э�����ؽ��յ��û�����������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // rssi:    �����ź�ǿ��
    // ����:      0x00: ���ݽ�����ȷ
    //            0x01: ���ݽ����쳣
    uint8_t (*RecvData_V10)(uint8_t * data, uint8_t len, int16_t rssi);

    //���ؽ��յ��ڵ���û����ݽ�������������������û�������
    // data:    �������û�����ָ��
    // len:     �û����ݳ���
    // rssi:    �����ź�ǿ��
    // srcAddr: �ڵ��ַ
    // ���أ�    ��  
    void (*RecvDataAck)(uint8_t * data, uint8_t len, int16_t rssi, uint32_t srcAddr);

    //���ط����źż������ʱ���ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendSingleTestAck)(uint8_t * data, uint8_t maxLen);

    //���ط���ע������ʱ���ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendRegisteringAck)(uint8_t * data, uint8_t maxLen);

    //���ط������ݷ�������ʱ���ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendDataAck)(uint8_t * data, uint8_t maxLen);

    //���ط��������������û������ݣ������ڴ���ð�����ʱ���ɵ���InterBow_SetBusy()ʹ����һֱ����
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendData)(uint8_t * data, uint8_t maxLen, uint32_t dstDevId);

    //���ط��Ϳ�������ɸ�������û�������
    // data:    �������û�����ָ��
    // maxLen:  �ɴ������󳤶�
    // ���أ�    ����ĳ���
    uint8_t (*SendControlData)(uint8_t * data, uint8_t maxLen);
}InterBow_Gateway_Cb_t;




typedef struct 
{
// Ӳ����Ƶ��MODULE_LORA, MODULE_CC1310
    uint32_t radioType;
// ��ƵӲ�����ߺ���
    void (*Sleep)(void);
// ��ƵӲ��standby����
    void (*Standby)(void);
// ��ƵӲ����ʼ������
    void (*Reset)(void);
// ��ƵӲ���������ģʽ
    bool (*RxMode)(void);
// ��ƵӲ��Ӳ����ʼ��
    void (*Init)(void (*Cb)(void));
// ��ȡ��ƵӲ������
    void (*GetHwConfig)(IB_hw_config_t* config);
// ��ƵӲ��Ƶ������
    void (*FreqSet)(uint32_t freq);
// ��ƵӲ������ģʽ����
    void (*RateModeSet)(uint8_t rateMode);
// ��ƵӲ���������߷��书��
    void (*RfPowerSet)(uint8_t rfPower);
// ��ƵӲ����ȡ��������
    uint8_t (*ReadRxPacket)(uint8_t* buf, uint8_t len);
// ��ƵӲ��������������
    bool (*SendPacket)(uint8_t* buf, uint8_t len);
// ��ƵӲ����ȡ�����ź�ǿ��
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

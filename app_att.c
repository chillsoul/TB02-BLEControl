/********************************************************************************************************
 * @file     app_att.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 public@telink-semi.com;
 * @date     Sep. 18, 2015
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "application/print/u_printf.h"
#include "at_cmd.h"
#include "drivers\8258\pwm.h"
typedef struct
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  u16 latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  u16 timeout;
} gap_periConnectParams_t;

static const u16 clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG;

static const u16 extReportRefUUID = GATT_UUID_EXT_REPORT_REF;

static const u16 reportRefUUID = GATT_UUID_REPORT_REF;

static const u16 characterPresentFormatUUID = GATT_UUID_CHAR_PRESENT_FORMAT;

static const u16 userdesc_UUID	= GATT_UUID_CHAR_USER_DESC;

static const u16 serviceChangeUUID = GATT_UUID_SERVICE_CHANGE;

static const u16 my_primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;

static const u16 my_characterUUID = GATT_UUID_CHARACTER;

static const u16 my_devServiceUUID = SERVICE_UUID_DEVICE_INFORMATION;

static const u16 my_PnPUUID = CHARACTERISTIC_UUID_PNP_ID;

static const u16 my_devNameUUID = GATT_UUID_DEVICE_NAME;

static const u16 my_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;

static const u16 my_appearanceUIID = GATT_UUID_APPEARANCE;

static const u16 my_periConnParamUUID = GATT_UUID_PERI_CONN_PARAM;

static const u16 my_appearance = GAP_APPEARE_UNKNOWN;

static const gap_periConnectParams_t my_periConnParameters = {20, 40, 0, 1000};

static const u16 my_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE;

static u16 serviceChangeVal[2] = {0};

static u8 serviceChangeCCC[2] = {0,0};

#define MY_DEV_NAME                        "Ai-Thinker"
extern  u8 ble_devName[];

static const u8 my_PnPtrs [] = {0x02, 0x8a, 0x24, 0x66, 0x82, 0x01, 0x00};



//////////////////////// OTA  ////////////////////////////////////////////////////
static const  u8 my_OtaUUID[16]					    = TELINK_SPP_DATA_OTA;
static const  u8 my_OtaServiceUUID[16]				= TELINK_OTA_UUID_SERVICE;
static u8 my_OtaData 						        = 0x00;
static const  u8 my_OtaName[] 						= {'O', 'T', 'A'};

////////////////////// SPP ////////////////////////////////////
static const u8 TelinkSppServiceUUID[16]	      	    = TELINK_SPP_UUID_SERVICE;
static const u8 TelinkSppDataServer2ClientUUID[16]      = TELINK_SPP_DATA_SERVER2CLIENT;
static const u8 TelinkSppDataClient2ServerUUID[16]      = TELINK_SPP_DATA_CLIENT2SERVER;


// Spp data from Server to Client characteristic variables
static u8 SppDataServer2ClientDataCCC[2]  				= {0};

//this array will not used for sending data(directly calling HandleValueNotify API), so cut array length from 20 to 1, saving some SRAM

static u8 SppData_1[1] 					= {0};  //SppDataServer2ClientData[20]
// Spp data from Client to Server characteristic variables
//this array will not used for receiving data(data processed by Attribute Write CallBack function), so cut array length from 20 to 1, saving some SRAM
static u8 SppData_2[1] 					= {0};  //SppDataClient2ServerData[20]


//SPP data descriptor
static const u8 Telink_Descriptor_1[] 		 		= "Ai-Thinker SPP: Module<->Phone";

//// GAP attribute values
static const u8 my_devNameCharVal[5] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(GenericAccess_DeviceName_DP_H), U16_HI(GenericAccess_DeviceName_DP_H),
	U16_LO(GATT_UUID_DEVICE_NAME), U16_HI(GATT_UUID_DEVICE_NAME)
};
static const u8 my_appearanceCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(GenericAccess_Appearance_DP_H), U16_HI(GenericAccess_Appearance_DP_H),
	U16_LO(GATT_UUID_APPEARANCE), U16_HI(GATT_UUID_APPEARANCE)
};
static const u8 my_periConnParamCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(CONN_PARAM_DP_H), U16_HI(CONN_PARAM_DP_H),
	U16_LO(GATT_UUID_PERI_CONN_PARAM), U16_HI(GATT_UUID_PERI_CONN_PARAM)
};


//// GATT attribute values
static const u8 my_serviceChangeCharVal[5] = {
	CHAR_PROP_INDICATE,
	U16_LO(GenericAttribute_ServiceChanged_DP_H), U16_HI(GenericAttribute_ServiceChanged_DP_H),
	U16_LO(GATT_UUID_SERVICE_CHANGE), U16_HI(GATT_UUID_SERVICE_CHANGE)
};


//// device Information  attribute values
static const u8 my_PnCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_pnpID_DP_H), U16_HI(DeviceInformation_pnpID_DP_H),
	U16_LO(CHARACTERISTIC_UUID_PNP_ID), U16_HI(CHARACTERISTIC_UUID_PNP_ID)
};


//// Telink spp  attribute values
static const u8 TelinkSppData_1[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_NOTIFY,
	U16_LO(SPP_SERVER_TO_CLIENT_DP_H), 
	U16_HI(SPP_SERVER_TO_CLIENT_DP_H), 
	TELINK_SPP_DATA_SERVER2CLIENT
};

int my_strncmp(const char* firstString, const char* secondString,int n) {
	secondString += 6;
    while ((*firstString == *secondString)&&(n>0)) {
        ++firstString;
        ++secondString;
		n--;
    }
	if (n == 0)
		return 1;//严格比较
	else
		return 0;

}

extern u32 latest_user_event_tick;
char buff[64] = {0};
char key[64] = { 0 };
#define USER_NUM  6
char whitelist[USER_NUM][32] = {
	//OPENID
};
int module_onReceiveData(rf_packet_att_write_t *p)
{
	latest_user_event_tick = clock_time();//更新计时器
	//rf_packet_att_write_t是SDK定义的蓝牙写入数据的结构体，见SDK142页
	u8 len = p->l2capLen - 3;
	int right_user = 0;
	if((gpio_read(CONTROL_GPIO) == 0)) //AT模式
	{
		u_sprintf(buff, "\r\n+DATA:%d,", len);//长度和字符串赋值给buff
		at_print(buff);//输出字符串 \r\n+DATA:%d
		at_send((char*)&p->value, len);//输出传输的值
		at_print("\r\n");//输出结尾字符串
		/*读出at_send和print作用后得出思路如下：
			读取这段数值然后比对，符合则执行我们需要的控制函数*/
		u_sprintf(key, "%s", (char*)&p->value);//字符串赋值给key
		for (int Index = 0; Index < USER_NUM ; Index++)
		{
			if (my_strncmp(key, whitelist[Index], 20)) {//只比较20个字符（一个包）
				right_user = 1;
			}
		}
        if (right_user) {

            pwm_set_clk(CLOCK_SYS_CLOCK_HZ, 2000000);
            gpio_set_func(GPIO_PC3, AS_PWM1);
            pwm_set_mode(PWM1_ID, PWM_NORMAL_MODE);
            pwm_set_phase(PWM1_ID, 0);   //no phase at pwm beginning
			//舵机初始位置在字对面~
			pwm_set_cycle_and_duty(PWM1_ID, (u16)(20 * 2000000 / 1000), (u16)(0.5 * 2000000 / 1000));//舵机0度 在字上
            pwm_start(PWM1_ID);
            WaitMs(2000);
			pwm_set_cycle_and_duty(PWM1_ID, (u16)(20 * 2000000 / 1000), (u16)(2.5 * 2000000 / 1000));//舵机180度 在字对面
            WaitMs(2000);
            pwm_stop(PWM1_ID);
			bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
        }
		else {
			while (bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN) == 0x3A);
			at_print("Wrong User!\r\n");
		}



	}
	else
	{
		at_send((char*)&p->value, len);//传送数据到串口
	}

	return 0;
}


static const attribute_t my_Attributes[] = {

	{ATT_END_H -1, 0,0,0,0,0},	// total num of attribute

	// 0001 - 0007  gap
	{7,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gapServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_devNameCharVal),(u8*)(&my_characterUUID), (u8*)(my_devNameCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,MAX_DEV_NAME_LEN, (u8*)(&my_devNameUUID), (u8*)(ble_devName), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_appearanceCharVal),(u8*)(&my_characterUUID), (u8*)(my_appearanceCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_appearance), (u8*)(&my_appearanceUIID), 	(u8*)(&my_appearance), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_periConnParamCharVal),(u8*)(&my_characterUUID), (u8*)(my_periConnParamCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_periConnParameters),(u8*)(&my_periConnParamUUID), 	(u8*)(&my_periConnParameters), 0},


	// 0008 - 000b gatt
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gattServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_serviceChangeCharVal),(u8*)(&my_characterUUID), 		(u8*)(my_serviceChangeCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (serviceChangeVal), (u8*)(&serviceChangeUUID), 	(u8*)(&serviceChangeVal), 0},
	{0,ATT_PERMISSIONS_RDWR,2,sizeof (serviceChangeCCC),(u8*)(&clientCharacterCfgUUID), (u8*)(serviceChangeCCC), 0},


	// 000c - 000e  device Information Service
	{3,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_devServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_PnCharVal),(u8*)(&my_characterUUID), (u8*)(my_PnCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_PnPtrs),(u8*)(&my_PnPUUID), (u8*)(my_PnPtrs), 0},


	// 000f - 0016 SPP
	{5,ATT_PERMISSIONS_READ,2,16,(u8*)(&my_primaryServiceUUID), 	(u8*)(&TelinkSppServiceUUID), 0},

	{0,ATT_PERMISSIONS_READ,2,sizeof(TelinkSppData_1),(u8*)(&my_characterUUID), 		(u8*)(TelinkSppData_1), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,16,sizeof(SppData_1),(u8*)(&TelinkSppDataServer2ClientUUID), (u8*)(SppData_1), (att_readwrite_callback_t)&module_onReceiveData},	//value 写服务
	{0,ATT_PERMISSIONS_RDWR,2,2,(u8*)&clientCharacterCfgUUID,(u8*)(&SppDataServer2ClientDataCCC)},
	{0,ATT_PERMISSIONS_READ,2,sizeof(Telink_Descriptor_1),(u8*)&userdesc_UUID,(u8*)(&Telink_Descriptor_1)},
};

void my_att_init (void)
{
	bls_att_setAttributeTable ((u8 *)my_Attributes);

	u8 device_name[] = MY_DEV_NAME;
	bls_att_setDeviceName(device_name, sizeof(MY_DEV_NAME));
}
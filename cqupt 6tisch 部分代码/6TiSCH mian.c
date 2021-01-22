
/******************************************************************************************************
*
* 文 件 名：test_temperature.c
*
* 文件描述：协议栈温度传感器示例
*
* 创 建 者：Wang Heng
*
* 当前版本:  0.50

* 修 改 者：
*
* 修改历史：


********************************************************************************************************/

#include "6lowsn.h"
#include "slip.h"
#include "halstack.h"




// 若定义LOWSN_SIMULATED_SENSOR_DATA，则采用从0-99循环递增的温度模拟数据，方便测试
// 若取消该宏的定义，则利用CC2530真实的温度传感器功能读取环境温度值作为数据.

#define LOWSN_SIMULATED_SENSOR_DATA

extern UINT8 g_initRadioFlag;
UINT32 repair_timer;
#define REPAIRTIMER  1
extern UINT16 g_rx_pkt_cnt;

void relaJudge();
extern UINT8 recRelaJudge[10];





#ifdef LOWSN_RFD
#define NODE_SLEEP_TIME  1   //节点睡眠时间
#endif

UINT32 my_timer;
IPADDR dstIPADDR;
IPADDR network_prefix;
UINT8 prefix_len;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IPADDR network_contextprefix;//用于压缩的文本前缀
UINT8 contextprefix_len;//用于压缩的文本前缀长度
uint8_t Lifetime_delay;//发送等待的生存期延迟

//extern uint32_t retrans_time;//邻居可到达时间
//static UINT32 nwk_retrans_timer;//邻居可到达时间定时

extern lowsn_ds6_prefix_t lowsn_ds6_prefix_list[LOWSN_DS6_PREFIX_NB];//brief prefix list
static lowsn_ds6_prefix_t *prefix;
extern lowsn_ds6_defrt_t lowsn_ds6_defrt_list[LOWSN_DS6_DEFRT_NB]; //brief defrt list
static lowsn_ds6_defrt_t *defrt;
uint8_t prefixFORregistration[8];
UINT8 AddressRegis=0;

extern NL_DIO nRpl_ParentSet[RPL_PARENTSET_CAPACITY];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

UINT8 ReadyToResponse = 0;
UINT8 g_testRPLJoinedDODAG=1;

//  EUI64地址使用示例(本例子中未使用长地址做目标地址)
// 按阅读顺序排列，注意第一个字节的LSB倒数第2位必须为0，即U/L原始位为0.
// 倒数第1位也为0，表示是单播。建议第一个字节为0x00.
// 可以用buf设定目标节点的EUI64地址，若要设定节点自己的EUI64，首选在IAR的工程文件编译选项
// 中的预配置中进行定义，若没有在编译参数中定义，则还可在6lowsn_config.h中
// 用aExtendedAddress_B0-aExtendedAddress_B7宏设定。若编译参数中已经定义，在在文件中定义，
// 就会出现重定义的警告。

UINT8 coordEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0xff,0x00};
UINT8 rfdEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0x53,0x00};


//0x01  0xc4,0x0b
//0x02  0xc4,0x38
//0x03  0xc5,0xe9
//0x04  0xc4,0x5e
//0x05  0xc5,0x8f
//0x06  0xc5,0xbc
//0x07  0xc4,0x6d
//0x08  0xc4,0x92
//0x09  0xC5,0x43
//0x0a  0xC5,0x70
#define isUpData_Sensor()  (seriobuf[0] == 0x0a && seriobuf[1] == 0x03 )// RTU address zhangyan
UINT8 realDataUsart[8]={0x0a,0x03,0x00,0x00,0x00,0x02,0xC5,0x70};//RTU address zhangyan

UINT8 LastSentData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //最后一近上传的数据，通过记录和判断最近一次上传数据的校验码，避免重复性上传相同数据


UINT16 len;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t Lifetime_delay;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//*********apl**********
INT8 temp_upper_bound, temp_lower_bound;


#ifdef LOWSN_RFD

#define REMOTE_PORT    0xF0B2
#define LOCAL_PORT      0xF0B3
#define DEFAULT_SLEEP_PERIOD  2000   //以ms为单位	
#define DEFAULT_RX_PERIOD  5000   //以ms为单位

#define REMOTE_OBJECT_ID    1

#define OBJECT_ID_TEMP    1          // 温度对象ID号
#define ATTRIBUTE_ID_CURRENT_TEMP    1
#define ATTRIBUTE_ID_TEMP_UPPER        2
#define ATTRIBUTE_ID_TEMP_LOWER       3	

#define OBJECT_ID_LIGHT   2   // LED灯对象ID

#define OBJECT_ID_POWER_MGT   12   // 能量管理，暂时放在应用程序中，将来应放入管理程序中
#define ATTRIBUTE_ID_SLEEP_PERIOD    1   // 休眠时间长度，以ms为单位
#define ATTRIBUTE_ID_RX_PERIOD          2   // 醒来后等待接收的时间长度，以ms为单位
#define ATTRIBUTE_ID_GOTO_SLEEP        3   // 手动要求节点进入休眠状态，第一个参数为多少ms后进入休眠

typedef enum _NODE_STATE_ENUM {
        NODE_STATE_IDLE,
        NODE_STATE_REAL_DATA,
	NODE_STATE_SENSE_DATA, 
	NODE_STATE_SEND_DATA, 
	NODE_STATE_SLEEP,
	NODE_STATE_SEND,
	NODE_STATE_WAIT_FOR_TX, 
	NODE_STATE_WAIT_FOR_RX, 
	NODE_STATE_WAIT_FOR_RESPONSE_TX
}NODE_STATE_ENUM;

typedef struct _RESPONSE_PARAM {
	IPADDR dstIPADDR;
	UINT16 dstPort;
	BYTE dstObjID;
	UINT16 srcPort;
	BYTE srcObjID; 
	BYTE serviceID; 
	BYTE actDirection; 
	BYTE actType;
	BYTE *pload;
	BYTE plen;
}RESPONSE_PARAM;

NODE_STATE_ENUM NodeState;
RESPONSE_PARAM ResponseParam;
INT8 temp_upper_bound, temp_lower_bound;
INT8 current_temp;
INT8 SensorPayload[255];
INT8 ResponsePayload[20];
UINT32 SleepPeriod, RxPeriod;
BOOL sendResponseFlag;
INT8 temp_value=0;

#ifdef  LOWSN_SIMULATED_SENSOR_DATA

INT8 getTempValue()
{
	temp_value++;
	
	if (temp_value > 100)  {
		temp_value=0;
	}

	return (temp_value);

}

#else

INT8 getTempValue()
{
  static UINT16 voltageAtTemp22;
  static UINT8 bCalibrate=TRUE;
  UINT16 value;
  INT8 temp;

  ATEST = 0x01;
  TR0  |= 0x01; 
  
  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_512 | HAL_ADC_CHN_TEMP);

  while ( !ADCIF );//?

  value = ADCL;
  value |= ((UINT16) ADCH) << 8;

  value >>= 4;
  
    /* Assume ADC = 1480 at 25C and ADC = 4/C */
  #define VOLTAGE_AT_TEMP_25        1480
  #define TEMP_COEFFICIENT          4

  if(bCalibrate) 
  {
    voltageAtTemp22=value;
    bCalibrate=FALSE;
  }
  
  temp = 22 + ( (value - voltageAtTemp22) / TEMP_COEFFICIENT );
  
  // 温度只支持0-100度之间，负温度不支持
  if( temp >= 100) 
  {
    return 100;
  }
  else if (temp <= 0) {
    return 0;
  }
  else { 
    return temp;
  }

}

#endif


void NodeAction (void ) {

	apsFSM();

	switch (NodeState) {

          
                        case NODE_STATE_IDLE:
          
                            break;
                            
                        case NODE_STATE_REAL_DATA:
                          //serioFlushBuf(500);
                          for(UINT8 i = 0;i<8;i++){
                            //conPrintUINT8(realDataUsart[i]);
                            //conPrintROMString(" ");
                            
                            //zhangyan
                            halPutch(realDataUsart[i]);
                            
                          }
                          conPrintROMString("");//conPrintROMString(" ");
                          
                          
                          NodeState = NODE_STATE_WAIT_FOR_TX;//NODE_STATE_IDLE
                          break;
                            
			case NODE_STATE_SENSE_DATA:
				

					current_temp = getTempValue();

					//conPrintROMString("Current temperature is : ");
					//conPrintUINT8((UINT8)current_temp);
                                   //conPrintROMString("\n");
				
					NodeState = NODE_STATE_SEND_DATA;

					// 对温度值进行滤波等处理

				break;
				
			case NODE_STATE_SEND_DATA:


//					conPrintROMString("Start to send, num: ");
//					conPrintUINT8(current_temp);
//					conPrintROMString("  ");

					// 准备温度发送的负载  //huangyi
					SensorPayload[0] = 1;  // Attribute Number
					SensorPayload[1] = ATTRIBUTE_ID_CURRENT_TEMP; //  Attribute identifier
					SensorPayload[2] = 0; //  Attribute secondary identifier
					SensorPayload[3] = 1; // Length 
					SensorPayload[4] = 0xff; // Value                                         
                                        

//                                        for(UINT8 i=0;i<len;i++)
//                                        {
//                                          SensorPayload[i+5] = seriobuf[i];
//                                        }
                                      //huangyi 上传数据
                                        memcpy(&SensorPayload[5],&seriobuf,len);
                                        
                                        //huangyi  清除数据
                                        serioFlushBuf(500);
                  
                                        

			                //NodeState = NODE_STATE_IDLE;
                                        
					// 判定温度值是否超标
					if ((current_temp >= temp_upper_bound) || (current_temp <= temp_lower_bound))  {
					//if (0)  {

						// 做本地的报警动作，闪烁LED


						// 以report/alarm方式发送数据
                                          aplSendReportData(dstIPADDR, REMOTE_PORT, REMOTE_OBJECT_ID, LOCAL_PORT, OBJECT_ID_TEMP, 
  						                                  apsGenServiceID(), 0, (BYTE *)(&SensorPayload[0]), len+5); 
                                                
						
					}	
					else  {
						
						// 以publish方式发送数据
  						aplSendPublishData(dstIPADDR, REMOTE_PORT, REMOTE_OBJECT_ID, LOCAL_PORT, OBJECT_ID_TEMP, 
                                                                   apsGenServiceID(), 0, (BYTE *)(&SensorPayload[0]), len+5); 
                                               
                                          
									
					}
                                        
//                                        for(UINT8 i=0;i<100;i++){
//                                         SensorPayload[i]=0;
//                                        }
                                        
                                        //zhangyan
                                        if(len == 9 || len == 38){ //
                                            NodeState = NODE_STATE_WAIT_FOR_TX;
                                        }
                                        else {
                                            NodeState = NODE_STATE_IDLE;
                                        }    

				break;

			case NODE_STATE_WAIT_FOR_TX:
				if (apsBusy()) break; 
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					//conPrintROMString("Sent OK! \n");
					my_timer= halGetMACTimer();
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}else {
					//conPrintROMString("Sensor Data Send failed! Retry! \n");
					my_timer= halGetMACTimer();
					NodeState =NODE_STATE_WAIT_FOR_RX;  //不再重传, 协议栈内部会重传
					//NodeState = NODE_STATE_SEND_DATA;
				}
				break;

			case  NODE_STATE_WAIT_FOR_RX:
				
////////////////////////////////////////////////////////////////////////////////////				
					if (halMACTimerNowDelta(my_timer) > MSECS_TO_MACTICKS(2000)) {//power_pib.RxPeriod zhangyan
////////////////////////////////////////////////////////////////////////////////////

					NodeState =NODE_STATE_SLEEP;
                                        
                                        
					
					break;
					
				}

				if (sendResponseFlag)  {

					sendResponseFlag = 0;
				
					
					aplSendCSData(ResponseParam.dstIPADDR, ResponseParam.dstPort, ResponseParam.dstObjID, ResponseParam.srcPort, ResponseParam.srcObjID, 
					                          ResponseParam.serviceID, 0, ResponseParam.pload, ResponseParam.plen, ResponseParam.actDirection, ResponseParam.actType); 		

					NodeState = NODE_STATE_WAIT_FOR_RESPONSE_TX;
	
				}	
				
				break;	

			case NODE_STATE_WAIT_FOR_RESPONSE_TX:
				if (apsBusy()) break;
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					
					conPrintROMString("Response Sent Successfully! \n");
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}else {
				
					conPrintROMString("Response Send failed! Retry! \n");
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}
				break;
				
			case  NODE_STATE_SLEEP:
			   // conPrintROMString("Node begin to sleep! \n");
			    // 须保障sleep时不掉网, 不丢RAM数据，醒来无需重新入网
#ifdef SleepTest
/////////////////////////////////////////////////////////////////////////////////////
			   		halSleep(power_pib.SleepPeriod);
/////////////////////////////////////////////////////////////////////////////////////
				    //conPrintROMString("Node wake up!\n");
#endif
					//NodeState = NODE_STATE_SENSE_DATA;
                                        NodeState =NODE_STATE_REAL_DATA;
				break;		

			default:
				conPrintROMString("Never run to this state, something must be wrong! \n");
				break;

	}
}

void UsartPoll(void)
{
  if(halGetchRdy())
  {
//    len = serioRcv();
//    if(len == 9 ){
//      if(isUpData_Sensor()){
//        if(seriobuf[len-2]==LastSentData[0] && seriobuf[len-1]==LastSentData[1])//校验码相同就不上传
//        {
//          serioFlushBuf(500);
//          NodeState = NODE_STATE_WAIT_FOR_TX;//NODE_STATE_IDLE
//        }
//        else
//        {
//            LastSentData[0] = seriobuf[len-2];
//            LastSentData[1] = seriobuf[len-1];
//            NodeState = NODE_STATE_SEND_DATA;
//        }
//      }
//    }
//   if(len == 38){//执行到这一步就说明出错啦
//       NodeState = NODE_STATE_SEND_DATA;
//    }
    len = serioRcv();
    if(len == 9 ){
      if(isUpData_Sensor()){
        NodeState = NODE_STATE_SEND_DATA;
      }
    }
   if(len == 38){
       NodeState = NODE_STATE_SEND_DATA;
    }
  }  
}

// 返回负载的实际长度
UINT8 FmtCurrentTempResponsePayload(void)
{
	INT8 temp;
	
	temp = getTempValue();

	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_CURRENT_TEMP;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp;

	return 6;
}

UINT8 FmtTempUpperResponsePayload(void)
{

	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_TEMP_UPPER;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp_upper_bound;

	return 6;
}

UINT8 FmtTempLowerResponsePayload(void)
{
	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_TEMP_LOWER;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp_lower_bound;

	return 6;
}


UINT8 FmtSleepPeriodResponsePayload(void)
{
	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_SLEEP_PERIOD;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 4;  //大段模式传输
	ResponsePayload[5] = (UINT8)((SleepPeriod & 0xF000) >> 24);
	ResponsePayload[6] = (UINT8)((SleepPeriod & 0x0F00) >> 16);
	ResponsePayload[7] = (UINT8)((SleepPeriod & 0x00F0) >> 8);
	ResponsePayload[8] = (UINT8)(SleepPeriod & 0x000F);

	return 9;
}

UINT8 FmtRxPeriodResponsePayload(void)
{
	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_RX_PERIOD;
	ResponsePayload[4] = 4;  //大段模式传输
	ResponsePayload[5] = (UINT8)((RxPeriod & 0xF000) >> 24);
	ResponsePayload[6] = (UINT8)((RxPeriod & 0x0F00) >> 16);
	ResponsePayload[7] = (UINT8)((RxPeriod & 0x00F0) >> 8);
	ResponsePayload[8] = (UINT8)(RxPeriod & 0x000F);

	return 9;
}


#endif

void main (void){

    BYTE testpload[5] = {0,1,2,3,4};
	my_timer = 0;
	Lifetime_delay=0;
	BYTE i;

        
	halInit();
	aplInit(); 

	#ifdef LOWSN_COORDINATOR
	#if (defined LOWSN_SLIP_TO_HOST)|(defined TSCH_TEST)
	slipInit();
        #endif
	#endif
		
	conPrintConfig();
	ENABLE_GLOBAL_INTERRUPT();  //enable interrupts

	debug_level = DBG_MAX_LEVEL;
        
        
/******************/
	#ifdef LOWSN_COORDINATOR
	#ifdef LOWSN_SLIP_TO_HOST
	// First, get a prefix from host.
	do {
			my_timer= halGetMACTimer();
			while(slipTxLocked());
			conPrintROMString("Request a golbal prefix ...  \n");
			slipGrabTxLock(); 
			slipRequestPrefix();
			slipReleaseTxLock(); 

			while ((!slipReady()) && (halMACTimerNowDelta(my_timer) < MSECS_TO_MACTICKS(2*1000)));

			if (slipReady())  {
		
				slipRcv();

				if (slipParsePrefix(&network_prefix, &prefix_len)  == 0)  {
					conPrintROMString("Get global prefix successfully! \n");
					aplSetPrefix(&network_prefix, prefix_len);
					conPrintROMString("Set global prefix:  ");
					conPrintIP6ADDR(&network_prefix, 1);
					conPrintROMString("\n Prefix Len: ");
					conPrintUINT8(prefix_len);
					conPrintROMString("\n");
                    ds6GlobalIPFromEUI64();
					break;

				}	

				else  {
					conPrintROMString("Not a prefix slip command. Try again! \n");
				}
			}	
		
	 } while (1);

	#else
	lowsn_ip6addr(&network_prefix,0x0077,0x0077,0x1234,0x5678,0,0,0,0);
	prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
	aplSetPrefix(&network_prefix, prefix_len);
	conPrintROMString("Set global prefix:  ");
	conPrintIP6ADDR(&network_prefix, 1);
	conPrintROMString("  Prefix Len: ");
	conPrintUINT8(prefix_len);
	conPrintROMString("\n");
        ds6GlobalIPFromEUI64();
	#endif //SLIP TO PREFIX
	
        
	#ifdef  LOWSN_NO_JOIN_PROCESS
	aplFormNetworkDirectly(); 
	#else
	aplFormNetwork();
	#endif
	while(apsBusy()) {apsFSM();} //wait for finish
	conPrintROMString("Network is formed. \n");

        g_testRPLJoinedDODAG = 1;
        
/******************/ //RFD,ROUTER       
	#else 
                  
        g_testRPLJoinedDODAG = 0;

       //=====入网 
	#ifdef LOWSN_NO_JOIN_PROCESS
	aplJoinNetworkDirectly(0x1699, 0, &coordEUI[0], 1);
	while(apsBusy()) {apsFSM();} 
	#else
	do {
		aplJoinNetwork();
		while(apsBusy()) {apsFSM();} //wait for finish
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			//EVB_LED1_ON();
			conPrintROMString("Network Join succeeded!\n");
			conPrintROMString("My ShortAddress is: ");
			conPrintUINT16(aplGetMyShortAddress());
			conPCRLF();
			conPrintROMString("Parent LADDR: ")
			conPrintLADDR(aplGetParentLongAddress());
			conPrintROMString(", Parent SADDR: ");
			conPrintUINT16(aplGetParentShortAddress());
			conPCRLF();
			break;
		}else {
			conPrintROMString("Network Join FAILED! Waiting, then trying again\n");
			my_timer= halGetMACTimer();
			//wait for 2 seconds
			while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
		}
	} while(1);
        
        ///while(1){apsFSM();}////huangyi
	#endif
        
        
        //=====前缀
	#ifdef LOWSN_NO_PREFIX_PROCESS
	// 手动设置节点的全局前缀
              #ifdef LOWSN_FFD
              lowsn_ip6addr(&network_prefix,0x0099,0x0077,0x1234,0x5678,0,0,0,0);
              prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
              aplSetPrefix(&network_prefix, prefix_len);
              conPrintROMString("Set global prefix:  ");
              conPrintIP6ADDR(&network_prefix, 1);
              conPrintROMString("Prefix Len: ");
              conPrintUINT8(prefix_len);
              conPrintROMString("\n");
              ds6GlobalIPFromEUI64();
              #endif
	#else
//////////////////////////////////////////////////////////////////////////////////////////////////	
              #ifdef LOWSN_ROUTER
              lowsn_ip6addr(&network_prefix,0xaaa1,0,0,0,0,0,0,0);
              prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
              aplSetPrefix(&network_prefix, prefix_len);
              conPrintROMString("Set global prefix:  ");
              conPrintIP6ADDR(&network_prefix, 1);
              conPrintROMString("Prefix Len: ");
              conPrintUINT8(prefix_len);
              conPrintROMString("\n");
              ds6GlobalIPFromEUI64();
              #endif
	
        do {
		conPrintROMString("Begin to get a gloabal prefix. \n");
		aplGetPrefix();
		while(apsBusy()) {apsFSM();} //wait for finish
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			conPrintROMString("Get a global prefix!\n");
			break;
		}else {
			conPrintROMString("Prefix obtaining FAILED! Waiting, then trying again\n");
            my_timer= halGetMACTimer();
           //wait for 2 seconds
            while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
		}
	} while(1);
//////////////////////////////////////////////////////////////////////////////////////////////////	
	#endif

#endif //COORDINATOR,RFD,ROUTER

        
/******************/   
	aplSetMacMaxFrameRetries(0);
	repair_timer = halGetMACTimer();

	
/******************/
	#ifdef LOWSN_COORDINATOR
    	my_timer= halGetMACTimer();
	while (1) {
		
          apsFSM();
          
          #if (defined LOWSN_SLIP_TO_HOST)|(defined TSCH_TEST)
          slipFSM();
          #endif
          
          #if  0
          if ((halMACTimerNowDelta(my_timer))> MSECS_TO_MACTICKS(10*1000) && g_testRPLJoinedDODAG == 1)
          {
            nSendDIO();
            my_timer= halGetMACTimer();
            
          }
          #endif
	}	
	#endif
        
/******************/
	#ifdef LOWSN_ROUTER
    	my_timer= halGetMACTimer();
	while (1) {
		
		while(g_testRPLJoinedDODAG == 0)
		{
			nRPLJoinAction();
			while(apsBusy()) {apsFSM();} //wait for finish
			my_timer= halGetMACTimer();
             //wait for X seconds
                while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(1*1000))
               {
				apsFSM();
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0
		if((AddressRegis==0)&&(g_testRPLJoinedDODAG == 1))
		{
			for(prefix = lowsn_ds6_prefix_list; prefix < lowsn_ds6_prefix_list + LOWSN_DS6_PREFIX_NB; prefix++) {
				if((prefix->isused) &&!(lowsn_is_addr_link_local(&prefix->ipaddr))) {
					
					for(defrt= lowsn_ds6_defrt_list; defrt< lowsn_ds6_defrt_list + LOWSN_DS6_DEFRT_NB;defrt++) {
						if((defrt->isused)&&(lowsn_ipaddr_prefixcmp(&prefix->ipaddr,&defrt->ipaddr, 64))){
							if(lowsn_ds6_cmp(&defrt->ipaddr,&nRpl_ParentSet[g_preferdParent].ParentAddress)) {
								 for(i=0;i<8;i++)
								{
									prefixFORregistration[i]= prefix->ipaddr.u8[i];
								}
								do {
									conPrintROMString("\nBegin to Address Registration. \n");
									aplAddrRegister();
									while(apsBusy()) {apsFSM();} //wait for finish
									if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
										conPrintROMString("Address Registration Success!\n");
										conPrintROMString("\n");
										break;
									}else {
										conPrintROMString("Address Registration FAILED! Waiting, then trying again\n");
										my_timer= halGetMACTimer();
									   //wait for 2 seconds
									  while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
									}
								} while(1);
							}
						}
					}
				}
			}
			AddressRegis=1;
		}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		apsFSM();
		
/*                if ((halMACTimerNowDelta(my_timer))> MSECS_TO_MACTICKS(10*1000) && g_testRPLJoinedDODAG == 1)
                {
                    nSendDIO();
                    my_timer= halGetMACTimer();
                    
                }*/
/*                
                if ( (halMACTimerNowDelta(my_timer))> MSECS_TO_MACTICKS(1*1000) && g_testRPLJoinedDODAG == 0 && apsIdle() )
                {
                    nRPLJoinAction();
                    my_timer= halGetMACTimer();                    
                }*/
	}	
	#endif
        
        
        
/******************/
	#ifdef LOWSN_RFD
	
     //aplSetLinkLocalAddr16(&dstIPADDR, aplGetPanID(), 0);
	lowsn_ip6addr(&dstIPADDR,0x0077,0x0077,0x1234,0x5678,0x0210,0x2030,0x4050,0x0000);   
	conPrintROMString("RFD Object IP is: \n");
	conPrintIP6ADDR(&dstIPADDR, 1);
	conPrintROMString("\n");

	SleepPeriod = DEFAULT_SLEEP_PERIOD;
	RxPeriod = DEFAULT_RX_PERIOD;
	
	sendResponseFlag = 0;
	temp_upper_bound = 70;
	temp_lower_bound = 5;

	NodeState = NODE_STATE_WAIT_FOR_RX;
	
	
	conPrintROMString("Node FSM Start!\n");
	my_timer= halGetMACTimer();
	while (1) {
    
		apsFSM();
		
		while(g_testRPLJoinedDODAG == 0)
		{
			nRPLJoinAction();
			while(apsBusy()) {apsFSM();} //wait for finish
			my_timer= halGetMACTimer();
                        //wait for 1.5 seconds
                        while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(3000))
			{
				apsFSM();
			}
		}
		
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 1
		if((AddressRegis==0)&&(g_testRPLJoinedDODAG == 1))
		{
			for(prefix = lowsn_ds6_prefix_list; prefix < lowsn_ds6_prefix_list + LOWSN_DS6_PREFIX_NB; prefix++) {
				if((prefix->isused) &&!(lowsn_is_addr_link_local(&prefix->ipaddr))) {
					
					for(defrt= lowsn_ds6_defrt_list; defrt< lowsn_ds6_defrt_list + LOWSN_DS6_DEFRT_NB;defrt++) {
						if((defrt->isused)&&(lowsn_ipaddr_prefixcmp(&prefix->ipaddr,&defrt->ipaddr, 64))){
							if(lowsn_ds6_cmp(&defrt->ipaddr,&nRpl_ParentSet[g_preferdParent].ParentAddress)) {
								 for(i=0;i<8;i++)
								{
									prefixFORregistration[i]= prefix->ipaddr.u8[i];
								}
								do {
									conPrintROMString("\nBegin to Address Registration. \n");
									//aplAddrRegister();
									while(apsBusy()) {apsFSM();} //wait for finish
									if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
										conPrintROMString("Address Registration Success!\n");
										conPrintROMString("\n");
										break;
									}else {
										conPrintROMString("Address Registration FAILED! Waiting, then trying again\n");
										my_timer= halGetMACTimer();
									   //wait for 2 seconds
									  while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
									}
								} while(1);
							}
						}
					}
				}
			}
			AddressRegis=1;
		}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            NodeAction();///huangyi
            UsartPoll();
          /*    if ((halMACTimerNowDelta(my_timer))> MSECS_TO_MACTICKS(1*1000) && g_testRPLJoinedDODAG == 0)// && apsIdle() )
                {
                   nRPLJoinAction();
                    my_timer= halGetMACTimer();
                    
                }*/
                /*if(g_testRPLJoinedDODAG == 1)
                {
                   NodeAction();
		   
                    //apsFSM();
                }
                else
                    apsFSM();*/
	}

	#endif
	
}


LOWSN_STATUS_ENUM  usrRxPacketCallback(void) 
{

	BYTE *ptr;
	UINT8 plen;
        UINT8 TestDownPayload[5] = {0,1,2,3,4};
        UINT8 i;
	#if 1  //打印一些接收包的信息，仅供调试
	/*conPrintROMString("Data Received. \n");          ///huangyi
	conPrintROMString("Source IP: \n");
	conPrintIP6ADDR(&(aplGetRxSrcIPAddr()), 1);
	conPrintROMString("\n");
	conPrintROMString("Source Port: ");
	conPrintUINT16(aplGetRxSrcPort());
	conPrintROMString(", Service Type: ");
	conPrintUINT8(aplGetRxServiceType());
	conPCRLF();*/
//for test down route
//        aplSendReportData(aplGetRxSrcIPAddr(), 0xF0B4, 1, 0xF0B5, 1, 
//  						                                  apsGenServiceID(), 0, (BYTE *)(&TestDownPayload[0]), 5); 
        ReadyToResponse = 1;
	ptr = aplGetRxMsgData();

	#ifdef LOWSN_COORDINATOR
	#ifndef  LOWSN_SLIP_TO_HOST
	////conPrintROMString("Msg Length: ");////huangyi
	
  plen = aplGetRxMsgLen();
  
  /*conPrintUINT8(plen);     ///huangyi 
	conPCRLF();
	conPrintROMString("Msg:  ");*/
         //i=5;
	while(plen){
		
     //（ptr+i)++;
      halRawPut(*(ptr+i));//串口数据测试20151118   //huangyi
    //conPrintUINT8(*ptr);
		//conPrintROMString("  ");
       plen--;i++;  //huangyi  从第五个开始才是上传的数据
	}
	//conPCRLF();
	#endif
	#endif

	#endif
	
	
	#ifdef LOWSN_RFD

	// 检查端口号
	if ((aplGetRxSrcPort() != REMOTE_PORT) || (aplGetRxDstPort() != LOCAL_PORT)) {
		conPrintROMString("invalid port, discard it. ");
		return LOWSN_STATUS_SUCCESS;
	}	

	// 设节点只支持C/S读写 , 设一次只读/写一个属性
	if (aplGetRxServiceType() != APS_SERVICE_TYPE_CS) {
		conPrintROMString("Not a client/server type, discard it. ");
		return LOWSN_STATUS_SUCCESS;
	}	

	//  准备发送响应帧的部分参数
	ResponseParam.dstIPADDR = aplGetRxSrcIPAddr();
	ResponseParam.dstPort = aplGetRxSrcPort();
	ResponseParam.dstObjID = aplGetRxSrcObjectID();
	ResponseParam.srcObjID = aplGetRxDstObjectID();
	ResponseParam.srcPort = LOCAL_PORT;
	ResponseParam.serviceID = aplGetRxServiceID();
	ResponseParam.actDirection = APS_ACTION_RESPONSE;
	ResponseParam.actType = aplGetRxActType();

	//  处理C/S请求



	switch (ResponseParam.actType)  {
		
		case APS_ACTION_TYPE_READ:

			if (*ptr == 1)    {
				
				if (aplGetRxDstObjectID() ==  OBJECT_ID_TEMP)  {
					if (*(ptr+1) == ATTRIBUTE_ID_CURRENT_TEMP) {
					
						plen = FmtCurrentTempResponsePayload();
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_UPPER) {
						plen = FmtTempUpperResponsePayload();
						
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_LOWER) {
						plen = FmtTempLowerResponsePayload();
					}	

					else
					{
						conPrintROMString("invalid attribute ID for temperature, discard it. ");
						return LOWSN_STATUS_SUCCESS;

					}


				}
	
				else if  (aplGetRxDstObjectID() ==  OBJECT_ID_POWER_MGT)   {

					if (*(ptr+1) == ATTRIBUTE_ID_SLEEP_PERIOD) {
					
						plen = FmtSleepPeriodResponsePayload();
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_RX_PERIOD) {
					
						plen = FmtRxPeriodResponsePayload();
					}	

					else
					{
						conPrintROMString("invalid attribute ID for power management, discard it. ");
						return LOWSN_STATUS_SUCCESS;

					}


				}	

				else  {
						conPrintROMString("invalid object ID for READ, discard it. ");
						return LOWSN_STATUS_SUCCESS;

				}
					

				ResponseParam.pload = (BYTE *)(&ResponsePayload[0]);
				ResponseParam.plen = plen;
				sendResponseFlag = 1;
					
			}	


		break;
		
		case APS_ACTION_TYPE_WRITE:

			if (*ptr == 1)  {

				if (aplGetRxDstObjectID() ==  OBJECT_ID_TEMP)  {

					
					if (*(ptr+1) == ATTRIBUTE_ID_TEMP_UPPER) {
					
						temp_upper_bound = *(ptr+4);
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_LOWER) {
					
						temp_lower_bound = *(ptr+4);
					}	

					else
					{
						conPrintROMString("invalid attribute ID for temperature bound write, discard it. \n");
						return LOWSN_STATUS_SUCCESS;
					}

				}	

				else if  (aplGetRxDstObjectID() ==  OBJECT_ID_LIGHT)  {
					

						 conPrintROMString("Not support light control now, discard it. \n ");
						return LOWSN_STATUS_SUCCESS;

				}	

				else if (aplGetRxDstObjectID() ==  OBJECT_ID_POWER_MGT)  {

					
					if (*(ptr+1) == ATTRIBUTE_ID_SLEEP_PERIOD) {
					

						SleepPeriod = (UINT32)(*(ptr+4)) << 24;
						SleepPeriod = SleepPeriod + (UINT32)(*(ptr+5)) << 16;
						SleepPeriod = SleepPeriod + (UINT32)(*(ptr+6)) << 8;
						SleepPeriod = SleepPeriod + (UINT32)(*(ptr+7));
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_RX_PERIOD) {
					
						RxPeriod = (UINT32)(*(ptr+4)) << 24;
						RxPeriod = RxPeriod + (UINT32)(*(ptr+5)) << 16;
						RxPeriod = RxPeriod + (UINT32)(*(ptr+6)) << 8;
						RxPeriod = RxPeriod + (UINT32)(*(ptr+7));
					}	

					else
					{
						conPrintROMString("invalid attribute ID for power management write, discard it. \n");
						return LOWSN_STATUS_SUCCESS;
					}

				}	
				
				ResponsePayload[0] = 1;
				ResponsePayload[1] = APS_RESULT_OK;
				
				ResponseParam.pload = (BYTE *)(&ResponsePayload[0]);
				ResponseParam.plen = 2;
				sendResponseFlag = 1;
					
			}	

		break;		

		default: 

			conPrintROMString("invalid action type, discard it. ");
			break;
		
	}

	#endif 
	
	return LOWSN_STATUS_SUCCESS;
	
}

#ifdef LOWSN_FFD
//Callback to user level to see if OK for this node
//to join - implement Access Control Lists here based
//upon IEEE address if desired
BOOL usrJoinVerifyCallback(LADDR *ptr, BYTE capinfo)
{

#if 0      //set this to '1' if you want to test through a router
//only accept routers.
//only let routers join us if we are coord
#ifdef LOWSN_COORDINATOR
if (LOWSN_GET_CAPINFO_DEVTYPE(capinfo)) {
	//this is a router, let it join
	conPrintROMString("Accepting router\n");
	return TRUE;
}else {
	conPrintROMString("Rejecting non-router\n");
	return FALSE;
}
#else
return TRUE;
#endif

#else

return TRUE;

#endif

}

BOOL usrJoinNotifyCallback(LADDR *ptr){

	//allow anybody to join
	#ifndef LOWSN_SLIP_TO_HOST
	conPrintROMString("Node joined: ");
	conPrintLADDR(ptr);
	conPCRLF();
	DEBUG_PRINTNEIGHBORS(DBG_INFO);
	#endif
	
	return TRUE;
}
#endif


#ifndef LOWSN_COORDINATOR
BOOL usrGetPrefixCallback(IPADDR *prefix_ptr, UINT8 prefix_len)
{

	conPrintROMString("Get Prefix Successfully, Prefix: ");
	conPrintIP6ADDR(prefix_ptr, 1);
	conPCRLF();
	conPrintROMString("Prefix Length: ");
	conPrintUINT8(prefix_len);
	conPCRLF();
	
	return TRUE;

}
#endif

//called when the slow timer interrupt occurs
#ifdef LOWSN_ENABLE_SLOW_TIMER
void usrSlowTimerInt(void ) {}
#endif


//general interrupt callback , when this is called depends on the HAL layer.
void usrIntCallback(void){}

///huangyi
extern UINT8 numNeighbor;
extern SADDR   childTopoSADDR;
extern SADDR   parentTopoSADDR;
extern uint8_t childTopoType;
extern uint8_t parentTopoType;


TOPO_RELA_ADDRESS topolist;
#ifdef LOWSN_COORDINATOR
uint8_t isNodeExist = 0;
static uint8_t neiborTotalNum =1 ;
void relaJudge()
{
   
   topolist.neighborNum = neiborTotalNum;  //邻居节点总数量 需要剪掉父节点本身
   for(int i = 0;i<topolist.neighborNum;i++){
    if(topolist.relaNode[i].dst_address ==  childTopoSADDR)       //判断节点是否已经添加
    {
      return;
    }
   }
   for(int i = neiborTotalNum-1;i<topolist.neighborNum;i++){
      topolist.relaNode[i].src_address  = parentTopoSADDR;
      topolist.relaNode[i].src_category = parentTopoType;
      
      topolist.relaNode[i].dst_category = childTopoType;
      topolist.relaNode[i].dst_address  = childTopoSADDR;
      neiborTotalNum++;
    
   }
   
   halWait(1000); //延时1s，保证上位机收到的为单独一个字符串，权宜之计
   DEBUG_STRING(DBG_INFO, "\n");
   for(int i=0;i<topolist.neighborNum;i++)   ///huangyi
   {
     //conPCRLF();
     conPrintROMString("Relationship address Number is:");
     conPrintUINT8(i);
     //conPCRLF();
     //父节点信息
     conPrintROMString("Parent address is:")
     conPrintUINT16(topolist.relaNode[i].src_address);
     //conPCRLF();
     conPrintROMString("Parent category is:")
     conPrintUINT16(topolist.relaNode[i].src_category);
     //conPCRLF();
     //子节点
     conPrintROMString("Child address is:")
     //conPrintUINT16(topolist.relaNode[i].dst_address);
     DEBUG_UINT16(DBG_INFO,topolist.relaNode[i].dst_address);
     
     //conPCRLF();
     conPrintROMString("Child category is:")
     conPrintUINT16(topolist.relaNode[i].dst_category);
     //conPCRLF();
     conPrintROMString("Number of total address relationship is:");
     conPrintUINT8(topolist.neighborNum);
     //conPCRLF();
   }
   DEBUG_STRING(DBG_INFO, "\n");
   halWait(1000);

}
#endif

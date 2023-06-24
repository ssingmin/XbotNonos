/*
 * app_can.c
 *
 *  Created on: 2023. 6. 24.
 *      Author: ssingmin
 */


#include "app_can.h"

uint32_t FLAG_RxCplt = 0;

int8_t					g_uCAN_Rx_Data[6][8] = {0,};//6 is trash can. never used
CAN_RxHeaderTypeDef 	g_tCan_Rx_Header[6];//6 is trash can. never used

CAN_FilterTypeDef       sFilterConfig;
CAN_FilterTypeDef       sFilterConfig2;

void CAN_disableirq(void){HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);}
void CAN_enableirq(void){HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);/*HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);*/}

void CanInit(uint32_t id, uint32_t mask, uint8_t EXT_Select)
{
	#if 0//example idlist mode
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;	//receive only canid of 1002, 5001
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 1002<<5;	//for receive id about 1002(pc)
    sFilterConfig.FilterIdLow = 0x0000;		//for receive id about 1002(pc)
    sFilterConfig.FilterMaskIdHigh = (5001<<3)>>16;		//for receive id about 5001(bottom board)
    sFilterConfig.FilterMaskIdLow = ((5001<<3)&0xffff)|(EXT_Select<<2);		//for receive id about 5001(bottom board)
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

	#else//example idmask mode
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//    sFilterConfig.FilterIdHigh = (id<<3)>>16;
//    sFilterConfig.FilterIdLow = ((id<<3)&0xffff)|(EXT_Select<<2);//(0x1<<2) is extended id check register
//    sFilterConfig.FilterMaskIdHigh = (mask<<3)>>16;
//    sFilterConfig.FilterMaskIdLow = ((mask<<3)&0xffff)|(EXT_Select<<2);
    sFilterConfig.FilterIdHigh = (id<<5);
    sFilterConfig.FilterIdLow = 0;//(0x1<<2) is extended id check register
    sFilterConfig.FilterMaskIdHigh = (mask<<5);
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;
    #endif

    if (HAL_CAN_Start(&hcan1) != HAL_OK){Error_Handler();}/* Start Error */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){while(1){;}}

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
		/* Filter configuration Error */
		Error_Handler();
    }
}


void sendCan(uint32_t ID, int8_t *buf, uint8_t len, uint8_t ext)
{

	CAN_TxHeaderTypeDef tCan_Tx_Header;

    uint32_t dwTxMailBox;
    uint32_t dwCheck;

    tCan_Tx_Header.StdId = ID;//for send id 3001
	tCan_Tx_Header.ExtId = ID;//for send id 3001
	tCan_Tx_Header.RTR = CAN_RTR_DATA;
	tCan_Tx_Header.IDE = ext ? CAN_ID_EXT : CAN_ID_STD;
	tCan_Tx_Header.DLC = len;
	tCan_Tx_Header.TransmitGlobalTime = DISABLE;

    dwTxMailBox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);	//resolve the error situation
    //printf("%d: cantx \n", osKernelGetTickCount());
    if(dwTxMailBox == 0){}
    else
    {
        dwCheck = HAL_CAN_AddTxMessage(&hcan1, &tCan_Tx_Header, buf, &dwTxMailBox);
        if(dwCheck != HAL_OK){while(1){;}}
    }
    //HAL_Delay(1);//must be
}

void SDOMsg(uint8_t Node_id,uint16_t index, uint8_t subindex, uint32_t msg, uint8_t len)
{
	uint8_t buf[8]={0,};

	switch (len) {
		case 1:
			buf[0]=0x2f;	break;	//1byte
		case 2:
			buf[0]=0x2b;	break;	//2byte
		case 3:
			buf[0]=0x27;	break;	//3byte
		case 4:
			buf[0]=0x23;	break;	//4byte
	}

	memcpy(buf+1,&index,2);	//index
	buf[3]=subindex;		//subindex
	memcpy(buf+4,&msg,len);	//data

	sendCan(0x600+Node_id,buf,8,0);
}

void NMT_Mode(uint8_t command, uint8_t Node_id)// command 1= pre-operation, 2=operation
{
	uint8_t buf[8]={0,};


	if(command == 1){buf[0]=0x80;}//enter nmt pre-operational command
	else{buf[0]=0x01;}//enter nmt operational command for PDO operation
	buf[1]=Node_id;//node id

	sendCan(0, buf, 8, 0);
}


int PDOMapping(uint8_t Node_id, uint16_t PDO_index, MappingPar Param, uint8_t Num_entry)//entry rr
{
	uint32_t tmp = 0;
	uint16_t tmp_TxRx = 0;
	uint8_t type = 0;

	if(Num_entry>=5){printf("Num_entry error: %d\n", Num_entry); return 0;}

	if(PDO_index>=0x1600&&PDO_index<=0x17ff){tmp_TxRx=0x200+0x100*(PDO_index-0x1600); type=0xff;}
	else if(PDO_index>=0x1a00&&PDO_index<=0x1bff) {
		tmp_TxRx=0x180+0x100*(PDO_index-0x1a00);
		if(Param.option==0){type=0xfe;}
		else {type=0xff;}
		}
	else {printf("PDO_index error: %d\n", PDO_index); return 0;}

	NMT_Mode(PRE_OPERATION, Node_id);//pre-operation mode

	for(int i=0;i<Num_entry;i++) {//clear rpdo0 mapping, 0x60ff(index) 03(subindex) 20(length)
		SDOMsg(Node_id, PDO_index, 0, 0, 1);//clear rpdo0 mapping
		tmp=(0x10000*Param.index[i])+(0x100* Param.subindex[i])+(Param.length[i]);
		SDOMsg(Node_id, PDO_index, i+1, tmp, 4);
		SDOMsg(Node_id, PDO_index-0x200, 1, tmp_TxRx+Node_id, 4);//cob-id??
		SDOMsg(Node_id, PDO_index-0x200, 2, type, 1);//transmission type, fix asynchronous with 0xff
		SDOMsg(Node_id, PDO_index-0x200, 3+(Param.option*2), Param.option_time, 2);//not necessary 3= inhibit mode, 5=event timer mode
		SDOMsg(Node_id, PDO_index, 0, 0x01, 1);//set rpdo0 mapping
	}

	NMT_Mode(OPERATION, Node_id);//operation mode

	return 1;
}

void PDOMsg(uint8_t Node_id, uint16_t PDO_index, uint8_t *buf, uint8_t length)
{
	sendCan((PDO_index-0x1800)+Node_id,buf,length,0);
}

void Vel_PDOMsg(uint8_t Node_id, uint16_t PDO_index, uint16_t vel_left, uint16_t vel_right)
{
	uint8_t buf[8];

	buf[0]=(uint8_t)vel_left;
	buf[1]=(uint8_t)(vel_left>>8);
	buf[2]=(uint8_t)vel_right;
	buf[3]=(uint8_t)(vel_right>>8);

	PDOMsg(Node_id, PDO_index, buf, 4);
}


void Tor_OnOff(uint8_t OnOff)
{
	if(OnOff==1){
		for(int i=0;i<2;i++){
			SDOMsg(i+1,0x6040, 0x0, 0x00, 2);//Node_id, index,  subindex,  msg,  len//Initialization step 0: At this time, the low 4-bit status of 6041 is 0000, motor is released;
			SDOMsg(i+1,0x6040, 0x0, 0x06, 2);//Node_id, index,  subindex,  msg,  len//Initialization step 1: At this time, the low 4-bit status of 6041 is 0001, motor is released;
			SDOMsg(i+1,0x6040, 0x0, 0x07, 2);//Node_id, index,  subindex,  msg,  len//Initialization step 2: At this time, the low 4-bit status of 6041 is 0011, motor is enabled;
			SDOMsg(i+1,0x6040, 0x0, 0x0f, 2);//Node_id, index,  subindex,  msg,  len//Initialization step 3: At this time, the low 4-bit status of 6041 is 0111, motor is enabled;
		}
	}
	else{for(int i=0;i<2;i++){SDOMsg(i+1,0x6040, 0x0, 0x00, 2);}}//Node_id, index,  subindex,  msg,  len//Initialization step 0: At this time, the low 4-bit status of 6041 is 0000, motor is released;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
	//printf("%d: canrx \n", HAL_GetTick());
	if(FLAG_RxCplt<5)
	{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_tCan_Rx_Header[FLAG_RxCplt], g_uCAN_Rx_Data[FLAG_RxCplt]) != HAL_OK){while(1){;}}
//		printf("%d: RF %d %d %d\n", osKernelGetTickCount(),
//				g_tCan_Rx_Header[FLAG_RxCplt].StdId, g_tCan_Rx_Header[FLAG_RxCplt].ExtId, g_tCan_Rx_Header[FLAG_RxCplt].IDE);
		FLAG_RxCplt++;
	}
	else{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_tCan_Rx_Header[6], g_uCAN_Rx_Data[6]) != HAL_OK){while(1){;}}
//		printf("%d: RF_TC %d %d %d\n", osKernelGetTickCount(),
//						g_tCan_Rx_Header[FLAG_RxCplt].StdId, g_tCan_Rx_Header[FLAG_RxCplt].ExtId, g_tCan_Rx_Header[FLAG_RxCplt].IDE);
	}


}

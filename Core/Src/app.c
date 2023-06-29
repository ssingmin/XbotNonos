/*
 * app.c
 *
 *  Created on: Jun 3, 2023
 *      Author: ssingmin
 */
#include"app.h"

#define VERSION_MAJOR 2
#define VERSION_MINOR 5

#define TAR_RPM	7
#define MS_PER_DEG	(1000/(6*TAR_RPM))
//#define MS_PER_DEG	11.11
#define RES_SM	100	//SM= STEERING MOTOR
#define LIMIT_MODE_C 300//300=50deg, 460=30deg, 280=55deg

#define DELAYTIME 4//0.5s per 1

extern uint8_t tmp_rx[4][SERVO_RXBUFLEN];
extern int flag_rx;
extern uint8_t monitorirq;
uint32_t g_tick_1ms=0;
uint32_t g_tick_5ms=0;
uint32_t g_tick_100ms=0;
uint32_t g_tick_500ms=0;
uint32_t g_tick_1000ms=0;

uint8_t g_PS_SIGx_Pin = 0;	//0000 4321


uint8_t g_buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left

MappingPar vel_RxPDO0={{0x60ff,0,0,0},//index //target speed
						{0x03,0,0,0},//subindex //left and rigt target speed combination
						{0x20,0,0,0},//length //32bit
						0x01,//option//event timer
						500};//option_time //500

MappingPar vel_TxPDO0={{0x606C,0,0,0},//index //target speed
						{0x03,0,0,0},//subindex //left and rigt target speed combination
						{0x20,0,0,0},//length //32bit
						0x01,//option
						200};//option_time //inhibit time 10000, event time 1000 = 500ms
//						200};//option_time //inhibit time 10000, event time 1000 = 500ms

MappingPar vel_TxPDO1={{0x603F,0,0,0},//index //error code
						{0x00,0,0,0},//subindex //left and rigt target speed combination
						{0x10,0,0,0},//length //16bit
						0x00,//option
						2000};//option_time //inhibit time 10000, event time 1000 = 500ms

uint32_t STM_FT_ID[4][2] = {	{312,135},	//id1 cw = 3.12, ccw = 1.35 degree
							{0,400},	//id2 cw = 0.0, ccw = 4.0 degree
							{0,500},	//id3 cw = 0.0, ccw = 5.0 degree
							{400,100}};	//id4 cw = 4.0, ccw = 1.0 degree

int16_t g_Tmp_cmd_FL = 0;
int16_t g_Tmp_cmd_FR = 0;
int16_t g_Tmp_cmd_RL= 0;
int16_t g_Tmp_cmd_RR = 0;

double g_angle_rad_c;
double g_angle_rad_i;
double g_angle_rad_o;

double g_Real_cmd_v_x = 0;
double g_Real_cmd_v_y = 0;
double g_Real_cmd_w = 0;

double g_real_angle_c;
double g_real_angle_i;
double g_real_angle_o;


double g_Tar_cmd_v_x = 0;
double g_Tar_cmd_v_i = 0;
double g_Tar_cmd_v_o = 0;
double g_Tar_cmd_v_y = 0;
double g_Tar_cmd_w = 0;

int16_t g_temp_x = 0;
int16_t g_temp_y = 0;
int16_t g_temp_w = 0;

uint8_t g_torqueSW = 0;

int8_t g_state_stop = 0;
int8_t g_state_stop_tmp = 0;
uint32_t g_Stop_flag = 0;

int8_t g_sendcanbuf[8]={0,};

int8_t g_canbuf[8]={0,};

uint32_t g_CanId = 0;

int16_t g_Tar_cmd_FL = 0;//Front Left
int16_t g_Tar_cmd_FR = 0;//Front Right
int16_t g_Tar_cmd_RL= 0;//Rear Left
int16_t g_Tar_cmd_RR = 0;//Rear Right

uint8_t g_ModeABCD = 4;//4 is stop mode
uint8_t g_Pre_ModeABCD = 0;

//uint8_t g_EndMode = 2;
//uint8_t g_timerflag = 2;

uint32_t g_EndMode = 2;
uint32_t g_timerflag = 2;
uint32_t g_delaytime = 4;//0.5s per 1

int16_t g_SteDeg[4] = {0,};	//steering degree unit=0.01 degree

int32_t g_SAngle[4] = {0,};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//. generate per 500ms
{
  if(htim->Instance == TIM6)
  {
		g_timerflag++;
		g_EndMode++;
		//printf("TCB %d %d\n", g_timerflag, g_EndMode);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == PS_SIG1_Pin) {
    	g_PS_SIGx_Pin |= 0b00000001;
    	printf("GPIO_EXTI_Callback PS_SIG1_Pin.\n");
	}

    if(GPIO_Pin == PS_SIG2_Pin) {
    	g_PS_SIGx_Pin |= 0b00000010;
    	printf("GPIO_EXTI_Callback PS_SIG2_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG3_Pin) {
    	g_PS_SIGx_Pin |= 0b00000100;
    	printf("GPIO_EXTI_Callback PS_SIG3_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG4_Pin) {
    	g_PS_SIGx_Pin |= 0b00001000;
    	printf("GPIO_EXTI_Callback PS_SIG4_Pin.\n");
    }
}

void SMotorSet()
{

	uint8_t Dir_Rot = 0; //direction of rotation
	uint8_t FT_flag = 0; //FineTuning_flag
	uint8_t STinitdone = 0;
	uint8_t EndInit = 0;

	for(int i=0;i<4;i++){
		if(HAL_GPIO_ReadPin(GPIOA, ((1<<i)<<4))){//GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
			if((i==STMotorID2) || (i==STMotorID3)) 	{Dir_Rot = SERVO_CW;}
			else {Dir_Rot = SERVO_CCW; FT_flag |= (1<<i);}
		}
		else {
			if((i==STMotorID2) || (i==STMotorID3))	{Dir_Rot = SERVO_CCW;FT_flag |= (1<<i);}
			else {Dir_Rot = SERVO_CW;}
		}
		DataSetSteering(g_buf, i, Dir_Rot, RPM_1, SERVO_INIT, INIT_SPEED);// i= STMotorIDx, x=1~4
		printf("PS_SIG1_Pin ccw init. %d %x\n", FT_flag, ((1<<i)<<4));
	}

	for(int i=0;i<400;i++){
		HAL_Delay(10);

		if((i%20)==0){
			ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
			printf("%d ", i);
		}


		if(g_PS_SIGx_Pin&1){//1ch init
			g_PS_SIGx_Pin &= ~(1); printf(" PS_SIG1_stop.\n");
			DataSetSteering(g_buf, 0, SERVO_CCW, 0, 0, 30);
			EndInit |= 1;
			ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
		}

		if(g_PS_SIGx_Pin&2){//2ch init
			g_PS_SIGx_Pin &= ~(2); printf(" PS_SIG2_stop.\n");
			DataSetSteering(g_buf, 1, SERVO_CCW, 0, 0, 30);
			EndInit |= 2;
			ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
		}

		if(g_PS_SIGx_Pin&4){//3ch init
			g_PS_SIGx_Pin &= ~(4); printf(" PS_SIG3_stop.\n");
			DataSetSteering(g_buf, 2, SERVO_CCW, 0, 0, 30);
			EndInit |= 4;
			ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
		}

		if(g_PS_SIGx_Pin&8){//4ch init
			g_PS_SIGx_Pin &= ~(8); printf(" PS_SIG4_stop.\n");
			DataSetSteering(g_buf, 3, SERVO_CCW, 0, 0, 30);
			EndInit |= 8;
			ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms

			printf("EndInit %d\n", EndInit);
		}

		if(EndInit == 15) {

			//GPIO_disableirq();
			STinitdone++;
			printf("%d: EndInit == 15.\n", HAL_GetTick());
			//EndInit = 0;
		}
		if(STinitdone){printf("steering origin init done!!!.\n"); break;}

		if(i==399){
			HAL_Delay(100);
			printf("steering origin init failed reset!!!!.\n");
			HAL_Delay(100);
			//NVIC_SystemReset();
		}
	}

	for(int i=0;i<4;i++){
		if(FT_flag&(1<<i)){
			DataSetSteering(g_buf, i, SERVO_CW, STM_FT_ID[i][SERVO_CW], SERVO_POS, INIT_SPEED);
			printf("SERVO_cW\n");
		}
		else {
			DataSetSteering(g_buf, i, SERVO_CCW, STM_FT_ID[i][SERVO_CCW], SERVO_POS, INIT_SPEED);
			printf("SERVO_ccW\n");
		}
		g_PS_SIGx_Pin |= (1<<i);
	}

	for(int i=0;i<10;i++){
		ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
		HAL_Delay(500);
		}

	for(int i=0;i<4;i++){
		if(FT_flag&(1<<i)){
			DataSetSteering(g_buf, i, SERVO_CW, 0, SERVO_INIT, INIT_SPEED);
			printf("SERVO_cW\n");
		}
		else {
			DataSetSteering(g_buf, i, SERVO_CCW, 0, SERVO_INIT, INIT_SPEED);
			printf("SERVO_ccW\n");
		}
		g_PS_SIGx_Pin |= (1<<i);
	}

	for(int i=0;i<10;i++){
		ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
		HAL_Delay(500);
		}


	Dir_Rot = 0;//init

}
void CancommSet()
{
	CanInit(FILTERID,MASKID,STDID);//must be to use it

	PDOMapping(1, RxPDO0, vel_RxPDO0, 1);
	PDOMapping(2, RxPDO0, vel_RxPDO0, 1);

	PDOMapping(1, TxPDO0, vel_TxPDO0, 1);//event time mode 100ms
	PDOMapping(2, TxPDO0, vel_TxPDO0, 1);//event time mode
	PDOMapping(1, TxPDO1, vel_TxPDO1, 1);//inhibit mode 100ms
	PDOMapping(2, TxPDO1, vel_TxPDO1, 1);//inhibit mode

	for(int i=0;i<2;i++){
		SDOMsg(i+1,0x2010, 0x0, 0x01, 1);//Node_id, index,  subindex,  msg,  len//save eeprom
		SDOMsg(i+1,0x6060, 0x0, 0x03, 1);//Node_id, index,  subindex,  msg,  len//3: Profile velocity mode;
		Tor_OnOff(TORQUEON);
		SDOMsg(i+1,0x200f, 0x0, 0x01, 2);//Node_id, index,  subindex,  msg,  len//1e: Synchronization control
	}
}

void Cal_Real_cmd(void)
{

	double tempL;
	double tempR;

	tempL=(double)(g_Tmp_cmd_FL+g_Tmp_cmd_RL)/(2*10);
	tempR=-(double)(g_Tmp_cmd_FR+g_Tmp_cmd_RR)/(2*10);

	if(g_angle_rad_c == 0){

	//g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
	g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));

	}
	else{
		if((tempL<tempR)  &&  ((tempL>0) && (tempR>0))){
			if((sin(g_real_angle_c)<0.1) && (sin(g_real_angle_c)>-0.1))
			{
				g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
				//printf("%d:g_Real_cmd_v_x 221 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}
			else{
				g_Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(g_real_angle_i)/sin(g_real_angle_c))*tempL)
								+((sin(g_real_angle_o)/sin(g_real_angle_c))*tempR));
				//printf("%d:g_Real_cmd_v_x 222 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}

		}

		else if((tempL>tempR)  &&  ((tempL>0) && (tempR>0))){
			if((sin(g_real_angle_c)<0.1) && (sin(g_real_angle_c)>-0.1)){
				g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
							//	printf("%d:g_Real_cmd_v_x 331 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}
			else{
			g_Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(g_real_angle_i)/sin(g_real_angle_c))*tempR)
							+((sin(g_real_angle_o)/sin(g_real_angle_c))*tempL));
			//printf("%d:g_Real_cmd_v_x 332 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}
		}

		else if((tempL<tempR)  &&  ((tempL<0) && (tempR<0))){
			if((sin(g_real_angle_c)<0.1) && (sin(g_real_angle_c)>-0.1)){
				g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
				//				printf("%d:g_Real_cmd_v_x 441 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}
			else{
			g_Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(g_real_angle_i)/sin(g_real_angle_c))*tempR)
				+((sin(g_real_angle_o)/sin(g_real_angle_c))*tempL));
			//printf("%d:g_Real_cmd_v_x 442 %f\n", HAL_GetTick(), g_Real_cmd_v_x);
			}
		}

		else if((tempL>tempR)  &&  ((tempL<0) && (tempR<0))){
			if((sin(g_real_angle_c)<0.1) && (sin(g_real_angle_c)>-0.1)){
				g_Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
			//	printf("%d:g_Real_cmd_v_x 551 %f %f %f %f %f\n", HAL_GetTick(), g_Real_cmd_v_x, tempL, tempL, sin(g_real_angle_i), sin(g_real_angle_c));
			}
			else{
			g_Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(g_real_angle_i)/sin(g_real_angle_c))*tempL)
				+((sin(g_real_angle_o)/sin(g_real_angle_c))*tempR));
		//	printf("%d:g_Real_cmd_v_x 552 %f\n", HAL_GetTick(), g_Real_cmd_v_x);
			}
		}
	}

	if((g_Tmp_cmd_FL>=0) && (g_Tmp_cmd_FR>=0)  ||  (g_Tmp_cmd_FL<=0) && (g_Tmp_cmd_FR<=0))//mode C
	{
		g_Real_cmd_w = -(CONSTANT_C_AxC_V*((tempL-tempR)/2));
	}
	else	//mode B
	{
//		g_Real_cmd_w = (C_4PIRxINv60WB*((tempL+tempR)/2)*fabs(sin(g_angle_rad_c)))*1000;

		if		((tempL<tempR)  &&  ((tempL>0) && (tempR>0))){g_Real_cmd_w = ((g_Real_cmd_v_x*sin(g_real_angle_c))/230)*1000;}
		else if	((tempL>tempR)  &&  ((tempL>0) && (tempR>0))){g_Real_cmd_w = -((g_Real_cmd_v_x*sin(g_real_angle_c))/230)*1000;}
		else if	((tempL<tempR)  &&  ((tempL<0) && (tempR<0))){g_Real_cmd_w = -((g_Real_cmd_v_x*sin(g_real_angle_c))/230)*1000;}
		else if	((tempL>tempR)  &&  ((tempL<0) && (tempR<0))){g_Real_cmd_w = ((g_Real_cmd_v_x*sin(g_real_angle_c))/230)*1000;}
		printf("%d:g_Real_cmd_ww 552 %d %d %d %d %d %f %f\n", HAL_GetTick(),
				(int)g_Real_cmd_w, (int )g_Real_cmd_v_x, (int)(g_real_angle_c*1000), (int)(g_real_angle_i*1000), (int)(g_real_angle_o*1000), tempL, tempR);
	}

	g_sendcanbuf[5] = (((int16_t)(g_Real_cmd_w)))>>8 & 0xff;
	g_sendcanbuf[4] = (int16_t)(g_Real_cmd_w)&0xff;
	g_sendcanbuf[3] = (((int16_t)(g_Real_cmd_v_y)))>>8 & 0xff;
	g_sendcanbuf[2] = (int16_t)(g_Real_cmd_v_y)&0xff;
	g_sendcanbuf[1] = (((int16_t)(g_Real_cmd_v_x)))>>8 & 0xff;
	g_sendcanbuf[0] = (int16_t)(g_Real_cmd_v_x)&0xff;
}


void Canparsing()
{
	if(FLAG_RxCplt>0)	//real time, check stdid, extid
	{
		while(FLAG_RxCplt>0){

			FLAG_RxCplt--;
			for(int i=0;i<8;i++){g_canbuf[i] = g_uCAN_Rx_Data[FLAG_RxCplt][i];}
			if(g_tCan_Rx_Header[FLAG_RxCplt].StdId>g_tCan_Rx_Header[FLAG_RxCplt].ExtId){g_CanId = g_tCan_Rx_Header[FLAG_RxCplt].StdId;}//�????????????????체크
			else {g_CanId = g_tCan_Rx_Header[FLAG_RxCplt].ExtId;}


			switch(g_CanId)//parse
			{
				case 0x3E9:
					g_temp_x = (((int16_t)g_canbuf[1])<<8) | ((int16_t)g_canbuf[0])&0xff;
					g_temp_y = (((int16_t)g_canbuf[3])<<8) | ((int16_t)g_canbuf[2])&0xff;
					g_temp_w = (((int16_t)g_canbuf[5])<<8) | ((int16_t)g_canbuf[4])&0xff;
					if(g_canbuf[7]==1){//stop mode
						g_state_stop = g_canbuf[7];
						g_state_stop_tmp = 1;
						}
					else if(g_canbuf[7]==0){g_state_stop_tmp = 0;}//move mode
					// g_state_stop = g_canbuf[7];
					g_Tar_cmd_v_x = (double)g_temp_x;
					g_Tar_cmd_v_y = (double)g_temp_y;
					g_Tar_cmd_w = (double)g_temp_w;
					g_torqueSW = g_canbuf[6];
					printf("g_temp_x: %d\n", g_temp_x);

					g_Stop_flag++;

					break;

				case 0x181:
					g_Tmp_cmd_FL = (((int16_t)g_canbuf[1])<<8) | ((int16_t)g_canbuf[0])&0xff;
					g_Tmp_cmd_FR = (((int16_t)g_canbuf[3])<<8) | ((int16_t)g_canbuf[2])&0xff;
					//printf("0x181 %d\n", g_Tmp_cmd_FL);
					break;

				case 0x182:
					g_Tmp_cmd_RL = (((int16_t)g_canbuf[1])<<8) | ((int16_t)g_canbuf[0])&0xff;
					g_Tmp_cmd_RR = (((int16_t)g_canbuf[3])<<8) | ((int16_t)g_canbuf[2])&0xff;
					break;

				case 2002:

					break;
			}

			g_tCan_Rx_Header[FLAG_RxCplt].StdId=0;
			g_tCan_Rx_Header[FLAG_RxCplt].ExtId=0;
			g_CanId = 0;

			//for(int i=0;i<8;i++){g_canbuf[i]=0;}
		}

	}
}

int16_t rad2deg(double radian)
{
    return (int16_t)(radian*180/MATH_PI);
}

double deg2rad(int16_t degree)
{
    return (double)(degree*MATH_PI/180);
}

void candataset()
{
	if(g_state_stop==0){g_delaytime=0;}
	else {g_delaytime = DELAYTIME;}

	if((g_temp_w && (g_temp_x==0) && (g_temp_y==0)) || ( fabs((g_Tar_cmd_v_x*1000)/g_Tar_cmd_w)<LIMIT_MODE_C) )//MODE C
	{
		g_ModeABCD = 3;
		if((g_Pre_ModeABCD!=g_ModeABCD) || (g_EndMode<g_delaytime)){
			if(g_Pre_ModeABCD!=g_ModeABCD){g_EndMode = 0;}
			g_Pre_ModeABCD = g_ModeABCD;
			g_Tar_cmd_RR = g_Tar_cmd_RL = g_Tar_cmd_FR = g_Tar_cmd_FL=0;
			for(int i=0;i<4;i++){g_SteDeg[i]=rad2deg(ANGLE_VEL);}
			g_state_stop = 0;
		}
		else {
			//g_ModeABCD = 3;
			g_Tar_cmd_v_x=0;
			g_Tar_cmd_v_y=0;

			g_Tar_cmd_FL = -1*((g_Tar_cmd_w*CONSTANT_C_AxC_V)/SIGNIFICANT_FIGURES);

			if(g_Tar_cmd_FL>LIMIT_W){g_Tar_cmd_FL=LIMIT_W;}
			if(g_Tar_cmd_FL<-LIMIT_W){g_Tar_cmd_FL=-LIMIT_W;}
			g_Tar_cmd_RR = g_Tar_cmd_RL = g_Tar_cmd_FR = g_Tar_cmd_FL;

			//for(int i=0;i<4;i++){g_SteDeg[i]=rad2deg(ANGLE_VEL);}

		}
	}

	else //mode2
	{
		//printf("%d mode21 %d \n", HAL_GetTick(), g_SteDeg[0]);
		g_ModeABCD = 2;//B mode
		if((g_Pre_ModeABCD!=g_ModeABCD) || (g_EndMode<g_delaytime)){
			if(g_Pre_ModeABCD!=g_ModeABCD){g_EndMode = 0;}
			g_Pre_ModeABCD = g_ModeABCD;
			g_Tar_cmd_RR = g_Tar_cmd_RL = g_Tar_cmd_FR = g_Tar_cmd_FL=0;
			g_angle_rad_i = 0;
			g_angle_rad_o = 0;
			
			g_SteDeg[0]=0;
			g_SteDeg[1]=0;
			g_SteDeg[2]=0;
			g_SteDeg[3]=0;

			g_state_stop = 0;
		}
		else{
		if(g_Tar_cmd_v_x>LIMIT_V){g_Tar_cmd_v_x=LIMIT_V;}
		if(g_Tar_cmd_v_x<-LIMIT_V){g_Tar_cmd_v_x=-LIMIT_V;}


		g_angle_rad_c = fabs(asin(((230*g_Tar_cmd_w) / (g_Tar_cmd_v_x*1000))));

		g_angle_rad_i = fabs(atan2(230,(230 / tan(g_angle_rad_c))-209.5));
		g_angle_rad_o = fabs(atan2(230,(230 / tan(g_angle_rad_c))+209.5));

		g_Tar_cmd_v_i = (g_Tar_cmd_v_x*sin(g_angle_rad_c)) / sin(g_angle_rad_i);
		g_Tar_cmd_v_o = (g_Tar_cmd_v_x*sin(g_angle_rad_c)) / sin(g_angle_rad_o);

		if(g_temp_w==0){
			g_Tar_cmd_v_i=g_Tar_cmd_v_o=g_Tar_cmd_v_x;
			g_angle_rad_i=g_angle_rad_o=g_angle_rad_c=0;

			g_Tar_cmd_FL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_RL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);

			g_SteDeg[0]=0;
			g_SteDeg[1]=0;
			g_SteDeg[2]=0;
			g_SteDeg[3]=0;

		}

		if((g_temp_w>0) && (g_temp_x>0)){
			g_Tar_cmd_FL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_RL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);

			g_SteDeg[0]=rad2deg(g_angle_rad_o);
			g_SteDeg[1]=rad2deg(g_angle_rad_i);
			g_SteDeg[2]=rad2deg(g_angle_rad_o);
			g_SteDeg[3]=rad2deg(g_angle_rad_i);


		}

		else if((g_temp_w<0) && (g_temp_x>0)){
			g_Tar_cmd_FL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_RL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);

			g_SteDeg[0]=rad2deg(g_angle_rad_i);
			g_SteDeg[1]=rad2deg(g_angle_rad_o);
			g_SteDeg[2]=rad2deg(g_angle_rad_i);
			g_SteDeg[3]=rad2deg(g_angle_rad_o);

		}

		else if((g_temp_w>0) && (g_temp_x<0)){
			g_Tar_cmd_FL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_RL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);

			g_SteDeg[0]=rad2deg(g_angle_rad_i);
			g_SteDeg[1]=rad2deg(g_angle_rad_o);
			g_SteDeg[2]=rad2deg(g_angle_rad_i);
			g_SteDeg[3]=rad2deg(g_angle_rad_o);

		}

		else if((g_temp_w<0) && (g_temp_x<0)){
			g_Tar_cmd_FL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_RL = (int16_t)(C_60xINv2PIR * g_Tar_cmd_v_i);
			g_Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);
			g_Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * g_Tar_cmd_v_o);


			g_SteDeg[0]=rad2deg(g_angle_rad_o);
			g_SteDeg[1]=rad2deg(g_angle_rad_i);
			g_SteDeg[2]=rad2deg(g_angle_rad_o);
			g_SteDeg[3]=rad2deg(g_angle_rad_i);

		}
	}

}

//	if(g_state_stop == 1)
//	{
//		if(((g_temp_x==0) && (g_temp_y==0) && (g_temp_w==0))  ||  (g_Stop_flag==1))//mode 4
//		{
//
//			g_ModeABCD = 4;//temp
//			g_Pre_ModeABCD = 4;//temp
//			g_Tar_cmd_RR = g_Tar_cmd_RL = g_Tar_cmd_FR = g_Tar_cmd_FL=0;
//
//			for(int i=0;i<4;i++){g_SteDeg[i] = rad2deg(ANGLE_VEL);}
//
//		}
//	}
			if(g_state_stop_tmp == 1)//mode 4
			{

				g_ModeABCD = 4;//temp
				g_Pre_ModeABCD = 4;//temp
				g_Tar_cmd_RR = g_Tar_cmd_RL = g_Tar_cmd_FR = g_Tar_cmd_FL=0;

				for(int i=0;i<4;i++){g_SteDeg[i] = rad2deg(ANGLE_VEL);}

			}

	g_sendcanbuf[7] = VERSION_MINOR;
	g_sendcanbuf[6] = VERSION_MAJOR;
}


void candatasend()
{
	Vel_PDOMsg(1, TxPDO0, g_Tar_cmd_FL, g_Tar_cmd_FR);
	Vel_PDOMsg(2, TxPDO0, g_Tar_cmd_RL, g_Tar_cmd_RR);

	sendCan(0x7D1, g_sendcanbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)
}

void STProcess()//steering process
{
	uint8_t Dir_Rot = 0; //direction of rotation
	uint8_t FT_flag = 0; //FineTuning_flag
	uint8_t send_flag = 0; //FineTuning_flag
	uint8_t set_flag = 0; //FineTuning_flag

	int32_t angle = 0;
	int32_t pre_angle = 0;
	int32_t speed_angle = 0;

	int16_t pre_SteDeg[4] = {0,};	//steering degree unit=0.01 degree
	int16_t start_SteDeg[4] = {0,};
	int16_t end_SteDeg[4] = {0,};


	if(g_ModeABCD == 2){
		if(g_SteDeg[0] == 0){//forward, rear
			for(int i=0;i<4;i++){g_SteDeg[i]= 0;}
		}
		if		((g_Tar_cmd_v_x>0) && (g_Tar_cmd_w>0)){/*SteDeg*=1;*/							Dir_Rot=SERVO_CCW; }//the first quadrant
		else if	((g_Tar_cmd_v_x<0) && (g_Tar_cmd_w<0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CCW; }//the second quadrant
		else if	((g_Tar_cmd_v_x<0) && (g_Tar_cmd_w>0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CW; }//the third quadrant
		else if	((g_Tar_cmd_v_x>0) && (g_Tar_cmd_w<0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CW; }//the fourth quadrant

		for(int i=0;i<4;i++){
			if(g_SteDeg[i]>90){g_SteDeg[i]= 90;}//prevent over angle
		}

		if(pre_SteDeg[0] == HAL_GetTick(), g_SteDeg[0]){
			set_flag = 1;
			for(int i=0;i<4;i++){
				end_SteDeg[i] = ((g_SteDeg[i]*MS_PER_DEG)+5) / RES_SM;//+5 is round
				if(start_SteDeg[i]>end_SteDeg[i]) {g_SAngle[i] = start_SteDeg[i] - end_SteDeg[i];}
				else if (start_SteDeg[i]<end_SteDeg[i]) {g_SAngle[i] = end_SteDeg[i] - start_SteDeg[i];}
				start_SteDeg[i] = g_SAngle[i];
				//printf("%d: input data %d, %d, %d, %d\n", HAL_GetTick(),
						//g_SteDeg[i], g_SAngle[i], end_SteDeg[i] , start_SteDeg[i] );
			}
		}

		else{
			for(int i=0;i<4;i++){
				pre_SteDeg[i] = g_SteDeg[i];
				send_flag = 1;
//				printf("%d: change data %d, %d, %d, %d\n", HAL_GetTick(),
//						g_SteDeg[i], g_SAngle[i], end_SteDeg[i] , start_SteDeg[i] );
			}
		}

		DataSetSteering(g_buf, STMotorID1, Dir_Rot, g_SteDeg[0]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID2, Dir_Rot, g_SteDeg[1]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID3, Dir_Rot^1, g_SteDeg[2]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID4, Dir_Rot^1, g_SteDeg[3]*100, SERVO_PSMODE, TAR_RPM*10);
	//	printf("%d: MM %d\n", HAL_GetTick(), HAL_GetTick(), g_SteDeg[0]);
	}

	if(g_ModeABCD == 3){
//		SteDeg=rad2deg(ANGLE_VEL);
//		for(int i=0;i<4;i++){Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL), i);}
		send_flag = 1;
		set_flag = 1;
		pre_SteDeg[0] = 1; //for set send_flag of mode B
		//printf("%d: abs %d\n", HAL_GetTick(), SteDeg);
		DataSetSteering(g_buf, STMotorID1, SERVO_CCW, g_SteDeg[0]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID2, SERVO_CW, g_SteDeg[1]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID3, SERVO_CW, g_SteDeg[2]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID4, SERVO_CCW, g_SteDeg[3]*100, SERVO_PSMODE, TAR_RPM*10);
		for(int i=0;i<4;i++){
			g_SAngle[i] = ((g_SteDeg[i]*MS_PER_DEG)+5) / RES_SM;
		}
		printf("Mode c\n");
	}

	if(g_ModeABCD == 4){
		send_flag = 1;
		set_flag = 1;
		pre_SteDeg[0] = 1; //for set send_flag of mode B

		DataSetSteering(g_buf, STMotorID1, SERVO_CW, g_SteDeg[0]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID2, SERVO_CCW, g_SteDeg[1]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID3, SERVO_CCW, g_SteDeg[2]*100, SERVO_PSMODE, TAR_RPM*10);
		DataSetSteering(g_buf, STMotorID4, SERVO_CW, g_SteDeg[3]*100, SERVO_PSMODE, TAR_RPM*10);
//		EndModeD = 0;
		//osDelay(10);
		for(int i=0;i<4;i++){
			g_SAngle[i] = ((g_SteDeg[i]*MS_PER_DEG)+5) / RES_SM;
		}
		printf("Mode D\n");
	}

#if 0//testing
	if((send_flag==1) && (set_flag==1)){
		ServoMotor_writeDMA(g_buf);//use osdelay(6)*2ea
		send_flag = 0;
		set_flag = 0;
		for(int i=0;i<4;i++){
			printf("[%d] %d ", i, g_SAngle[i]);
			start_SteDeg[i] = 0;
			end_SteDeg[i] = 0;
//			g_SAngle[i] = 0;
		}

		printf("%d: writeDMA %d %d %d %d %d %d %d %d\n", HAL_GetTick(),SteDeg[0],SteDeg[1],SteDeg[2],SteDeg[3],g_SAngle[0],g_SAngle[1],g_SAngle[2],g_SAngle[3]);
	}

#else
	//origin
	ServoMotor_writeDMA(g_buf);//use osdelay(6)*2ea
#endif
	HAL_Delay(5); DataReadSteering(STMotorID1, 0xA1);
	HAL_Delay(5); DataReadSteering(STMotorID2, 0xA1);
	HAL_Delay(5); DataReadSteering(STMotorID3, 0xA1);
	HAL_Delay(5); DataReadSteering(STMotorID4, 0xA1);
}

void readSTmotor()
{
	uint8_t tmparr[4][12] = {0};
	uint8_t tempID = 0;
	uint8_t rx_checksum[4] = {0,};
	uint16_t real_angle[4] = {0,};

	for(int k=0;k<4;k++){//copy data to buffer
		for(int i=0;i<12;i++){
			if(tmp_rx[k][i]==0xFF && tmp_rx[k][(i+1)%12]==0xFE)//parsing
			{

				tempID = tmp_rx[k][(i+2)%12];
				printf("tempID: %d %d\n",tempID,(i+2)%12);
				for(int j=0;j<12;j++)
				{
//					if(i+j<12){tmparr[tempID][j]=tmp_rx[k][i+j];}
//					else {tmparr[tempID][j]=tmp_rx[k][i+j-12];}
					tmparr[tempID][j]=tmp_rx[k][(i+j)%12];
				}
			}
		}
	}

	if(flag_rx == 1){
		printf("monitorirq: %d\n", monitorirq);
		printf("tmparr[0]"); for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[0][i]);} printf("\n");
		printf("tmparr[1]"); for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[1][i]);} printf("\n");
		printf("tmparr[2]"); for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[2][i]);} printf("\n");
		printf("tmparr[3]"); for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[3][i]);} printf("\n");

		flag_rx = 0;
	}

//	for(int j=0;j<4;j++){
//		rx_checksum[j] = tmparr[j][2]+tmparr[j][3];//id+length
//
//		for(int i=5;i<tmparr[j][3]+4;i++) {
//			rx_checksum[j] += tmparr[j][i];
//		}//checksum ~(Packet 2 + Packet 3 + Packet '5' + ?? + Packet N) [1byte]
//		rx_checksum[j] ^= 0xff;//invert value. checksum done.
//
//
//		if(tmparr[j][4]==rx_checksum[j]){
//			real_angle[j] = tmparr[j][7]*0x100+tmparr[j][8];
//			//printf("%d: angle[%d]: %03d \n", HAL_GetTick(), j, real_angle[j]);
//		}
//	}

	for(int j=0;j<4;j++){
		rx_checksum[j] = tmparr[j][2]+tmparr[j][3];//id+length

		for(int i=5;i<tmparr[j][3]+4;i++) {
			rx_checksum[j] += tmparr[j][i];
		}//checksum ~(Packet 2 + Packet 3 + Packet '5' + ?? + Packet N) [1byte]
		rx_checksum[j] ^= 0xff;//invert value. checksum done.


		if(tmparr[j][4]==rx_checksum[j]){
			real_angle[j] = tmparr[j][7]*0x100+tmparr[j][8];
			//printf("%d: angle[%d]: %03d \n", HAL_GetTick(), j, real_angle[j]);
		}
	}
#if 0
	if(real_angle[0]>real_angle[1]){
		g_real_angle_i = deg2rad((double)((round((double)(real_angle[0])/100) + round((double)(real_angle[2])/100)) /2));//unit 0.01
		g_real_angle_o = deg2rad((double)((round((double)(real_angle[1])/100) + round((double)(real_angle[3])/100)) /2));//unit 0.01
	}

	else{
		g_real_angle_i = deg2rad((double)((round((double)(real_angle[1])/100) + round((double)(real_angle[3])/100)) /2));//unit 0.01
		g_real_angle_o = deg2rad((double)((round((double)(real_angle[0])/100) + round((double)(real_angle[2])/100)) /2));//unit 0.01
	}
#else
	if(real_angle[0]>real_angle[1]){
		g_real_angle_i = deg2rad((double)((round(((double)real_angle[0])/100) + round(((double)real_angle[2])/100)) /2));//unit 0.01
		g_real_angle_o = deg2rad((double)((round(((double)real_angle[1])/100) + round(((double)real_angle[3])/100)) /2));//unit 0.01
	}

	else{
		g_real_angle_i = deg2rad((double)((round(((double)real_angle[1])/100) + round(((double)real_angle[3])/100)) /2));//unit 0.01
		g_real_angle_o = deg2rad((double)((round(((double)real_angle[0])/100) + round(((double)real_angle[2])/100)) /2));//unit 0.01
	}
#endif

	g_real_angle_c = (atan2(230*tan(g_real_angle_i),230+(209.5*tan(g_real_angle_i)))
				+ atan2(230*tan(g_real_angle_o),230-(209.5*tan(g_real_angle_o))))/2;

//	printf("%d: angle %f %f %f %f %f %f %f\n", HAL_GetTick(), atan2(230*tan(g_real_angle_i),230+(209.5*tan(g_real_angle_i))),
//			atan2(230*tan(g_real_angle_o),230-(209.5*tan(g_real_angle_o))), g_real_angle_c, g_real_angle_i, g_real_angle_o);
	printf("%d: angle %f %f %f\n", HAL_GetTick(),  g_real_angle_c, g_real_angle_i, g_real_angle_o);

}


void app()
{
	int32_t DRSCounter = 0;	//DataReadSteering Counter
	int8_t DRSCounterflag = 0;	//DataReadSteering Counter

	HAL_Delay(1000);

	SMotorSet();

	CancommSet();

	ws2812AllColor(70,70,70);//r, g, b
	ws2812NumOn(NUM_NPLED);

	fanInit();
	fanOn(100);

	HAL_UART_Receive_IT(&huart3, tmp_rx[0] , 12);

	HAL_Delay(1000);
	while(1){


		uint32_t tick_ms = HAL_GetTick();
		///////////////////////////

		if((tick_ms - g_tick_1000ms) >= 1000){//status led toogle 1
			g_tick_1000ms = tick_ms;

			printf("debug cycle %d\n", tick_ms);
			HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
		}
		///////////////////////////

		///////////////////////////
		if((tick_ms - g_tick_100ms) >= 100)	//can parsing, calculator xyw 2
		{
			g_tick_100ms = tick_ms;
//			if(DRSCounter >= 4)
//			{
//				DRSCounterflag = 1;
//				DRSCounter = 0;
				printf("%d: 1 cycle start \n", HAL_GetTick());

				Canparsing();

				printf("%d: parsing end\n", HAL_GetTick());


			///////////////////////////

				candataset();
				printf("%d: candataset end \n", HAL_GetTick());

				candatasend();
				printf("%d: sendCan end\n", HAL_GetTick());


				STProcess();
				printf("%d: STProcess end\n", HAL_GetTick());

				readSTmotor();
				printf("%d: readSTmotor end\n", HAL_GetTick());

				Cal_Real_cmd();
				printf("%d: Cal_Real_cmd end\n", HAL_GetTick());

				//DRSCounterflag = 0;
//			}

		}

		if((tick_ms - g_tick_5ms) >= 5){//operation steering motor 3 //edit tick system
			g_tick_5ms = tick_ms;

//			if(DRSCounterflag == 0){
//				if(DRSCounter < 4){
//					DataReadSteering(DRSCounter, 0xA1);
//					printf("%d: DRSCounter %d\n", HAL_GetTick(), DRSCounter);
//					printf("tmp_rx: "); for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmp_rx[DRSCounter][i]);} printf("\n");
//				}
//				DRSCounter++;
//			}
			//printf("2 cycle %d\n", tick_ms);
		}
//
//		if((tick_ms - tick_1000ms) >= 1000){//operation NPled 4
//			tick_1000ms = tick_ms;
//
//			printf("3 cycle %d\n", tick_ms);
//		}
//
//		if((tick_ms - tick_1000ms) >= 1000){//operation fan 5
//			tick_1000ms = tick_ms;
//
//			printf("4 cycle %d\n", tick_ms);
//		}
//
//		if((tick_ms - tick_100ms) >= 100){//cal real angle 6
//			tick_1000ms = tick_ms;
//
//			printf("5 cycle %d\n", tick_ms);
//		}

	}


}

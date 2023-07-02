/*
 * global.c
 *
 *  Created on: Jul 2, 2023
 *      Author: ssingmin
 */


//extern uint8_t tmp_rx[4][SERVO_RXBUFLEN];
//extern int flag_rx;
//extern uint8_t monitorirq;

#include <stdint.h>
#include <app_can.h>

uint32_t g_tick_1ms=0;
uint32_t g_tick_10ms=0;
uint32_t g_tick_100ms=0;
uint32_t g_tick_500ms=0;
uint32_t g_tick_1000ms=0;

uint8_t g_PS_SIGx_Pin = 0;	//0000 4321

uint8_t g_tmparr[4][12] = {0};

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
int8_t g_modeDflag = 0;
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

uint32_t g_EndMode = 2;
uint32_t g_timerflag = 2;
uint32_t g_delaytime = 4;//0.5s per 1

int16_t g_SteDeg[4] = {0,};	//steering degree unit=0.01 degree

int32_t g_SAngle[4] = {0,};




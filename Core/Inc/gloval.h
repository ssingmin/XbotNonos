/*
 * gloval.h
 *
 *  Created on: Jul 2, 2023
 *      Author: ssingmin
 */

#ifndef INC_GLOVAL_H_
#define INC_GLOVAL_H_

extern uint8_t tmp_rx[4][SERVO_RXBUFLEN];
extern int flag_rx;
extern uint8_t monitorirq;

extern uint32_t g_tick_1ms;
extern uint32_t g_tick_10ms;
extern uint32_t g_tick_100ms;
extern uint32_t g_tick_500ms;
extern uint32_t g_tick_1000ms;

extern uint8_t g_PS_SIGx_Pin;	//0000 4321

extern uint8_t g_tmparr[4][12];

extern uint8_t g_buf[48];	//4 rear left

extern MappingPar vel_RxPDO0;

extern MappingPar vel_TxPDO0;
extern MappingPar vel_TxPDO1;

extern uint32_t STM_FT_ID[4][2];

extern int16_t g_Tmp_cmd_FL;
extern int16_t g_Tmp_cmd_FR;
extern int16_t g_Tmp_cmd_RL;
extern int16_t g_Tmp_cmd_RR;

extern double g_angle_rad_c;
extern double g_angle_rad_i;
extern double g_angle_rad_o;

extern double g_Real_cmd_v_x;
extern double g_Real_cmd_v_y;
extern double g_Real_cmd_w;

extern double g_real_angle_c;
extern double g_real_angle_i;
extern double g_real_angle_o;


extern double g_Tar_cmd_v_x;
extern double g_Tar_cmd_v_i;
extern double g_Tar_cmd_v_o;
extern double g_Tar_cmd_v_y;
extern double g_Tar_cmd_w;

extern int16_t g_temp_x;
extern int16_t g_temp_y;
extern int16_t g_temp_w;

extern uint8_t g_torqueSW;

extern int8_t g_state_stop;
extern int8_t g_modeDflag;
extern uint32_t g_Stop_flag;

extern int8_t g_sendcanbuf[8];

extern int8_t g_canbuf[8];

extern uint32_t g_CanId;

extern int16_t g_Tar_cmd_FL;//Front Left
extern int16_t g_Tar_cmd_FR;//Front Right
extern int16_t g_Tar_cmd_RL;//Rear Left
extern int16_t g_Tar_cmd_RR;//Rear Right

extern uint8_t g_ModeABCD;//4 is stop mode
extern uint8_t g_Pre_ModeABCD;

extern uint32_t g_EndMode;
extern uint32_t g_timerflag;
extern uint32_t g_delaytime;//0.5s per 1

extern int16_t g_SteDeg[4];	//steering degree unit=0.01 degree

extern int32_t g_SAngle[4];



#endif /* INC_GLOVAL_H_ */

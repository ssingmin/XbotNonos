/*
 * app.c
 *
 *  Created on: Jun 3, 2023
 *      Author: ssingmin
 */
#include"app.h"

uint32_t g_tick_1ms=0;
uint32_t g_tick_100ms=0;
uint32_t g_tick_500ms=0;
uint32_t g_tick_1000ms=0;

uint8_t g_PS_SIGx_Pin = 0;	//0000 4321

uint8_t g_buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left


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

void app()
{

	uint8_t Dir_Rot = 0; //direction of rotation
	uint8_t FT_flag = 0; //FineTuning_flag

	HAL_Delay(1000);

	for(int i=0;i<4;i++){
		if(HAL_GPIO_ReadPin(GPIOA, ((1<<i)<<4))){//GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
			if((i==STMotorID2) || (i==STMotorID3)) 	{Dir_Rot = SERVO_CW;}
			else {Dir_Rot = SERVO_CCW; FT_flag |= (1<<i);}
		}
		else {
			if((i==STMotorID2) || (i==STMotorID3))	{Dir_Rot = SERVO_CCW;FT_flag |= (1<<i);}
			else {Dir_Rot = SERVO_CW;}
		}
		//DataSetSteering(g_buf, i, Dir_Rot, RPM_1, SERVO_INIT, INIT_SPEED);// i= STMotorIDx, x=1~4
		printf("PS_SIG1_Pin ccw init. %d %x\n", FT_flag, ((1<<i)<<4));
		ServoMotor_writeDMA(g_buf);//servo init. must done init within 500*20ms
	}

	while(1){

		uint32_t tick_ms = HAL_GetTick();

		if((tick_ms - g_tick_1000ms) >= 1000){//status led toogle 1
			g_tick_1000ms = tick_ms;

			printf("debug cycle %d\n", tick_ms);
			HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
		}

//		if((tick_ms - tick_100ms) >= 100){//can parsing, calculator xyw 2
//			tick_1000ms = tick_ms;
//			printf("1 cycle %d\n", tick_ms);
//
//		}
//
//		if((tick_ms - tick_1000ms) >= 1000){//operation steering motor 3
//			tick_1000ms = tick_ms;
//
//			printf("2 cycle %d\n", tick_ms);
//		}
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

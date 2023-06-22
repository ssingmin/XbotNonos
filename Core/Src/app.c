/*
 * app.c
 *
 *  Created on: Jun 3, 2023
 *      Author: ssingmin
 */
#include"app.h"

uint32_t temp = 0;

void app()
{
	temp = HAL_GetTick();
	if((temp%100) == 1){

		HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
		printf("hihi%d\n", temp);
		temp = 0;
	}

}

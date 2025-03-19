/*
 * callbacks.c
 *
 *  Created on: Feb 5, 2025
 *      Author: Usuario
 */


#include "main.h"
#include "motor_control.h"
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
		if(htim->Instance == TIM2 ){
			__disable_irq();
			uint8_t channel = 0;
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && float_W ) {
				channel = 1;
				zero_crossing(channel);
			}
			else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && float_U) {
				channel = 2;
				zero_crossing(channel);
			}
			else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && float_V) {
				channel = 3;
				zero_crossing(channel);
			}
			__enable_irq();
		}
		if(htim->Instance == TIM1){
			//pwm_input(htim);
		}


}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
	    //TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
		event_delay();


	}
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		__disable_irq();
		pi_control();
		__enable_irq();
	}
	if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		__disable_irq();
		update_pwm_startup_foc();
		__enable_irq();
	}


}

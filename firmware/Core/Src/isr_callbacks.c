/*
 * callbacks.c
 *
 *  Created on: Feb 5, 2025
 *      Author: Usuario
 */

#include "tim.h"
#include "motor_control.h"
#include "hard_config.h"
#include "zc_diag.h"

#pragma GCC optimize("Os")


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM2) {
    __disable_irq();
    if (app_state == FREEWHEEL) {
      zc_diag_handle_tim2_capture(htim);
    } else {
      uint8_t channel = 0;
      if (htim->Channel == zc_phase_to_active_channel(PHASE_W) && floating_W) {
        channel = 1;
        zero_crossing_handler(channel, HAL_TIM_ReadCapturedValue(htim, zc_phase_to_tim_channel(PHASE_W)));
      } else if (htim->Channel == zc_phase_to_active_channel(PHASE_U) && floating_U) {
        channel = 2;
        zero_crossing_handler(channel, HAL_TIM_ReadCapturedValue(htim, zc_phase_to_tim_channel(PHASE_U)));
      } else if (htim->Channel == zc_phase_to_active_channel(PHASE_V) && floating_V) {
        channel = 3;
        zero_crossing_handler(channel, HAL_TIM_ReadCapturedValue(htim, zc_phase_to_tim_channel(PHASE_V)));
      }
    }
    __enable_irq();
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 &&
      CLOSEDLOOP == app_state) {
    pi_control();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM4) {
    update_pwm_startup_foc();
  }
}

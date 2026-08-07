/*
 * app_state.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#include "state_machine.h"
#include "usart.h"

#pragma GCC optimize("Os")

#define RUNNING_RESTART_TIMEOUT_MS 4500U
#define RUNNING_RESTART_BRAKE_MS 500U

volatile App_States_t app_state = IDLE;

__attribute__((optimize("Oz"))) App_States_t handleState(void)
{

  static uint8_t  comm_initialized  = 0;
  static uint32_t last_comm_check   = 0;
  static uint8_t  flash_initialized = 0;
  static App_States_t last_state = IDLE;
  static uint32_t running_enter_time = 0;
  static uint32_t restart_brake_start_time = 0;
  static uint8_t  restart_pending = 0;
  uint32_t        current_time      = HAL_GetTick();

  if (!comm_initialized) {
    commInit();
    comm_initialized = 1;
  }

  if (!flash_initialized) {
    FlashResultCode result = flash_config_init();
    if (result == FLASH_RESULT_OK || result == FLASH_RESULT_EMPTY) {
      update_all_esc();
      updateAllMotorControl();
      flash_initialized = 1;
    } else {
      app_state         = HARD_ERROR;
      flash_initialized = 0;
    }
  }

  if (app_state != HARD_ERROR && (uint32_t)(current_time - last_comm_check) >= 5U) {
    processUartData();
    last_comm_check = current_time;
  }

  if (app_state != IDLE && app_state != HARD_ERROR) {
    process_logging_queue();
  }

  sine_drive_check_watchdog();

  if (cmd_received_ack && (app_state == IDLE || app_state == RUNNING || app_state == CLOSEDLOOP)) {
    if (process_speed_command()) {
      cmd_speed_received_ack = 0;
    } else {
      processCurrentCommand();
      handleCommandEffects();
    }
    cmd_received_ack = 0;
  }

  if (app_state != last_state) {
    if (app_state == RUNNING) {
      running_enter_time = current_time;
      restart_pending = 0;
    } else {
      if (app_state != CLOSEDLOOP) {
        restart_pending = 0;
      }
    }
    last_state = app_state;
  }

  if (restart_pending) {
    if (app_state != RUNNING && app_state != CLOSEDLOOP) {
      restart_pending = 0;
    } else if ((uint32_t)(current_time - restart_brake_start_time) >= RUNNING_RESTART_BRAKE_MS) {
      restart_pending = 0;
      motor_stalled = false;
      foc_startup();
    }
    return app_state;
  }

  switch (app_state) {
  case IDLE:
    break;

  case STARTUP:
    break;

  case FOC_STARTUP:
    break;
  case SINE_DRIVE:
    break;
  case RUNNING:
    if (consistent_zero_crossing) {
      app_state = READY;
      break;
    }
    if (motor_stalled ||
        (uint32_t)(current_time - running_enter_time) >= RUNNING_RESTART_TIMEOUT_MS) {
      PWM_STOP();
      bldc_set_pwm(0);
      restart_brake_start_time = current_time;
      restart_pending = 1;
      motor_stalled = false;
      break;
    }
    break;

  case CONFIG:
    if (!esc_config_done && !motor_control_config_done)
      update_all_esc();
    if (esc_config_done && !motor_control_config_done)
      updateAllMotorControl();
    if (esc_config_done && motor_control_config_done)
      app_state = IDLE;
    break;
  case CLOSEDLOOP:
    if (hil_session_state != (uint8_t)HIL_EXECUTION_STEPPED + 1U) {
      HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
    }
    current_time = HAL_GetTick();
    if (hil_session_state == 0U && motor_stalled) {
      HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
      PWM_STOP();
      bldc_set_pwm(0);
      restart_brake_start_time = current_time;
      restart_pending = 1;
      motor_stalled = false;
      break;
    }
    if (hil_session_state == 0U && 0 == consistent_zero_crossing)
      app_state = RUNNING;
    break;
  case READY:
    TIM4->PSC = MOTOR_CONTROL_TIMER_PSC;
    TIM4->ARR = MOTOR_CONTROL_TIMER_ARR;
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_CONTROL_TIMER_COMPARE);
    motor_control_prepare_closed_loop();
    app_state = CLOSEDLOOP;

    break;
  case STOPPED:
    HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);

    break;

  case FINISH:
    // Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    app_state = IDLE; // Vuelve al estado IDLE

    break;

  case HARD_ERROR:
    Error_Handler();
    break;
  }

  return (app_state);
}

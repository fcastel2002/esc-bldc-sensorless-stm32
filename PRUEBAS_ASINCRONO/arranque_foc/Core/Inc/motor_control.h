/*
 * basic_func.h
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include "main.h"
#include "hard_config.h"

// macros
#define PWM_STOP() do { \
   	__HAL_TIM_SET_COMPARE(&htim1, EN_U, 0);\
	__HAL_TIM_SET_COMPARE(&htim1, EN_V, 0);\
	__HAL_TIM_SET_COMPARE(&htim1, EN_W, 0);\
} while (0)


#define PWM_INIT() do { \
	HAL_TIM_PWM_Start(&htim1, EN_U);\
	HAL_TIM_PWM_Start(&htim1,EN_V);\
	HAL_TIM_PWM_Start(&htim1, EN_W);\
} while (0)

//RUNTIME VARIABLES
static volatile uint32_t last_zc_timestamp = 0 ;
static volatile bool motor_stalled = false;
static volatile uint16_t max_limit_pwm;
static volatile uint16_t min_limit_pwm;
static volatile float pwm_speed_range_relation;
extern volatile uint16_t max_pwm;
extern volatile uint16_t speed_setpoint;
extern volatile uint16_t speed_command;
extern volatile uint16_t pwmVal;
extern volatile uint16_t commutationTime;
extern volatile int8_t commutationStep ;
extern bool float_W ;
extern bool float_U ;
extern bool float_V;
extern volatile uint8_t motor_control_config_done;
extern volatile uint8_t consistent_zero_crossing;
extern volatile uint16_t reference_comm;
extern volatile uint16_t reference_zcp;

extern volatile uint16_t current_cnt ;

extern volatile uint8_t direction;
// RUNTIME FUNCTIONS
void alignment(void);
void startup(void);
extern void update_all_motor_control(void);
void pi_control(void);
extern void motor_detection(void);
void commutation(int8_t step);
void zero_crossing(uint8_t fase);
void pwm_input(TIM_HandleTypeDef *htim);
extern void check_motor_status(void);


#endif /* INC_MOTOR_CONTROL_H_ */

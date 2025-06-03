/*
 * motor_control.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */


#include "main.h"
#include <stdlib.h>
#include "motor_control.h"
#define KP 0.75f
#define KI 1.05f //real Ki = KI * 2/SCALE
#define SCALE 1
#define dt 0.002


/*
 *@brief: Macros para la medicion y mapeo de velocidad
 * unidades: PERIODO.
 * TIMER CLOCK: 72MHz
 * PSC: 400 (399 + 1)
 * ARR: 0xFFFF
 *
 * VELOCIDAD MINIMA RPM: 380
 * VELOCIDAD MAXIMA RPM: 12000
 *
 */

#define SPEED_MAX 920	//pwm
#define SPEED_MIN 17000
#define SPEED_RANGE (SPEED_MIN - SPEED_MAX)
#define OPEN_LOOP_FINAL_SPEED 3000
////////////

// limites PWM


//STALL HANDLING
#define TIMEOUT_MOTOR_STALL_MS 200
#define STALL_CHECK_TIME_MS 25
////////

#define  ZCP_TO_CHECK 4
#define SPEED_TOLERANCE_PCT 60

volatile uint16_t speed_command = 0;
volatile uint16_t pwmVal = 0;
volatile int8_t commutationStep = 0;
bool float_W = false;
bool float_U = false;
bool float_V = false;

static volatile uint16_t pwm_freq_input = 0;

volatile uint16_t speed_setpoint =  0;

static volatile int32_t speed_prev_error = 0;

//control pi variables
static volatile uint16_t max_limit_pwm = 0;
static volatile uint16_t min_limit_pwm = 0;
static volatile float pwm_speed_range_relation = 0.0f;
volatile uint8_t motor_control_config_done = 0;
volatile uint8_t consistent_zero_crossing = 0;
volatile uint16_t max_pwm = 0;
volatile bool motor_stalled = false;

static volatile uint16_t speed_measure;
static volatile int32_t speed_error;
static volatile int32_t speed_output = 0;
static volatile float speed_integral = 0;

static volatile float speed_proportional;
static volatile float max_speed_integral = 0;
static volatile float min_speed_integral = 0;
//zero crossing algorithm variables
volatile int32_t diff_speed = 0;
volatile uint8_t direction = 0;


static uint16_t last_W_timestamp = 0;
static uint16_t W_periods[ZCP_TO_CHECK];
static uint8_t W_period_idx = 0;
static uint8_t valid_W_zcp = 0;

// speed measure digital filter variables
static volatile uint8_t speed_buffer_size = 1;
static volatile uint16_t speed_buffer[5] = {0};
static volatile uint16_t filtered_speed = 0;
static volatile uint16_t last_speed_capture = 1;
int8_t safe_mod(int8_t value, int8_t mod) {
    return (value % mod + mod) % mod;
}

void commutation(int8_t step) {


	switch(step) {

	case POS_UV:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		float_W = true;
		float_V = false;
		float_U = false;

		GPIOA->ODR &= ~EN_W;
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;
	case POS_UW:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		float_W = false;
		float_U = false;
		float_V = true;
		GPIOB->ODR &= ~EN_V;
		GPIOB->ODR |= EN_U;
		GPIOA->ODR |=  EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
		break;
	case POS_VW:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

		float_W = false;
		float_V = false;
		float_U = true;
		GPIOB->ODR &= ~EN_U;

		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);

		break;
	case POS_VU:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

		float_W = true;
		float_V = false;
		float_U = false;
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |=  EN_V;
		GPIOA->ODR &= ~EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
		break;
	case POS_WU:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);

		float_W = false;
		float_U = false;
		float_V = true;
		GPIOB->ODR &= ~EN_V;
		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_U;
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
		break;
	case POS_WV:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

		float_W = false;
		float_V = false;
		float_U = true;
		GPIOB->ODR &= ~EN_U;
		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;

	case POS_INIT:
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		GPIOA->ODR |= EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;
	case POS_SOUND:
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		GPIOA->ODR |= EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		break;
	default:
		break;
	}
}



void update_all_motor_control(){
	max_limit_pwm = TIM1 -> ARR;
	max_pwm = max_limit_pwm;
	min_limit_pwm = max_limit_pwm * 0.05f;
	pwm_speed_range_relation = (float)(max_limit_pwm - min_limit_pwm)/(float)SPEED_RANGE;
	speed_setpoint = max_limit_pwm *0.8f;
	motor_control_config_done = 1;


}
uint16_t filtro_media_movil(uint16_t measurement){
	int32_t new_speed = 0;
	static uint16_t prev_speed = 0;
	static volatile uint8_t speed_index = 0;
	if(measurement < 1700 && prev_speed > 1700 && measurement < 12000 && prev_speed < 12000){
		speed_buffer[speed_index] = prev_speed;
	}
	else{
		speed_buffer[speed_index] = measurement;
		prev_speed = measurement;
	}
	speed_index  = (speed_index + 1) % speed_buffer_size;

	for (int i = 0; i < speed_buffer_size; i++) {
		new_speed += speed_buffer[i];
	}
	new_speed = new_speed/speed_buffer_size;


	return (uint16_t)new_speed;
}

static inline void check_zero_crossing_consistency(uint16_t current_timestamp) {
	if(last_W_timestamp != 0){
		uint16_t period;
		if(last_W_timestamp > current_timestamp){
		    // Solo si es estrictamente mayor, ha ocurrido un desbordamiento
		    period = (0xFFFF - last_W_timestamp) + current_timestamp + 1; // +1 porque el contador incluye el 0
		}
		else{
		    period = current_timestamp - last_W_timestamp;
		}

		W_periods[W_period_idx] = period;

		W_period_idx = (W_period_idx + 1) % ZCP_TO_CHECK;

		if(valid_W_zcp < ZCP_TO_CHECK){
			valid_W_zcp++;

		}
		if(valid_W_zcp == ZCP_TO_CHECK){

			uint16_t avg_period = 0;
			uint8_t is_consistent = 1;
			for(uint8_t i = 0; i< ZCP_TO_CHECK;i++){
				avg_period += W_periods[i];
			}
			avg_period /= ZCP_TO_CHECK;
			if(avg_period > 20){
				for(uint8_t i = 0; i < ZCP_TO_CHECK; i++){
					uint32_t tolerance = avg_period * SPEED_TOLERANCE_PCT / 100;
					if(abs(W_periods[i] - avg_period)> tolerance){
						is_consistent = 0;
						break;
					}
				}
				if(is_consistent){

					valid_W_zcp =0;
					consistent_zero_crossing = 1;
					speed_buffer_size = 1;

				}
				else{

					consistent_zero_crossing = 0;

				}
			}
			valid_W_zcp = 0;
		}
	}
	last_W_timestamp = current_timestamp;
}

void zero_crossing(uint8_t fase){

	switch(fase){
	case 1:
		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if (direction == 0) {
				commutationStep = safe_mod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safe_mod(commutationStep - 1, NUM_POS);
			}
			last_zc_timestamp = HAL_GetTick();

			commutation(commutationStep);
		}
		if(app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP){
			uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			check_zero_crossing_consistency(current_timestamp);
		}
		if((commutationStep == POS_UW || commutationStep == POS_WV) && app_state == CLOSEDLOOP){
		    uint16_t current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		    // Calcular diff_speed basado en la diferencia desde la última captura
		    if(last_speed_capture != 0) {
		        uint16_t speed_period;
		        if(last_speed_capture > current_capture) {
		            speed_period = (0xFFFF - last_speed_capture) + current_capture + 1;
		        } else {
		            speed_period = current_capture - last_speed_capture;
		        }
		        diff_speed = speed_period;
		        filtered_speed = filtro_media_movil(diff_speed);
		    }
		    last_speed_capture = current_capture;

		    // No reiniciar el timer
		}

		break;
	case 2:

		if(app_state == RUNNING || app_state == CLOSEDLOOP){

			if (direction == 0) {
				commutationStep = safe_mod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safe_mod(commutationStep - 1, NUM_POS);
			}
					commutation(commutationStep);
		}
		break;
	case 3:

		if(app_state == RUNNING || app_state == CLOSEDLOOP){

			if (direction == 0) {
				commutationStep = safe_mod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safe_mod(commutationStep - 1, NUM_POS);
			}
					commutation(commutationStep);
		}
		break;
	}

}

void motor_detection(){
	TIM1 -> PSC = 7;
	uint16_t arr_pwm = 0;
	arr_pwm = 1000;
	TIM1 -> ARR = arr_pwm;
	pwmVal = arr_pwm * 0.3f;
	uint8_t step = POS_UV;
	for(int k = 0; k <3; k++){
		PWM_STOP();
		HAL_Delay(60);
		PWM_INIT();
		for (int i = 0; i < 140; i++){
			commutation(step);
			step = safe_mod(step + 1, NUM_POS);

			HAL_Delay(1);
		}

	}
	PWM_STOP();

	pwmVal = 0;
}

void check_motor_status(){
	static uint32_t last_check_time = 0;
	static uint8_t stall_counter = 0;
	static uint8_t running_counter = 0;

	uint32_t current_time = HAL_GetTick();
	if(current_time - last_check_time >= STALL_CHECK_TIME_MS){
		last_check_time = current_time;

		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if(last_zc_timestamp > 0 && ((current_time - last_zc_timestamp)> TIMEOUT_MOTOR_STALL_MS)){
				stall_counter ++;
				running_counter = 0;
				if(stall_counter >= 3){
					motor_stalled = true;
				}

			}else{
				running_counter++;
				stall_counter = 0;
				if(running_counter >= 2 && motor_stalled){
					motor_stalled = false;
					running_counter = 0;
				}
			}
		}else{
			running_counter = 0;
			stall_counter = 0;

		}
	}
}

static inline uint16_t map_speed(uint16_t raw_speed){
	if(raw_speed == 0){
		raw_speed = SPEED_MIN;
	}


	if (raw_speed > SPEED_MIN) {
		raw_speed = SPEED_MIN;
	}
	else if (raw_speed <SPEED_MAX) {
		raw_speed = SPEED_MAX;
	}
	uint16_t mapped_speed  = (SPEED_MIN - raw_speed) * pwm_speed_range_relation + min_limit_pwm;
	return mapped_speed;

}

void pi_control(){
	__disable_irq();
	speed_measure =  map_speed(filtered_speed);
	if(app_state == CLOSEDLOOP){
		speed_error = speed_setpoint - speed_measure;
		speed_proportional = (KP * speed_error)/SCALE;
		speed_integral += KI * (speed_error+speed_prev_error)*dt;
		speed_prev_error = speed_error;
		if(max_limit_pwm > speed_proportional){
			max_speed_integral = (max_limit_pwm - speed_proportional);
		}
		else{
			max_speed_integral = 0;
		}
		if (min_limit_pwm < speed_proportional) {
			min_speed_integral = (min_limit_pwm - speed_proportional);
		} else {
			min_speed_integral = 0;
		}
		if (speed_integral > max_speed_integral) {
			speed_integral = max_speed_integral;
		}
		else if (speed_integral < min_speed_integral) {
			speed_integral = min_speed_integral;
		}
		speed_output = speed_proportional + speed_integral;
		if(speed_output < min_limit_pwm) speed_output = min_limit_pwm;
		if(speed_output > max_limit_pwm) speed_output = max_limit_pwm;
		pwmVal = (uint16_t)speed_output ;
	}
	__enable_irq();
}

static inline void pwm_break(void){
	pwmVal = 0;
	GPIOB->ODR 	&= ~EN_U;
	GPIOB->ODR |= EN_V;
	GPIOA->ODR |= EN_W;
	__HAL_TIM_SET_COMPARE(&htim1, IN_U, max_pwm);
	__HAL_TIM_SET_COMPARE(&htim1, IN_V, max_pwm);
	__HAL_TIM_SET_COMPARE(&htim1, IN_W, max_pwm);

}
void stop_motor(uint8_t mode){

	switch(mode){
		case 0:
		//gradual stop
			PWM_STOP();
			app_state = IDLE;
			break;
		case 1:
		//emergency stop
			app_state = STOPPED;

			direction = !direction;
			pwmVal = max_pwm;
			uint32_t stop_start_time = HAL_GetTick();
			commutation(POS_UV);
			while (HAL_GetTick() - stop_start_time < 5000);
			PWM_STOP();
			direction = !direction;	
			pwmVal = 0;
			app_state = IDLE;
			break;
	}
}

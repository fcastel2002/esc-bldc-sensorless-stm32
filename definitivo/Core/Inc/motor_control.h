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
   	__HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);\
	__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);\
	__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);\
} while (0)


#define PWM_INIT() do { \
	HAL_TIM_PWM_Start(&htim1, IN_U);\
	HAL_TIM_PWM_Start(&htim1,IN_V);\
	HAL_TIM_PWM_Start(&htim1, IN_W);\
} while (0)

//RUNTIME VARIABLES
static volatile uint32_t last_zc_timestamp = 0 ;
extern volatile bool motor_stalled;
static volatile uint16_t max_limit_pwm;
static volatile uint16_t min_limit_pwm;
static volatile float pwm_speed_range_relation;
extern volatile uint16_t max_pwm;
extern volatile uint16_t speed_setpoint;
extern volatile uint16_t speed_setpoint_rpm;
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
extern volatile int32_t diff_speed;
extern volatile uint8_t direction;
extern volatile uint16_t filtered_speed;
// RUNTIME FUNCTIONS
extern void updateAllMotorControl(void);
void PIcontrol(void);
extern void motorDetection(void);
//void commutation(int8_t step);
extern void zeroCrossing(uint8_t fase);
extern uint16_t convertSpeedValue(uint16_t value, bool to_ticks);
extern void checkMotorStatus(void);
extern void stopMotor(uint8_t mode);

extern uint16_t periodToPwm(uint16_t period);
extern uint16_t periodToRpm(uint16_t period);
extern uint16_t rpmToPeriod(uint16_t rpm);

extern uint16_t getActualSpeed(void);
// MACROS PARA MEDICION DE VELOCIDAD 
#define PHASE_COUNT 3
#define ZCP_BUFFER_SIZE 4
#define SPEED_CONSENSUS_THRESHOLD 15  // % de tolerancia entre fases

// Estructura para cada fase
typedef struct {
    uint16_t last_timestamp;
    uint16_t periods[ZCP_BUFFER_SIZE];
    uint8_t period_idx;
    uint8_t valid_periods;
    uint16_t avg_period;
    uint8_t is_consistent;
} phase_measurement_t;

// Variables externas para medición tri-fase
extern phase_measurement_t phase_measurements[PHASE_COUNT];
extern volatile uint16_t consensus_speed;
extern volatile uint8_t active_phases_count;
extern volatile uint8_t speed_measurement_ready;

// Funciones para medición tri-fase
void reset_phase_measurements(void);
void get_phase_diagnostics(uint16_t* phase_speeds, uint8_t* phase_status);

#endif /* INC_MOTOR_CONTROL_H_ */

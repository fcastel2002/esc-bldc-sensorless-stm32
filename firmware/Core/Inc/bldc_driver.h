#ifndef INC_BLDC_DRIVER_H_
#define INC_BLDC_DRIVER_H_
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "basic_config.h"
#include "tim.h"
// macros
#define PWM_STOP()                                                                                 \
  do {                                                                                             \
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);                                                        \
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);                                                        \
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);                                                        \
  } while (0)

#define PWM_INIT()                                                                                 \
  do {                                                                                             \
    HAL_TIM_PWM_Start(&htim1, IN_U);                                                               \
    HAL_TIM_PWM_Start(&htim1, IN_V);                                                               \
    HAL_TIM_PWM_Start(&htim1, IN_W);                                                               \
  } while (0)


//api 
extern bool floating_W;
extern bool floating_U;
extern bool floating_V;

extern void bldc_init(void);
extern void bldc_set_pwm(uint16_t new_pwm);
extern void bldc_commutate();
extern void bldc_stop(void);
extern void bldc_disable_power_stage(void);
extern uint16_t bldc_get_pwm(void);
extern int8_t bldc_get_commutation_step(void);
#endif /* INC_BLDC_DRIVER_H_ */

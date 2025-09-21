#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f1xx_hal.h"
extern int8_t safe_mod(int8_t value, int8_t mod);
extern uint16_t convert_speed_ticks(uint16_t value, bool to_ticks);


#endif /* INC_UTILITIES_H_ */
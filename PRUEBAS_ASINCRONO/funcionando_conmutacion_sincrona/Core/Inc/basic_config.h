/*
 * basic_config.h
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */

#ifndef INC_BASIC_CONFIG_H_
#define INC_BASIC_CONFIG_H_


#define IN_U TIM_CHANNEL_1
#define IN_V TIM_CHANNEL_2
#define IN_W TIM_CHANNEL_3
#define EN_U (1U << 13)  // pa4
#define EN_V (1U << 1)   // PB1
#define EN_W (1U << 5)  // PA5

// PWM CONFIG //
#define ARR_TIM3 3000   // FREQ PWM 1799 -> 20kHz
//////////////////////


// STARTUP CONFIG//
// timer4 base ARR 19999 -> 2000 ms
#define STARTUP_TIME_BASE 9999 // 1s this has to be reached several times

#define STARTUP_TIME_X 10 // this value multiplies STARTUP_TIME_BASE
#define DC_STARTUP_INIT 800
#define DC_STARTUP_END 920
#define DC_STARTUP_STEP 20
#define TIME_STARTUP_COMMUTATION_INIT 100
#define TIME_STARTUP_COMMUTATION_FINAL 18
#define TIME_STARTUP_STEP 2// commutation time step time 10ms
// POSICIONES ELECTRICAS DEL ROTOR 6 PASOS REPRESENTAN MEDIA VUELTA //
#define PHASE_U 0
#define PHASE_V 1
#define PHASE_W 2

#define POS_UV 0
#define POS_UW 1
#define POS_VW 2
#define POS_VU 3
#define POS_WU 4
#define POS_WV 5
#define NUM_POS 6

#define POS_INIT 7
#define POS_SOUND 8
////////////////////////////////////////////////////////////


#endif /* INC_BASIC_CONFIG_H_ */

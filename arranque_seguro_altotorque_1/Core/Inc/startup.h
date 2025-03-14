/*
 * startup.h
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */

#ifndef INC_STARTUP_H_
#define INC_STARTUP_H_
#include "main.h"
#include "motor_control.h"

typedef struct {
    uint16_t phase_counter;     // Posición actual en la tabla
    uint8_t phase_step;         // Incremento de posición (velocidad)
    float modulation_index;     // Índice de modulación (0.0-1.0)
    uint16_t rotation_count;    // Vueltas completadas
    uint16_t timer_arr;         // Período del timer para actualización
} SineDriveController;
extern void alignment(void);
extern void startup(void);

extern const uint16_t sineLookupTable[100];

/// ALIGN CONFIG
#define ALIGN_TIME 15000 // 150ms per step
#define DC_ALIGN 1400	// DDUTY CYCLE 1300 -> 72% PARA ALIGNMENT
///
#define PREPOSITIONING_RAMP_TABLE_SIZE 10
static const uint16_t prepositioningRamp_table[PREPOSITIONING_RAMP_TABLE_SIZE] =  {150,300,450,600,750,900,1050,1200,1300,1500};
extern volatile uint16_t zero_crossings;

// FOC STARTUP DEFINES
#define SIN_TABLE_SIZE 360
extern void generate_sine_tables(uint16_t max_pwm);
extern void update_pwm_startup_foc();
extern void foc_startup(void);

void executeTransition(void);

extern uint16_t sin_table_U[SIN_TABLE_SIZE];
extern uint16_t sin_table_V[SIN_TABLE_SIZE];
extern uint16_t sin_table_W[SIN_TABLE_SIZE];
extern bool ready_for_update_pwm;
extern bool finished_foc_startup;
#endif /* INC_STARTUP_H_ */

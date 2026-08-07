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
  uint32_t frequency_millihz;
  uint16_t amplitude_permille;
} SineDriveSettings;

extern const uint16_t sineLookupTable[100];

/// ALIGN CONFIG
#define ALIGN_TIME 15000 // 150ms per step
#define DC_ALIGN 1400    // DDUTY CYCLE 1300 -> 72% PARA ALIGNMENT

extern volatile uint16_t zero_crossings;

// FOC STARTUP DEFINES
#define SIN_TABLE_SIZE 359
#define SINE_DRIVE_WATCHDOG_MS 1500U
extern void update_pwm_startup_foc(void);
extern void foc_startup(void);
extern bool sine_drive_start_or_update(uint32_t frequency_millihz,
                                       uint16_t amplitude_permille,
                                       SineDriveSettings* applied);
extern void sine_drive_stop(void);
extern void sine_drive_check_watchdog(void);
extern bool sine_drive_is_active(void);

extern uint16_t sin_table_U[SIN_TABLE_SIZE];
extern uint16_t sin_table_V[SIN_TABLE_SIZE];
extern uint16_t sin_table_W[SIN_TABLE_SIZE];
extern bool     ready_for_update_pwm;
extern bool     finished_foc_startup;
#endif /* INC_STARTUP_H_ */

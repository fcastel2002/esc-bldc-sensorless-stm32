/*
 * power_monitor.h
 *
 * INA219 DC-bus monitor on I2C1 (PB6 = SCL, PB7 = SDA).
 *
 * PB6/PB7 are shared with USART1 (remapped) used by the UART transport, so the
 * monitor is only active when the board boots in USB transport mode (PB8 HIGH).
 */

#ifndef POWER_MONITOR_H_
#define POWER_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

/** Shunt resistor fitted on the INA219 module (R100 = 0.1 ohm). */
#define POWER_MONITOR_SHUNT_MILLIOHM 100U
/** Default 7-bit INA219 address (A0 = A1 = GND). Other 0x40..0x4F are probed. */
#define POWER_MONITOR_I2C_ADDR_DEFAULT 0x40U
/** Sample period of the main-loop poll. */
#define POWER_MONITOR_POLL_MS 20U
/** Retry period while the sensor is absent or the bus failed. */
#define POWER_MONITOR_RETRY_MS 1000U

/* Flags reported in GET_STATUS and telemetry (bitfield). */
#define POWER_FLAG_PRESENT 0x01U  /**< INA219 answered on the bus. */
#define POWER_FLAG_VALID 0x02U    /**< Last reading succeeded. */
#define POWER_FLAG_OVERFLOW 0x04U /**< INA219 math overflow (shunt out of PGA range). */
#define POWER_FLAG_DISABLED 0x08U /**< UART transport owns PB6/PB7; monitor disabled. */

typedef struct {
  uint16_t bus_mv;      /**< Bus voltage at VIN- relative to GND, mV. */
  int32_t current_ua;   /**< DC-bus current through the shunt, microamps. */
  int16_t shunt_raw;    /**< Raw shunt-voltage register (10 uV/LSB). */
  uint8_t flags;        /**< POWER_FLAG_* bitfield. */
  uint8_t i2c_addr;     /**< Detected 7-bit address, 0 when absent. */
  uint32_t sample_tick; /**< HAL tick of the last successful reading. */
} PowerMonitorSnapshot;

/** Configure I2C1 and probe the INA219. Call once, after the transport is chosen. */
void power_monitor_init(bool enabled);
/** Non-blocking-ish poll from the main loop (one short I2C transaction burst). */
void power_monitor_process(void);
/** Copy the latest reading. */
void power_monitor_get_snapshot(PowerMonitorSnapshot* out);

#endif /* POWER_MONITOR_H_ */

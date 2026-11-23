/*
 * power_monitor.h
 *
 * INA219 DC-bus monitor on I2C1 (PB6 = SCL, PB7 = SDA) using the ST HAL I2C driver.
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

/* Last error reported in GET_STATUS byte 18. */
#define POWER_ERR_NONE 0U
#define POWER_ERR_NOT_FOUND 1U /**< No ACK at 0x40..0x4F (wiring, pull-ups, power). */
#define POWER_ERR_NACK 2U      /**< Acknowledge failure during a transfer. */
#define POWER_ERR_TIMEOUT 3U   /**< HAL timeout (SCL held low / no response). */
#define POWER_ERR_BUS 4U       /**< Bus error or arbitration lost (noise). */
#define POWER_ERR_BUSY 5U      /**< Bus stuck busy (SDA held low). */
#define POWER_ERR_OTHER 6U

/* Stage of the last failure, GET_STATUS byte 21. */
#define POWER_STAGE_NONE 0U
#define POWER_STAGE_PROBE 1U        /**< No address ACK in 0x40..0x4F. */
#define POWER_STAGE_CONFIG_WRITE 2U /**< Writing the configuration register. */
#define POWER_STAGE_CONFIG_VERIFY 3U /**< Config read-back failed or mismatched. */
#define POWER_STAGE_READ_SHUNT 4U
#define POWER_STAGE_READ_BUS 5U

/* Idle line levels sampled as floating inputs before each probe, byte 20. */
#define POWER_LINE_SCL_HIGH 0x01U
#define POWER_LINE_SDA_HIGH 0x02U

typedef struct {
  uint8_t last_error;   /**< POWER_ERR_* of the last failed transaction. */
  uint8_t error_count;  /**< Saturating count of failed transactions. */
  uint8_t line_state;   /**< POWER_LINE_* idle levels (pull-ups present => 0x03). */
  uint8_t fail_stage;   /**< POWER_STAGE_* of the last failure. */
  uint16_t config_readback; /**< Config register read back after the probe write. */
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

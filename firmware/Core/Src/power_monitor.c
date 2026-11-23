/*
 * power_monitor.c
 *
 * INA219 DC-bus voltage/current monitor on PB6 = SCL, PB7 = SDA.
 *
 * Why a software (bit-banged) I2C master instead of the I2C1 peripheral:
 * PB6/PB7 are also the remapped USART1 pins. On the STM32F1 the alternate-
 * function outputs of the peripherals mapped to a pin interfere, and the
 * hardware I2C needs the pins in alternate-function mode. Verified on the
 * bench:
 *  - with USART1_REMAP cleared, USART1 falls back to PA8/PA9/PA10 (the TIM1
 *    phase PWM pins) and corrupts the power-stage drive (ZC auto-detect broke);
 *  - with USART1_REMAP kept, hardware I2C reads on PB6/PB7 fail; with it
 *    cleared the I2C peripheral also latched BUSY after every read.
 * In general-purpose output mode a pin is driven only by ODR, so no other
 * peripheral can interfere. USART1_REMAP therefore stays exactly as main()
 * configures it, and the bus is driven here through open-drain GPIOs.
 *
 * The master runs at ~100 kHz and supports clock stretching. One poll (two
 * register reads) blocks the main loop for roughly 1 ms; interrupts only
 * stretch individual bits, which is harmless because the master owns SCL.
 *
 * The INA219 runs in continuous shunt+bus mode with 16-sample averaging
 * (~8.5 ms per channel), PGA /8 (+/-320 mV shunt => +/-3.2 A with 0.1 ohm) and
 * the 32 V bus range. Current is computed from the shunt-voltage register
 * (10 uV/LSB), so the calibration register is not needed.
 */

#include "power_monitor.h"

#include "main.h"
#include "usart.h"

#include <string.h>

#define INA219_REG_CONFIG 0x00U
#define INA219_REG_SHUNT 0x01U
#define INA219_REG_BUS 0x02U

/* BRNG=32V | PGA=/8 | BADC=16 avg | SADC=16 avg | MODE=shunt+bus continuous */
#define INA219_CONFIG_VALUE 0x3E67U
#define INA219_SHUNT_UV_PER_LSB 10
#define INA219_BUS_MV_PER_LSB 4U
#define INA219_BUS_OVF 0x0001U

#define POWER_MONITOR_MAX_FAILURES 5U

#define I2C_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_HALF_PERIOD_US 5U /* ~100 kHz */
#define I2C_STRETCH_TIMEOUT_US 2000U

static PowerMonitorSnapshot snapshot;
static bool monitor_enabled = false;
static uint8_t failure_count = 0;
static uint32_t last_poll_tick = 0;
static uint32_t last_retry_tick = 0;
static uint32_t cycles_per_us = 72U;

/* ------------------------------------------------------- bit-bang I2C ---- */

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * cycles_per_us;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
  }
}

static inline void scl_low(void) { I2C_PORT->BRR = I2C_SCL_PIN; }
static inline void sda_low(void) { I2C_PORT->BRR = I2C_SDA_PIN; }
static inline void sda_release(void) { I2C_PORT->BSRR = I2C_SDA_PIN; }
static inline bool sda_read(void) { return (I2C_PORT->IDR & I2C_SDA_PIN) != 0U; }
static inline bool scl_read(void) { return (I2C_PORT->IDR & I2C_SCL_PIN) != 0U; }

/* Release SCL and wait for it to go high (slave clock stretching). */
static bool scl_release(void)
{
  I2C_PORT->BSRR = I2C_SCL_PIN;
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout = I2C_STRETCH_TIMEOUT_US * cycles_per_us;
  while (!scl_read()) {
    if ((uint32_t)(DWT->CYCCNT - start) > timeout) {
      return false;
    }
  }
  return true;
}

/* Sample idle SCL/SDA as floating inputs: both must read high when the
 * module's pull-ups are powered. */
static void sample_idle_lines(void)
{
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C_PORT, &gpio);
  delay_us(20U);

  uint8_t state = 0U;
  if (scl_read()) state |= POWER_LINE_SCL_HIGH;
  if (sda_read()) state |= POWER_LINE_SDA_HIGH;
  snapshot.line_state = state;
}

static void i2c_gpio_init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  if (cycles_per_us == 0U) {
    cycles_per_us = 1U;
  }

  sample_idle_lines();

  I2C_PORT->BSRR = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD; /* GPIO mode: no alternate function reaches the pin */
  gpio.Pull = GPIO_NOPULL;         /* the INA219 module provides the pull-ups */
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_PORT, &gpio);
}

static void i2c_stop(void)
{
  sda_low();
  delay_us(I2C_HALF_PERIOD_US);
  (void)scl_release();
  delay_us(I2C_HALF_PERIOD_US);
  sda_release();
  delay_us(I2C_HALF_PERIOD_US);
}

/* Clock out a slave stuck mid-byte, then STOP. */
static void i2c_bus_recover(void)
{
  sda_release();
  for (uint8_t i = 0; i < 9U && !sda_read(); i++) {
    scl_low();
    delay_us(I2C_HALF_PERIOD_US);
    (void)scl_release();
    delay_us(I2C_HALF_PERIOD_US);
  }
  scl_low();
  delay_us(I2C_HALF_PERIOD_US);
  i2c_stop();
}

static uint8_t i2c_start(void)
{
  sda_release();
  if (!scl_release()) {
    return POWER_ERR_TIMEOUT;
  }
  delay_us(I2C_HALF_PERIOD_US);
  if (!sda_read()) {
    return POWER_ERR_BUSY;
  }
  sda_low();
  delay_us(I2C_HALF_PERIOD_US);
  scl_low();
  delay_us(I2C_HALF_PERIOD_US);
  return POWER_ERR_NONE;
}

/* POWER_ERR_NONE on ACK, POWER_ERR_NACK on NACK, POWER_ERR_TIMEOUT if SCL is stuck low. */
static uint8_t i2c_write_byte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8U; i++) {
    if ((byte & 0x80U) != 0U) {
      sda_release();
    } else {
      sda_low();
    }
    byte = (uint8_t)(byte << 1);
    delay_us(I2C_HALF_PERIOD_US);
    if (!scl_release()) {
      return POWER_ERR_TIMEOUT;
    }
    delay_us(I2C_HALF_PERIOD_US);
    scl_low();
  }

  sda_release();
  delay_us(I2C_HALF_PERIOD_US);
  if (!scl_release()) {
    return POWER_ERR_TIMEOUT;
  }
  delay_us(I2C_HALF_PERIOD_US);
  bool ack = !sda_read();
  scl_low();
  delay_us(I2C_HALF_PERIOD_US);
  return ack ? POWER_ERR_NONE : POWER_ERR_NACK;
}

static uint8_t i2c_read_byte(bool ack, uint8_t* out)
{
  uint8_t byte = 0U;
  sda_release();
  for (uint8_t i = 0; i < 8U; i++) {
    delay_us(I2C_HALF_PERIOD_US);
    if (!scl_release()) {
      return POWER_ERR_TIMEOUT;
    }
    delay_us(I2C_HALF_PERIOD_US);
    byte = (uint8_t)((byte << 1) | (sda_read() ? 1U : 0U));
    scl_low();
  }

  if (ack) {
    sda_low();
  } else {
    sda_release();
  }
  delay_us(I2C_HALF_PERIOD_US);
  if (!scl_release()) {
    return POWER_ERR_TIMEOUT;
  }
  delay_us(I2C_HALF_PERIOD_US);
  scl_low();
  sda_release();
  delay_us(I2C_HALF_PERIOD_US);

  *out = byte;
  return POWER_ERR_NONE;
}

/* START + address byte; on failure the bus is released with a STOP. */
static uint8_t i2c_begin(uint8_t addr_byte)
{
  uint8_t err = i2c_start();
  if (err == POWER_ERR_NONE) {
    err = i2c_write_byte(addr_byte);
  }
  if (err != POWER_ERR_NONE) {
    i2c_stop();
  }
  return err;
}

static uint8_t ina219_write(uint8_t addr, uint8_t reg, uint16_t value)
{
  uint8_t err = i2c_begin((uint8_t)(addr << 1));
  if (err != POWER_ERR_NONE) {
    return err;
  }
  const uint8_t bytes[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFFU) };
  for (uint8_t i = 0; i < 3U && err == POWER_ERR_NONE; i++) {
    err = i2c_write_byte(bytes[i]);
  }
  i2c_stop();
  return err;
}

static uint8_t ina219_read(uint8_t addr, uint8_t reg, uint16_t* value)
{
  /* Pointer write, then repeated START for the 2-byte read. */
  uint8_t err = i2c_begin((uint8_t)(addr << 1));
  if (err != POWER_ERR_NONE) {
    return err;
  }
  err = i2c_write_byte(reg);
  if (err != POWER_ERR_NONE) {
    i2c_stop();
    return err;
  }

  err = i2c_begin((uint8_t)((addr << 1) | 1U));
  if (err != POWER_ERR_NONE) {
    return err;
  }

  uint8_t msb = 0U;
  uint8_t lsb = 0U;
  err = i2c_read_byte(true, &msb);
  if (err == POWER_ERR_NONE) {
    err = i2c_read_byte(false, &lsb);
  }
  i2c_stop();
  if (err != POWER_ERR_NONE) {
    return err;
  }

  *value = (uint16_t)(((uint16_t)msb << 8) | lsb);
  return POWER_ERR_NONE;
}

/* ------------------------------------------------------------- INA219 ---- */

static void record_error(uint8_t error, uint8_t stage)
{
  snapshot.last_error = error;
  snapshot.fail_stage = stage;
  if (snapshot.error_count < 0xFFU) {
    snapshot.error_count++;
  }
}

static bool ina219_probe(void)
{
  i2c_gpio_init();
  i2c_bus_recover();
  snapshot.flags &= (uint8_t)~(POWER_FLAG_PRESENT | POWER_FLAG_VALID);
  snapshot.i2c_addr = 0U;

  uint8_t err = POWER_ERR_NACK;
  for (uint8_t addr = POWER_MONITOR_I2C_ADDR_DEFAULT; addr <= 0x4FU; addr++) {
    err = i2c_begin((uint8_t)(addr << 1));
    if (err == POWER_ERR_NONE) {
      i2c_stop();
      snapshot.i2c_addr = addr;
      break;
    }
    if (err != POWER_ERR_NACK) {
      break; /* bus fault, not just an empty address */
    }
  }
  if (snapshot.i2c_addr == 0U) {
    record_error(err == POWER_ERR_NACK ? POWER_ERR_NOT_FOUND : err, POWER_STAGE_PROBE);
    return false;
  }

  err = ina219_write(snapshot.i2c_addr, INA219_REG_CONFIG, INA219_CONFIG_VALUE);
  if (err != POWER_ERR_NONE) {
    record_error(err, POWER_STAGE_CONFIG_WRITE);
    return false;
  }

  /* Only a correct read-back proves the INA219 (and the read path) works. */
  uint16_t readback = 0U;
  err = ina219_read(snapshot.i2c_addr, INA219_REG_CONFIG, &readback);
  snapshot.config_readback = readback;
  if (err != POWER_ERR_NONE || readback != INA219_CONFIG_VALUE) {
    record_error(err != POWER_ERR_NONE ? err : POWER_ERR_OTHER, POWER_STAGE_CONFIG_VERIFY);
    return false;
  }

  snapshot.flags |= POWER_FLAG_PRESENT;
  failure_count = 0U;
  return true;
}

void power_monitor_init(bool enabled)
{
  memset(&snapshot, 0, sizeof(snapshot));
  monitor_enabled = enabled;

  if (!enabled) {
    snapshot.flags = POWER_FLAG_DISABLED;
    return;
  }

  /* main() configures USART1 (remapped to PB6/PB7) unconditionally; stop it
   * and its interrupt. USART1_REMAP is deliberately left set: clearing it
   * moves USART1 onto the TIM1 phase PWM pins PA8/PA9/PA10. */
  (void)HAL_UART_DeInit(&huart1);

  last_retry_tick = HAL_GetTick();
  last_poll_tick = last_retry_tick;
  (void)ina219_probe();
}

void power_monitor_process(void)
{
  if (!monitor_enabled) {
    return;
  }

  uint32_t now = HAL_GetTick();

  if ((snapshot.flags & POWER_FLAG_PRESENT) == 0U) {
    if ((uint32_t)(now - last_retry_tick) >= POWER_MONITOR_RETRY_MS) {
      last_retry_tick = now;
      (void)ina219_probe();
    }
    return;
  }

  if ((uint32_t)(now - last_poll_tick) < POWER_MONITOR_POLL_MS) {
    return;
  }
  last_poll_tick = now;

  uint16_t shunt = 0U;
  uint16_t bus = 0U;
  uint8_t stage = POWER_STAGE_READ_SHUNT;
  uint8_t err = ina219_read(snapshot.i2c_addr, INA219_REG_SHUNT, &shunt);
  if (err == POWER_ERR_NONE) {
    stage = POWER_STAGE_READ_BUS;
    err = ina219_read(snapshot.i2c_addr, INA219_REG_BUS, &bus);
  }
  if (err != POWER_ERR_NONE) {
    record_error(err, stage);
    i2c_bus_recover();
    snapshot.flags &= (uint8_t)~POWER_FLAG_VALID;
    if (++failure_count >= POWER_MONITOR_MAX_FAILURES) {
      snapshot.flags &= (uint8_t)~POWER_FLAG_PRESENT; /* re-probe on next retry */
      last_retry_tick = now;
    }
    return;
  }

  failure_count = 0U;
  snapshot.shunt_raw = (int16_t)shunt;
  snapshot.current_ua = ((int32_t)snapshot.shunt_raw * INA219_SHUNT_UV_PER_LSB * 1000) /
                        (int32_t)POWER_MONITOR_SHUNT_MILLIOHM;
  snapshot.bus_mv = (uint16_t)((bus >> 3) * INA219_BUS_MV_PER_LSB);
  if ((bus & INA219_BUS_OVF) != 0U) {
    snapshot.flags |= POWER_FLAG_OVERFLOW;
  } else {
    snapshot.flags &= (uint8_t)~POWER_FLAG_OVERFLOW;
  }
  snapshot.flags |= POWER_FLAG_VALID;
  snapshot.sample_tick = now;
}

void power_monitor_get_snapshot(PowerMonitorSnapshot* out)
{
  if (out != NULL) {
    *out = snapshot;
  }
}

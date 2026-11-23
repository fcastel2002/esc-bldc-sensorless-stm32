/*
 * power_monitor.c
 *
 * INA219 DC-bus voltage/current monitor on I2C1 (PB6 = SCL, PB7 = SDA, no
 * remap), driven through the ST HAL I2C driver in blocking mode.
 *
 * PB6/PB7 are shared with remapped USART1 (UART transport) and PB6 is also
 * TIM4_CH1; the monitor therefore runs only in USB transport mode and the
 * motor-control timer is started without enabling the CH1 output
 * (see motor_control_timer_start()).
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

#define I2C_SPEED_HZ 100000U
#define I2C_TIMEOUT_MS 5U
#define POWER_MONITOR_MAX_FAILURES 5U

#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7

static I2C_HandleTypeDef hi2c1;
static PowerMonitorSnapshot snapshot;
static bool monitor_enabled = false;
static uint8_t failure_count = 0;
static uint32_t last_poll_tick = 0;
static uint32_t last_retry_tick = 0;

static void short_delay(void)
{
  for (volatile uint32_t i = 0; i < 60U; i++) { /* ~5 us at 72 MHz */
  }
}

/* Free a slave stuck mid-byte (up to 9 SCL pulses + STOP) with the pins as GPIO. */
static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOB, &gpio);
  short_delay();

  for (uint8_t i = 0; i < 9U && HAL_GPIO_ReadPin(GPIOB, I2C_SDA_PIN) == GPIO_PIN_RESET; i++) {
    HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_RESET);
    short_delay();
    HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_SET);
    short_delay();
  }
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_RESET);
  short_delay();
  HAL_GPIO_WritePin(GPIOB, I2C_SDA_PIN, GPIO_PIN_RESET);
  short_delay();
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_SET);
  short_delay();
  HAL_GPIO_WritePin(GPIOB, I2C_SDA_PIN, GPIO_PIN_SET);
  short_delay();
}

/* Sample idle SCL/SDA as floating inputs: both must read high when the
 * module's pull-ups are powered. */
static void sample_idle_lines(void)
{
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);
  short_delay();
  short_delay();

  uint8_t state = 0U;
  if (HAL_GPIO_ReadPin(GPIOB, I2C_SCL_PIN) == GPIO_PIN_SET) state |= POWER_LINE_SCL_HIGH;
  if (HAL_GPIO_ReadPin(GPIOB, I2C_SDA_PIN) == GPIO_PIN_SET) state |= POWER_LINE_SDA_HIGH;
  snapshot.line_state = state;
}

static void i2c_init(void)
{
  if (hi2c1.State != HAL_I2C_STATE_RESET) {
    (void)HAL_I2C_DeInit(&hi2c1);
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_AFIO_REMAP_I2C1_DISABLE();

  sample_idle_lines();
  i2c_bus_recover();

  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL; /* the INA219 module provides the pull-ups */
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Errata: the analog filter can latch BUSY after GPIO reconfiguration. */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_SPEED_HZ;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  (void)HAL_I2C_Init(&hi2c1);
}

static uint8_t map_hal_error(HAL_StatusTypeDef status)
{
  uint32_t code = HAL_I2C_GetError(&hi2c1);
  if ((code & HAL_I2C_ERROR_AF) != 0U) return POWER_ERR_NACK;
  if ((code & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) != 0U) return POWER_ERR_BUS;
  if ((code & HAL_I2C_ERROR_TIMEOUT) != 0U) return POWER_ERR_TIMEOUT;
  if (status == HAL_BUSY) return POWER_ERR_BUSY;
  if (status == HAL_TIMEOUT) return POWER_ERR_TIMEOUT;
  return POWER_ERR_OTHER;
}

static void record_error(uint8_t error, uint8_t stage)
{
  snapshot.last_error = error;
  snapshot.fail_stage = stage;
  if (snapshot.error_count < 0xFFU) {
    snapshot.error_count++;
  }
}

static uint8_t ina219_write(uint8_t addr, uint8_t reg, uint16_t value)
{
  uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFFU) };
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, buf, 2U, I2C_TIMEOUT_MS);
  return st == HAL_OK ? POWER_ERR_NONE : map_hal_error(st);
}

static uint8_t ina219_read(uint8_t addr, uint8_t reg, uint16_t* value)
{
  uint8_t buf[2] = { 0U, 0U };
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                                          I2C_MEMADD_SIZE_8BIT, buf, 2U, I2C_TIMEOUT_MS);
  if (st != HAL_OK) {
    return map_hal_error(st);
  }
  *value = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  return POWER_ERR_NONE;
}

static bool ina219_probe(void)
{
  i2c_init();
  snapshot.flags &= (uint8_t)~(POWER_FLAG_PRESENT | POWER_FLAG_VALID);
  snapshot.i2c_addr = 0U;

  HAL_StatusTypeDef st = HAL_ERROR;
  for (uint8_t addr = POWER_MONITOR_I2C_ADDR_DEFAULT; addr <= 0x4FU; addr++) {
    st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2U, I2C_TIMEOUT_MS);
    if (st == HAL_OK) {
      snapshot.i2c_addr = addr;
      break;
    }
  }
  if (snapshot.i2c_addr == 0U) {
    /* A plain NACK on every address means nobody answered; anything else is a bus fault. */
    record_error(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF ? POWER_ERR_NOT_FOUND
                                                               : map_hal_error(st),
                 POWER_STAGE_PROBE);
    return false;
  }

  uint8_t err = ina219_write(snapshot.i2c_addr, INA219_REG_CONFIG, INA219_CONFIG_VALUE);
  if (err != POWER_ERR_NONE) {
    record_error(err, POWER_STAGE_CONFIG_WRITE);
    return false;
  }

  /* A floating bus can fake an address ACK; only a correct read-back proves the INA219. */
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

  /* main() configures USART1 (remapped to PB6/PB7) unconditionally; release it. */
  (void)HAL_UART_DeInit(&huart1);
  __HAL_AFIO_REMAP_USART1_DISABLE();

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
    snapshot.flags &= (uint8_t)~POWER_FLAG_VALID;
    if (++failure_count >= POWER_MONITOR_MAX_FAILURES) {
      snapshot.flags &= (uint8_t)~POWER_FLAG_PRESENT; /* re-init + re-probe on next retry */
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

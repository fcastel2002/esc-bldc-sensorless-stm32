/*
 * power_monitor.c
 *
 * INA219 DC-bus voltage/current monitor on I2C1 (PB6 = SCL, PB7 = SDA, no remap).
 *
 * The HAL I2C module is not part of this project, so I2C1 is driven with a
 * small register-level, polling, timeout-guarded master that only implements
 * what the INA219 needs: 3-byte register write and 2-byte register read
 * (RM0008 "method 2" for N = 2 with POS/ACK handling).
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
#define I2C_TIMEOUT_MS 3U
#define POWER_MONITOR_MAX_FAILURES 5U

#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7

static PowerMonitorSnapshot snapshot;
static bool monitor_enabled = false;
static uint8_t failure_count = 0;
static uint32_t last_poll_tick = 0;
static uint32_t last_retry_tick = 0;

/* ---------------------------------------------------------------- I2C1 ---- */

static void i2c_short_delay(void)
{
  /* ~5 us at 72 MHz; only used for bus recovery bit-banging. */
  for (volatile uint32_t i = 0; i < 60U; i++) {
  }
}

static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOB, &gpio);
  i2c_short_delay();

  /* Clock out any slave stuck mid-byte, then issue a STOP. */
  for (uint8_t i = 0; i < 9U; i++) {
    HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_RESET);
    i2c_short_delay();
    HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_SET);
    i2c_short_delay();
  }
  HAL_GPIO_WritePin(GPIOB, I2C_SDA_PIN, GPIO_PIN_RESET);
  i2c_short_delay();
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_PIN, GPIO_PIN_SET);
  i2c_short_delay();
  HAL_GPIO_WritePin(GPIOB, I2C_SDA_PIN, GPIO_PIN_SET);
  i2c_short_delay();
}

static void i2c_hw_init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_AFIO_REMAP_I2C1_DISABLE();

  I2C1->CR1 = 0U;
  i2c_bus_recover();

  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Software reset clears a BUSY flag latched by the analog filter (errata). */
  I2C1->CR1 = I2C_CR1_SWRST;
  I2C1->CR1 = 0U;

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t freq_mhz = pclk1 / 1000000U;
  I2C1->CR2 = freq_mhz & I2C_CR2_FREQ;
  I2C1->CCR = pclk1 / (2U * I2C_SPEED_HZ); /* standard mode, 50 % duty */
  I2C1->TRISE = freq_mhz + 1U;             /* 1000 ns max rise time */
  I2C1->CR1 = I2C_CR1_PE;
}

static void i2c_abort(void)
{
  I2C1->SR1 &= ~I2C_SR1_AF;
  I2C1->CR1 |= I2C_CR1_STOP;
  I2C1->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);
}

static bool i2c_wait_sr1(uint32_t mask)
{
  uint32_t start = HAL_GetTick();
  while ((I2C1->SR1 & mask) == 0U) {
    if ((I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO)) != 0U) {
      I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
      return false;
    }
    if ((uint32_t)(HAL_GetTick() - start) > I2C_TIMEOUT_MS) {
      return false;
    }
  }
  return true;
}

static bool i2c_start(uint8_t addr_byte)
{
  uint32_t start = HAL_GetTick();
  while ((I2C1->CR1 & I2C_CR1_STOP) != 0U || (I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    if ((uint32_t)(HAL_GetTick() - start) > I2C_TIMEOUT_MS) {
      return false;
    }
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (!i2c_wait_sr1(I2C_SR1_SB)) {
    return false;
  }
  I2C1->DR = addr_byte;
  return i2c_wait_sr1(I2C_SR1_ADDR);
}

static bool ina219_write(uint8_t addr, uint8_t reg, uint16_t value)
{
  if (!i2c_start((uint8_t)(addr << 1))) {
    i2c_abort();
    return false;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  const uint8_t bytes[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFFU) };
  for (uint8_t i = 0; i < 3U; i++) {
    I2C1->DR = bytes[i];
    if (!i2c_wait_sr1(I2C_SR1_TXE)) {
      i2c_abort();
      return false;
    }
  }
  if (!i2c_wait_sr1(I2C_SR1_BTF)) {
    i2c_abort();
    return false;
  }
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

static bool ina219_read(uint8_t addr, uint8_t reg, uint16_t* value)
{
  /* Pointer write. */
  if (!i2c_start((uint8_t)(addr << 1))) {
    i2c_abort();
    return false;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->DR = reg;
  if (!i2c_wait_sr1(I2C_SR1_BTF)) {
    i2c_abort();
    return false;
  }
  I2C1->CR1 |= I2C_CR1_STOP;

  /* Two-byte read: ACK+POS before START, clear ACK before clearing ADDR. */
  I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;
  if (!i2c_start((uint8_t)((addr << 1) | 1U))) {
    i2c_abort();
    return false;
  }
  I2C1->CR1 &= ~I2C_CR1_ACK;
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  if (!i2c_wait_sr1(I2C_SR1_BTF)) {
    i2c_abort();
    return false;
  }
  I2C1->CR1 |= I2C_CR1_STOP;
  uint8_t msb = (uint8_t)I2C1->DR;
  uint8_t lsb = (uint8_t)I2C1->DR;
  I2C1->CR1 &= ~I2C_CR1_POS;

  *value = (uint16_t)(((uint16_t)msb << 8) | lsb);
  return true;
}

/* ------------------------------------------------------------- INA219 ---- */

static bool ina219_probe(void)
{
  i2c_hw_init();
  snapshot.flags &= (uint8_t)~(POWER_FLAG_PRESENT | POWER_FLAG_VALID);
  snapshot.i2c_addr = 0U;

  if (ina219_write(POWER_MONITOR_I2C_ADDR_DEFAULT, INA219_REG_CONFIG, INA219_CONFIG_VALUE)) {
    snapshot.i2c_addr = POWER_MONITOR_I2C_ADDR_DEFAULT;
  } else {
    for (uint8_t addr = 0x40U; addr <= 0x4FU; addr++) {
      if (addr != POWER_MONITOR_I2C_ADDR_DEFAULT &&
          ina219_write(addr, INA219_REG_CONFIG, INA219_CONFIG_VALUE)) {
        snapshot.i2c_addr = addr;
        break;
      }
    }
  }

  if (snapshot.i2c_addr == 0U) {
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
  if (!ina219_read(snapshot.i2c_addr, INA219_REG_SHUNT, &shunt) ||
      !ina219_read(snapshot.i2c_addr, INA219_REG_BUS, &bus)) {
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

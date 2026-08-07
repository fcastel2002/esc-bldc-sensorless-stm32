#include "bldc_driver.h"

// MANEJO DE CONMUTACION
// Variables
static volatile uint16_t g_pwm_val          = 0;
static volatile int8_t   commutation_step = 0;
bool              floating_W         = false;
bool              floating_U         = false;
bool              floating_V         = false;

// Funciones
static void commutate(int8_t step);




/**
 * @brief Realiza la conmutación de las fases del motor BLDC según el paso especificado
 *
 * Esta función controla las seis secuencias de conmutación del motor
 * configurando los MOSFETs de potencia y los canales PWM correspondientes. Cada paso
 * de conmutación energiza dos fases mientras deja la tercera flotante para detectar
 * el cruce por cero de la fuerza contraelectromotriz (back-EMF).
 *
 * @param step Paso de conmutación a ejecutar (POS_UV, POS_UW, POS_VW, POS_VU, POS_WU, POS_WV)
 *             También soporta POS_INIT y POS_SOUND para configuraciones especiales
 *
 * @details Secuencia de conmutación para motor BLDC:
 * - POS_UV: U=PWM, V=LOW, W=FLOAT (detecta cruce por cero en W con flanco descendente)
 * - POS_UW: U=PWM, W=LOW, V=FLOAT (detecta cruce por cero en V con flanco ascendente)
 * - POS_VW: V=PWM, W=LOW, U=FLOAT (detecta cruce por cero en U con flanco descendente)
 * - POS_VU: V=PWM, U=LOW, W=FLOAT (detecta cruce por cero en W con flanco ascendente)
 * - POS_WU: W=PWM, U=LOW, V=FLOAT (detecta cruce por cero en V con flanco descendente)
 * - POS_WV: W=PWM, V=LOW, U=FLOAT (detecta cruce por cero en U con flanco ascendente)
 *
 * @note Configuraciones especiales:
 * - POS_INIT: Configuración inicial para arranque
 * - POS_SOUND: Todas las fases con PWM para generar sonido
 *
 * @warning Esta función modifica direct  amente registros GPIO y TIM para máxima velocidad
 * @warning Debe llamarse con un paso válido para evitar estados indefinidos del motor
 */
static void commutate(int8_t step)
{
  switch (step) {

  case POS_UV:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    floating_W = true;
    floating_V = false;
    floating_U = false;

    GPIOA->ODR &= ~EN_W;
    GPIOB->ODR |= EN_U;
    GPIOB->ODR |= EN_V;
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
    break;
  case POS_UW:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    floating_W = false;
    floating_U = false;
    floating_V = true;
    GPIOB->ODR &= ~EN_V;
    GPIOB->ODR |= EN_U;
    GPIOA->ODR |= EN_W;
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
    break;
  case POS_VW:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

    floating_W = false;
    floating_V = false;
    floating_U = true;
    GPIOB->ODR &= ~EN_U;

    GPIOA->ODR |= EN_W;
    GPIOB->ODR |= EN_V;
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);

    break;
  case POS_VU:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    floating_W = true;
    floating_V = false;
    floating_U = false;
    GPIOB->ODR |= EN_U;
    GPIOB->ODR |= EN_V;
    GPIOA->ODR &= ~EN_W;
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
    break;
  case POS_WU:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);

    floating_W = false;
    floating_U = false;
    floating_V = true;
    GPIOB->ODR &= ~EN_V;
    GPIOA->ODR |= EN_W;
    GPIOB->ODR |= EN_U;
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
    break;
  case POS_WV:
    PWM_STOP();
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

    floating_W = false;
    floating_V = false;
    floating_U = true;
    GPIOB->ODR &= ~EN_U;
    GPIOA->ODR |= EN_W;
    GPIOB->ODR |= EN_V;
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
    break;

  case POS_INIT:
    GPIOB->ODR |= EN_U;
    GPIOB->ODR |= EN_V;
    GPIOA->ODR |= EN_W;
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
    break;
  case POS_SOUND:
    GPIOB->ODR |= EN_U;
    GPIOB->ODR |= EN_V;
    GPIOA->ODR |= EN_W;
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, g_pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, g_pwm_val);
    break;
  default:
    break;
  }
}

void bldc_commutate() {
  if (direction == 0) {
    commutation_step = safe_mod(commutation_step + 1, NUM_POS);
  } else {
    commutation_step = safe_mod(commutation_step - 1, NUM_POS);
  }
  commutate(commutation_step);
}

void bldc_set_pwm(uint16_t new_pwm) {
  g_pwm_val = (uint16_t)new_pwm;
}

uint16_t bldc_get_pwm(void) {
  return g_pwm_val;
}

int8_t bldc_get_commutation_step(void) {
  return commutation_step;
}

void bldc_disable_power_stage(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  floating_U = false;
  floating_V = false;
  floating_W = false;
  PWM_STOP();
  GPIOB->ODR &= ~(EN_U | EN_V);
  GPIOA->ODR &= ~EN_W;
  if (primask == 0U) __enable_irq();
}

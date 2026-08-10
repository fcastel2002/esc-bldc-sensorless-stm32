#include "speed_sensor.h"
#include "hard_config.h"



// DECLARACION DE FUNCIONES PRIVADAS
static void calculate_consensus_speed(void);

// VARIABLES PARA MEDICION DE VELOCIDAD DE CONSENSO TRIFASICO
static PhaseMeasurementThreePhase phase_measurements[PHASE_COUNT] = {0};
static volatile uint16_t   consensus_speed                 = 0;
static volatile uint8_t    active_phases_count             = 0;
static volatile uint8_t    speed_measurement_ready         = 0;
//====================================================
// VARIABLES PARA CHEQUEO DE CALIDAD DEL CRUCE POR CERO
static uint16_t  last_W_timestamp = 0;
static uint16_t  W_periods[ZCP_TO_CHECK];
static uint8_t   W_period_idx             = 0;
static uint8_t   valid_W_zcp              = 0;
//====================================================
// VARIABLES DE VELOCIDAD AUXILIARES
static volatile uint16_t filtered_speed;
static volatile int32_t  diff_speed;
//====================================================
// VARIABLES PARA FILTRO DIGITAL DE VELOCIDAD INPUT CAPTURE
static volatile uint8_t  speed_buffer_size  = 1;
static volatile uint16_t speed_buffer[2]    = {0};
static volatile uint16_t filtered_speed      = 0;
static volatile uint16_t last_speed_capture  = 1;
//====================================================
static void get_phase_diagnostics(uint16_t* phase_speeds, uint8_t* phase_status);



/**
 * @brief Procesa mediciones de una fase específica para el sistema de consenso tri-fase
 *
 * Esta función analiza los períodos entre cruces por cero de una fase individual,
 * mantiene un historial de mediciones y evalúa la consistencia de las mismas.
 * Es parte del sistema de medición redundante tri-fase para mayor confiabilidad.
 *
 * @param phase_idx Índice de la fase (0=W, 1=V, 2=U)
 * @param current_timestamp Timestamp actual capturado del timer (16-bit)
 *
 * @details Algoritmo de procesamiento:
 * 1. Calcula período entre timestamp actual y anterior
 * 2. Maneja overflow del contador de 16-bit correctamente
 * 3. Filtra períodos fuera de rango válido (50 < período < 50000)
 * 4. Almacena período en buffer circular de ZCP_BUFFER_SIZE elementos
 * 5. Calcula período promedio cuando buffer está lleno
 * 6. Evalúa consistencia comparando cada período con el promedio
 * 7. Marca fase como consistente si todos los períodos están dentro de tolerancia
 *
 * @note Estructura de datos utilizada:
 * - phase_measurements[phase_idx]: Estructura con buffer circular y estadísticas
 * - last_timestamp: Último timestamp registrado para la fase
 * - periods[]: Buffer circular de últimos períodos
 * - avg_period: Período promedio calculado
 * - is_consistent: Flag de consistencia de mediciones
 *
 * @note Criterios de validación:
 * - Período debe estar entre 50 y 50000 para evitar ruido
 * - Requiere ZCP_BUFFER_SIZE muestras para calcular promedio
 * - Tolerancia de consistencia definida por SPEED_TOLERANCE_PCT
 *
 * @warning Esta función debe ser llamada desde contexto de interrupción
 * @warning Los datos son válidos solo después de ZCP_BUFFER_SIZE mediciones
 */
void speed_sensor_process_phase_measurement(uint8_t phase_idx, uint16_t current_timestamp)
{
  PhaseMeasurementThreePhase* phase = &phase_measurements[phase_idx];

  if (phase->last_timestamp != 0) {
    uint16_t period;
    if (phase->last_timestamp > current_timestamp) {
      period = (0xFFFF - phase->last_timestamp) + current_timestamp + 1;
    } else {
      period = current_timestamp - phase->last_timestamp;
    }

    // Filtrar períodos muy pequeños o muy grandes (ruido)
    if (period > ZCP_VALID_MIN_THRESHOLD && period < ZCP_VALID_MAX_THRESHOLD) {
      phase->periods[phase->period_idx] = period;
      phase->period_idx                 = (phase->period_idx + 1) % ZCP_BUFFER_SIZE;

      if (phase->valid_periods < ZCP_BUFFER_SIZE) {
        phase->valid_periods++;
      }

      // Calcular promedio y consistencia si tenemos suficientes muestras
      if (phase->valid_periods >= ZCP_BUFFER_SIZE) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < ZCP_BUFFER_SIZE; i++) {
          sum += phase->periods[i];
        }
        phase->avg_period = sum / ZCP_BUFFER_SIZE;

        // Verificar consistencia
        phase->is_consistent = 1;
        for (uint8_t i = 0; i < ZCP_BUFFER_SIZE; i++) {
          uint32_t tolerance = phase->avg_period * SPEED_TOLERANCE_PCT / 100;
          if (abs(phase->periods[i] - phase->avg_period) > tolerance) {
            phase->is_consistent = 0;
            break;
          }
        }
      }
    }
  }
  phase->last_timestamp = current_timestamp;
}

/**
 * @brief Calcula velocidad por consenso utilizando mediciones de las tres fases
 *
 * Esta función implementa un algoritmo de consenso para determinar la velocidad
 * del motor basándose en las mediciones de múltiples fases, proporcionando
 * mayor robustez y confiabilidad que la medición de una sola fase.
 *
 * @details Algoritmo de consenso:
 * 1. Recopila velocidades válidas de todas las fases consistentes
 * 2. Requiere mínimo 2 fases para establecer consenso
 * 3. Calcula promedio inicial de todas las velocidades válidas
 * 4. Verifica que las velocidades estén dentro del umbral de consenso
 * 5. Calcula velocidad final promediando solo las mediciones consensuadas
 * 6. Si solo hay 1 fase válida, la usa con precaución
 * 7. Actualiza flags de estado del sistema de medición
 *
 * @note Variables globales actualizadas:
 * - consensus_speed: Velocidad calculada por consenso
 * - active_phases_count: Número de fases con mediciones válidas
 * - speed_measurement_ready: Flag que indica si hay medición confiable
 *
 * @note Criterios de validación:
 * - Fase debe ser consistente (is_consistent = 1)
 * - Fase debe tener buffer lleno (valid_periods >= ZCP_BUFFER_SIZE)
 * - Velocidades deben estar dentro de SPEED_CONSENSUS_THRESHOLD (15%)
 * - Mínimo 2 fases para consenso, 1 fase para operación degradada
 *
 * @note Estados de salida:
 * - speed_measurement_ready = 1: Medición confiable disponible
 * - speed_measurement_ready = 0: Mediciones inconsistentes o insuficientes
 *
 * @warning Esta función debe ser llamada periódicamente para mantener mediciones actuales
 */
static void calculate_consensus_speed(void)
{
  uint16_t valid_speeds[PHASE_COUNT];
  uint8_t  valid_count = 0;

  // Recopilar velocidades válidas de cada fase
  for (uint8_t i = 0; i < PHASE_COUNT; i++) {
    if (phase_measurements[i].is_consistent &&
        phase_measurements[i].valid_periods >= ZCP_BUFFER_SIZE) {
      valid_speeds[valid_count] = phase_measurements[i].avg_period;
      valid_count++;
    }
  }

  active_phases_count = valid_count;

  if (valid_count >= 2) { // Necesitamos al menos 2 fases para consenso
    uint32_t sum             = 0;
    uint8_t  consensus_count = 0;

    // Calcular promedio inicial
    for (uint8_t i = 0; i < valid_count; i++) {
      sum += valid_speeds[i];
    }
    uint16_t avg_speed = sum / valid_count;

    // Verificar consenso entre fases
    sum = 0;
    for (uint8_t i = 0; i < valid_count; i++) {
      uint32_t tolerance = avg_speed * SPEED_CONSENSUS_THRESHOLD / 100;
      if (abs(valid_speeds[i] - avg_speed) <= tolerance) {
        sum += valid_speeds[i];
        consensus_count++;
      }
    }

    if (consensus_count >= 2) {
      consensus_speed         = sum / consensus_count;
      speed_measurement_ready = 1;
    } else {
      speed_measurement_ready = 0;
    }
  } else if (valid_count == 1) {
    // Si solo tenemos una fase válida, la usamos con precaución
    consensus_speed         = valid_speeds[0];
    speed_measurement_ready = 1;
  } else {
    speed_measurement_ready = 0;
  }
}

void speed_sensor_reset(void)
{
  for (uint8_t i = 0; i < PHASE_COUNT; i++) {
    phase_measurements[i].last_timestamp = 0;
    phase_measurements[i].period_idx     = 0;
    phase_measurements[i].valid_periods  = 0;
    phase_measurements[i].avg_period     = 0;
    phase_measurements[i].is_consistent  = 0;
    for (uint8_t j = 0; j < ZCP_BUFFER_SIZE; j++) {
      phase_measurements[i].periods[j] = 0;
    }
  }
  consensus_speed         = 0;
  active_phases_count     = 0;
  speed_measurement_ready = 0;
  last_W_timestamp        = 0;
  W_period_idx            = 0;
  valid_W_zcp             = 0;
  filtered_speed          = 0;
  last_speed_capture      = 1;

  for (uint8_t i = 0; i < ZCP_TO_CHECK; i++) {
    W_periods[i] = 0;
  }

  for (uint8_t i = 0; i < speed_buffer_size; i++) {
    speed_buffer[i] = 0;
  }
}

// Función de diagnóstico para monitorear estado de mediciones
void get_phase_diagnostics(uint16_t* phase_speeds, uint8_t* phase_status)
{
  for (uint8_t i = 0; i < PHASE_COUNT; i++) {
    phase_speeds[i] = phase_measurements[i].avg_period;
    phase_status[i] = phase_measurements[i].is_consistent ? 1 : 0;
  }
}
uint16_t get_actual_speed(void)
{

  return (uint16_t)filtered_speed;
}
uint16_t rpm_to_period(uint16_t rpm)
{
  if (rpm == 0)
    return 0xFFFF;

  // Calcular ticks entre cruces por cero con aritmetica entera.
  uint8_t pole_pairs = get_pole_pairs();
  if (pole_pairs == 0)
    return SPEED_MIN;

  uint32_t timer_hz = HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1u);

  uint32_t ticks = (timer_hz * 30u) / ((uint32_t)rpm * pole_pairs);

  // Aplicar límites del sistema
  if (ticks < SPEED_MAX)
    return SPEED_MAX;
  if (ticks > SPEED_MIN)
    return SPEED_MIN;

  return (uint16_t)ticks;
}
uint16_t period_to_rpm(uint16_t period)
{
  if (period == 0 || period == 0xFFFF)
    return 0;

  // Convertir ticks entre cruces por cero a RPM mecanicos.
  uint8_t pole_pairs = get_pole_pairs();
  if (pole_pairs == 0)
    return 0;

  uint32_t timer_hz = HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1u);

  uint32_t rpm = (timer_hz * 30u) / ((uint32_t)period * pole_pairs);

  return rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)rpm;
}



uint16_t filtro_media_movil_zc(uint16_t measurement)
{
  int32_t                 new_speed   = 0;
  static uint16_t         prev_speed  = 0;
  static volatile uint8_t speed_index = 0;
  if (measurement < SPEED_MAX || measurement > SPEED_MIN) {
    if (prev_speed >= SPEED_MAX && prev_speed <= SPEED_MIN) {

      speed_buffer[speed_index] = prev_speed;

    } else {
      speed_buffer[speed_index] =
          (SPEED_MAX - SPEED_MIN) / 2; // Valor por defecto si la medición es inválida
    }

  } else {
    speed_buffer[speed_index] = measurement;
    prev_speed                = measurement;
  }
  speed_index = (speed_index + 1) % speed_buffer_size;

  for (int i = 0; i < speed_buffer_size; i++) {
    new_speed += speed_buffer[i];
  }
  new_speed = new_speed / speed_buffer_size;

  return (uint16_t)new_speed;
}

void speed_sensor_handle_W_measurement(uint16_t current_timestamp)
{
    speed_sensor_process_phase_measurement(0, current_timestamp); // Fase W = índice 0

    if (last_W_timestamp != 0) {
        uint16_t period;
        if (last_W_timestamp > current_timestamp) {
            period = (0xFFFF - last_W_timestamp) + current_timestamp + 1;
        } else {
            period = current_timestamp - last_W_timestamp;
        }
        W_periods[W_period_idx] = period;
        W_period_idx            = (W_period_idx + 1) % ZCP_TO_CHECK;
        if (valid_W_zcp < ZCP_TO_CHECK) {
            valid_W_zcp++;
        }
        if (valid_W_zcp == ZCP_TO_CHECK) {
            uint16_t avg_period    = 0;
            uint8_t  is_consistent = 1;
            for (uint8_t i = 0; i < ZCP_TO_CHECK; i++) {
                avg_period += W_periods[i];
            }
            avg_period /= ZCP_TO_CHECK;
            if (avg_period > 20) {
                for (uint8_t i = 0; i < ZCP_TO_CHECK; i++) {
                    uint32_t tolerance = avg_period * SPEED_TOLERANCE_PCT / 100;
                    if (abs(W_periods[i] - avg_period) > tolerance) {
                        is_consistent = 0;
                        break;
                    }
                }
                if (is_consistent) {
                    valid_W_zcp              = 0;
                    consistent_zero_crossing = 1;
                    speed_buffer_size        = 1;
                } else {
                    consistent_zero_crossing = 0;
                }
            }
            valid_W_zcp = 0;
        }
    }
    last_W_timestamp = current_timestamp;
}


void speed_sensor_handle_consensus(void)
{
    static uint8_t speed_calc_counter = 0;

    speed_calc_counter++;
    if (speed_calc_counter < 2)
        return;

    calculate_consensus_speed();

    if (speed_measurement_ready && consensus_speed > 0) {
        diff_speed     = consensus_speed;
        filtered_speed = filtro_media_movil_zc(diff_speed);
    } else {
        /* Fallback al método original (captura simple) */
        uint16_t current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
        if (last_speed_capture != 0) {
            uint16_t speed_period;
            if (last_speed_capture > current_capture) {
                speed_period = (0xFFFF - last_speed_capture) + current_capture + 1;
            } else {
                speed_period = current_capture - last_speed_capture;
            }
            diff_speed     = speed_period;
            filtered_speed = filtro_media_movil_zc(diff_speed);
        }
        last_speed_capture = current_capture;
    }
    speed_calc_counter = 0;
}

uint16_t speed_sensor_get_speed_range()
{

  return (SPEED_MIN - SPEED_MAX);
}

uint16_t speed_sensor_get_speed_rpm(void)
{
  return period_to_rpm((uint16_t)filtered_speed);
}

uint16_t speed_sensor_get_speed_period(void)
{
  return (uint16_t)filtered_speed;
}

uint8_t speed_sensor_get_bemf_quality_flags(void)
{
  uint8_t flags = 0U;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (speed_measurement_ready != 0U) flags |= 1U;
  for (uint8_t phase = 0; phase < PHASE_COUNT; phase++) {
    if (phase_measurements[phase].is_consistent != 0U) {
      flags |= (uint8_t)(1U << (phase + 1U));
    }
  }
  if (primask == 0U) __enable_irq();
  return flags;
}

bool speed_sensor_is_ready(void)
{
  return speed_measurement_ready != 0U;
}


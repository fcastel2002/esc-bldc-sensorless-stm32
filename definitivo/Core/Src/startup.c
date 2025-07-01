/*
 * startup.c
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */


#include "startup.h"
#define Q15_ONE 32768U
#define Q15_MAX 32760U

volatile uint16_t zero_crossings = 0;

bool ready_for_update_pwm = true;
bool finished_foc_startup = false;


uint16_t sin_table_U[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_V[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_W[SIN_TABLE_SIZE] = {0};
static uint16_t zero_sine_value = 0;
volatile uint16_t mod_q15 = 6553;       // 0.2 in Q15
static void generate_sine_tables(uint16_t pwm_arr, uint8_t direction);
static void executeTransition(void);

static void generate_sine_tables(uint16_t pwm_arr, uint8_t direction)
{
    for (uint16_t i = 0; i < SIN_TABLE_SIZE; i++)
    {
        /* Ángulo base 0°-360° (rad) */
        float angle = 2.0f * (float)M_PI * i / SIN_TABLE_SIZE;

        /* -------------------------------------------------------------
         * Convertimos de rango [-1 … +1] a [0 … Q15_MAX]
         *    ( 0 = 0 % duty, 32 767 ≈ 100 % duty )
         *    Fórmula:  q15 = (sin + 1) * 0.5 * 32767
         * -------------------------------------------------------------*/
        uint16_t u = (uint16_t)((sinf(angle) * 0.5f + 0.5f) * Q15_MAX);
        uint16_t v = (uint16_t)((sinf(angle - 2.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);
        uint16_t w = (uint16_t)((sinf(angle - 4.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);

        /* Sentido de giro: cambia el orden de las fases si direction=1 */
        if (direction == 0)         // Sentido “normal”
        {
            sin_table_U[i] = u;
            sin_table_V[i] = v;
            sin_table_W[i] = w;
        }
        else                        // Sentido invertido
        {
            sin_table_U[i] = u;
            sin_table_V[i] = w;     // intercambio V<->W
            sin_table_W[i] = v;
        }
    }
}
static const uint16_t initial_arr = 8000;
SineDriveController sine_ctrl = {
    .phase_step = 3,
    .timer_arr =initial_arr,// Frecuencia inicial baja (~50 Hz)
};

/**
 * @brief Inicializa y ejecuta el arranque suave del motor usando control FOC sinusoidal
 *
 * Esta función implementa un arranque suave del motor BLDC utilizando señales sinusoidales
 * en las tres fases. El arranque FOC (Field Oriented Control) permite acelerar gradualmente
 * el motor desde velocidad cero hasta una velocidad donde el control six-step puede tomar
 * el control de manera estable.
 *
 * @details Secuencia de inicialización:
 * 1. Configura el timer ARR con frecuencia inicial baja (initial_arr = 8000)
 * 2. Genera tablas sinusoidales para las tres fases según la dirección del motor
 * 3. Configura TIM4 como timer de actualización PWM con PSC=35
 * 4. Habilita todas las salidas PWM y los enable de las tres fases
 * 5. Inicia el timer TIM4 con interrupción de actualización
 * 6. Cambia el estado de la aplicación a FOC_STARTUP
 *
 * @note Configuración de hardware:
 * - TIM1: Generación PWM para las tres fases del motor
 * - TIM4: Timer de control de frecuencia del arranque FOC
 * - PSC de TIM4: 35 (para obtener frecuencia de actualización adecuada)
 * - ARR inicial: 8000 (frecuencia aproximada de 50 Hz)
 *
 * @note Variables globales inicializadas:
 * - app_state: Cambia a FOC_STARTUP
 * - float_U, float_V, float_W: Todas configuradas como true
 * - sine_ctrl.startup_counter: Resetea a 0
 * - sine_ctrl.timer_arr: Configurado con valor inicial
 *
 * @note Solo genera las tablas sinusoidales en la primera ejecución para optimizar performance
 * 
 * @warning Esta función debe ser llamada solo cuando el motor está detenido
 * @warning Requiere que las variables de dirección y configuración estén previamente establecidas
 */
void foc_startup(void){
	static uint8_t is_first_startup = 1;
    sine_ctrl.timer_arr = initial_arr;
	if(is_first_startup){
		generate_sine_tables(TIM1->ARR, direction);
		is_first_startup = 0;
	}
    TIM4->PSC =  35;
    TIM4->ARR = sine_ctrl.timer_arr;

	app_state = FOC_STARTUP;
	PWM_INIT();
	GPIOB->ODR |= EN_U;
	GPIOB->ODR |= EN_V;
	GPIOA->ODR |= EN_W;
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
    float_U = true;
    float_V = true;
    float_W = true;
    
    // Reset startup counter
    sine_ctrl.startup_counter = 0;
    
	HAL_TIM_Base_Start_IT(&htim4);
}

volatile uint16_t pwm_u =0;
volatile uint16_t pwm_v = 0;
volatile uint16_t pwm_w = 0;
volatile uint16_t sine_u = 0;
volatile uint16_t sine_v = 0;
volatile uint16_t sine_w = 0;

const  uint16_t ARR_LOCK_TEST = 1800;     // ≈300 rpm eléctricos
const  uint16_t STARTUP_ITERATIONS = 2000; // Fixed startup time iterations

/**
 * @brief Actualiza las señales PWM durante el arranque FOC sinusoidal del motor
 *
 * Esta función es llamada periódicamente por la interrupción de TIM4 para generar
 * señales sinusoidales trifásicas con rampa de frecuencia y amplitud. Implementa
 * un arranque de duración fija que acelera gradualmente el motor hasta alcanzar
 * una velocidad estable para la transición al control six-step.
 *
 * @details Algoritmo de control:
 * 1. Verifica que el estado sea FOC_STARTUP, sino retorna
 * 2. Incrementa contador de fase según phase_step para generar rotación
 * 3. Implementa rampa de frecuencia reduciendo timer_arr hasta ARR_LOCK_TEST
 * 4. Implementa rampa de amplitud incrementando mod_q15 hasta 1.0 (Q15_ONE)
 * 5. Calcula valores PWM sinusoidales para cada fase usando las tablas precalculadas
 * 6. Aplica modulación de amplitud y escalado a rango PWM del timer
 * 7. Después de STARTUP_ITERATIONS, ejecuta transición a modo six-step
 *
 * @note Parámetros de control:
 * - phase_step: Incremento de fase por actualización (determina velocidad eléctrica)
 * - timer_arr: Período de actualización (decrece para acelerar)
 * - mod_q15: Amplitud de modulación en formato Q15 (0.2 a 1.0)
 * - ARR_LOCK_TEST: Frecuencia objetivo (~300 rpm eléctricos, valor 1800)
 * - STARTUP_ITERATIONS: Duración fija del arranque (2000 iteraciones)
 *
 * @details Generación de señales PWM:
 * - Utiliza tablas sinusoidales precalculadas con desfase de 120° eléctricos
 * - Aplica modulación de amplitud: señal_final = sin_table[idx] * mod_q15
 * - Escalado final: PWM = (señal_modulada >> 15) * TIM1->ARR >> 15
 * - Actualiza directamente los registros de comparación de TIM1
 *
 * @note Variables globales utilizadas:
 * - sine_ctrl: Estructura de control con contadores y parámetros
 * - sin_table_U/V/W: Tablas sinusoidales para cada fase
 * - mod_q15: Amplitud de modulación en formato Q15
 * - app_state: Estado actual de la aplicación
 *
 * @warning Solo opera cuando app_state == FOC_STARTUP
 * @warning Debe ser llamada desde ISR de TIM4 con frecuencia constante
 * @warning La transición automática puede interrumpir el arranque prematuramente
 */
void update_pwm_startup_foc(void)
{
    if(app_state != FOC_STARTUP) return;

    static uint16_t phase_counter = 0;

    /* ---------- Generación senoidal open-loop ---------- */
    phase_counter += sine_ctrl.phase_step;
    if(phase_counter >= SIN_TABLE_SIZE) phase_counter -= SIN_TABLE_SIZE;

    /* Rampa de frecuencia */
    if(sine_ctrl.timer_arr > ARR_LOCK_TEST)
        sine_ctrl.timer_arr -= 8;
    TIM4->ARR = sine_ctrl.timer_arr;

    
    if(mod_q15 <(uint16_t)(1*Q15_ONE))
        mod_q15 += (mod_q15 >> 6);
        
    /* ---------- Simple iteration counter startup ---------- */
    sine_ctrl.startup_counter++;
    
    if(sine_ctrl.startup_counter >= STARTUP_ITERATIONS) {
        HAL_TIM_Base_Stop_IT(&htim4);
        executeTransition();
        phase_counter = 0;
        return;
    }

    /* ---------- Actualización de PWM ---------- */
    uint16_t idx = phase_counter;
    uint32_t u_raw = (sin_table_U[idx]*mod_q15);
    uint32_t v_raw = (sin_table_V[idx]*mod_q15);
    uint32_t w_raw = (sin_table_W[idx]*mod_q15);
    uint16_t du  = (u_raw >> 15) * TIM1->ARR >> 15;
    uint16_t dv  = (v_raw >> 15) * TIM1->ARR >> 15;
    uint16_t dw  = (w_raw >> 15) * TIM1->ARR >> 15;
    
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, du);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V, dv);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, dw);
}

volatile bool ready_for_running = false;

static void executeTransition(void) {
    // Configurar para six-step
    pwmVal = max_pwm * 0.45f;  // 30% duty cycle inicial
    // Desactivar salidas temporalmente
    GPIOB->ODR &= ~EN_U;
    GPIOB->ODR &= ~EN_V;
    GPIOA->ODR &= ~EN_W;

    // Detener PWM sinusoidal
    PWM_STOP();

    // Inicializar configuración six-step
    PWM_INIT();

    // Configurar estado de las fases
    float_U = true;
    float_V = true;
    float_W = true;

    // Cambiar estado de la aplicación
    app_state = RUNNING;

    // Detener timer de actualización sinusoidal
    HAL_TIM_Base_Stop_IT(&htim4);

    // Resetear variables
    ready_for_running = false;
}

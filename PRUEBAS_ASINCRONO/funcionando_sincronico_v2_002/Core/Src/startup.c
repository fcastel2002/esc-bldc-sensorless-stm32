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

void generate_sine_tables(uint16_t pwm_arr, uint8_t direction)
{
    /* -----------------------------------------------------------------
     * 1. Calculamos una sola vez; si llamas varias veces, protégelo con
     *    un flag estático o muévelo a la inicialización.
     * -----------------------------------------------------------------*/
    for (uint16_t i = 0; i < SIN_TABLE_SIZE; i++)
    {
        /* Ángulo base 0°-360° (rad) */
        float angle = 2.0f * (float)M_PI * i / SIN_TABLE_SIZE;

        /* -------------------------------------------------------------
         * 2. Convertimos de rango [-1 … +1] a [0 … Q15_MAX]
         *    ( 0 = 0 % duty, 32 767 ≈ 100 % duty )
         *    Fórmula:  q15 = (sin + 1) * 0.5 * 32767
         * -------------------------------------------------------------*/
        uint16_t u = (uint16_t)((sinf(angle) * 0.5f + 0.5f) * Q15_MAX);
        uint16_t v = (uint16_t)((sinf(angle - 2.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);
        uint16_t w = (uint16_t)((sinf(angle - 4.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);

        /* 3. Sentido de giro: cambia el orden de las fases si direction=1 */
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

void foc_startup(void){
	static uint8_t is_first_startup = 1;
    sine_ctrl.timer_arr = initial_arr;
	if(is_first_startup){
		generate_sine_tables(TIM1->ARR, direction);
		is_first_startup = 0;
	}
    // 2. Configurar timer para actualización PWM
    TIM4->PSC =  35;
    TIM4->ARR = sine_ctrl.timer_arr;
//    TIM4->CCR2 = 50;           // Valor inicial del comparador

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
void executeTransition(void) {
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

/*
 * startup.c
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */


#include "startup.h"

volatile uint16_t zero_crossings = 0;

bool ready_for_update_pwm = true;
bool finished_foc_startup = false;


uint16_t sin_table_U[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_V[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_W[SIN_TABLE_SIZE] = {0};


// Versión optimizada para precisión y performance
// Versión optimizada para rendimiento
void generate_sine_tables(uint16_t max_pwm) {
    const float amplitude = 0.36f;
    const uint16_t min_pwm = (uint16_t)(0.05f * max_pwm);
    const float offset = amplitude + 0.05f;

    for(int i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = 2 * M_PI * i / SIN_TABLE_SIZE;

        // Cálculo directo para las tres fases
        uint16_t pwm_u = (uint16_t)((float)(sinf(angle) * amplitude + offset) * max_pwm);
        uint16_t pwm_v = (uint16_t)((float)(sinf(angle + 2*M_PI/3) * amplitude + offset) * max_pwm);
        uint16_t pwm_w = (uint16_t)((float)(sinf(angle + 4*M_PI/3) * amplitude + offset) * max_pwm);

        // Aplicar límites con operaciones eficientes
        sin_table_U[i] = pwm_u > max_pwm ? max_pwm : (pwm_u < min_pwm ? min_pwm : pwm_u);
        sin_table_V[i] = pwm_v > max_pwm ? max_pwm : (pwm_v < min_pwm ? min_pwm : pwm_v);
        sin_table_W[i] = pwm_w > max_pwm ? max_pwm : (pwm_w < min_pwm ? min_pwm : pwm_w);
    }
}
SineDriveController sine_ctrl = {
    .phase_step = 3,
    .modulation_index = 0.1f,  // Valor inicial bajo
    .timer_arr = 9999// Frecuencia inicial baja (~50 Hz)
};

//volatile uint32_t phase_counter = 0;
//float modulation_index = 0.4;
volatile uint8_t ccr2_tim4 = 15;
volatile bool ready_for_running = false;
volatile uint16_t pwm_val_U = 0;
volatile uint16_t pwm_val_V = 0;
volatile uint16_t pwm_val_W = 0;
/*
static volatile  uint16_t phase_step = 1;
volatile uint16_t iteration_counter = 0;

*/
volatile uint8_t iteration_index = 0;
static inline uint16_t clamp_pwm_step(uint16_t prev, uint16_t current, uint16_t max_step) {
    if(current > prev) {
        return (current - prev > max_step) ? prev + max_step : current;
    } else {
        return (prev - current > max_step) ? prev - max_step : current;
    }
}

void foc_startup(void){

	generate_sine_tables(TIM1->ARR);
    // 2. Configurar timer para actualización PWM
    TIM4->PSC =  5 -1;
    TIM4->ARR = sine_ctrl.timer_arr;
    TIM4->CCR2 = 2400;           // Valor inicial del comparador
	app_state = FOC_STARTUP;
	PWM_INIT();
	GPIOB->ODR |= EN_U;
	GPIOB->ODR |= EN_V;
	GPIOA->ODR |= EN_W;
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);

}

volatile uint16_t pwm_u =0;
volatile uint16_t pwm_v = 0;
volatile uint16_t pwm_w = 0;
void update_pwm_startup_foc() {
    // Actualización del contador de fase con protección contra overflow
    static uint16_t phase_counter = 0;
    static uint16_t it_count = 0;
    phase_counter += sine_ctrl.phase_step;
    if(phase_counter >= SIN_TABLE_SIZE) {
        phase_counter -= SIN_TABLE_SIZE;  // Más eficiente que módulo
    }

    it_count++;
    if(it_count == 600){
    	sine_ctrl.phase_step = 5;
    	TIM4->CCR2 =1600;
    }

    if(it_count >=1300){
    	executeTransition();
    	return;
    }

    // Cálculo de índices con desplazamiento de 120°
    uint16_t idx_u = phase_counter;
    uint16_t idx_v = (idx_u + SIN_TABLE_SIZE/3) % SIN_TABLE_SIZE;
    uint16_t idx_w = (idx_u + 2*SIN_TABLE_SIZE/3) % SIN_TABLE_SIZE;

    // Aplicación de modulación con suavizado
    static uint16_t prev_pwm[3] = {0};
    const uint8_t filter_coeff = 3;  // Factor de suavizado (1-5)

     pwm_u = (prev_pwm[0] * (filter_coeff-1) + sin_table_U[idx_u]) / filter_coeff;
     pwm_v = (prev_pwm[1] * (filter_coeff-1) + sin_table_V[idx_v]) / filter_coeff;
     pwm_w = (prev_pwm[2] * (filter_coeff-1) + sin_table_W[idx_w]) / filter_coeff;

    // Actualizar PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_w);

    // Guardar valores para siguiente iteración
    prev_pwm[0] = pwm_u;
    prev_pwm[1] = pwm_v;
    prev_pwm[2] = pwm_w;
    __HAL_TIM_SET_COUNTER(&htim4,0);
}


void executeTransition(void) {
    // Configurar para six-step
    pwmVal = max_pwm * 0.3f;  // 30% duty cycle inicial

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
    HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_2);

    // Resetear variables
    ready_for_running = false;
}

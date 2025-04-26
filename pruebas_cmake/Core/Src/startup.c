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
static uint16_t zero_sine_value = 0;

void generate_sine_tables(uint16_t max_pwm) {
    // Calculamos el valor medio (offset) que representa el "cero" de la sinusoide
    uint16_t pwm_offset = max_pwm / 2;

    // Amplitud reducida para evitar saturación en los extremos
    const float amplitude = 1.0f;

    for(int i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = 2 * M_PI * i / SIN_TABLE_SIZE;

        // Valores sinusoidales centrados en pwm_offset
        // Convertimos explícitamente a uint16_t después de calcular el valor
        float sin_val_u = sinf(angle) * amplitude * pwm_offset;
        float sin_val_v = sinf(angle + 2*M_PI/3) * amplitude * pwm_offset;
        float sin_val_w = sinf(angle + 4*M_PI/3) * amplitude * pwm_offset;

        // Aseguramos que los valores estén dentro del rango válido para uint16_t
        sin_table_U[i] = (uint16_t)(pwm_offset + sin_val_u);
        sin_table_V[i] = (uint16_t)(pwm_offset + sin_val_v);
        sin_table_W[i] = (uint16_t)(pwm_offset + sin_val_w);
    }

    // Guardamos el valor de offset para posibles referencias futuras
    zero_sine_value = pwm_offset;
}
SineDriveController sine_ctrl = {
    .phase_step = 2,
    .modulation_index = 0.2f,  // Valor inicial bajo
    .timer_arr =8000// Frecuencia inicial baja (~50 Hz)
};

void foc_startup(void){
	uint8_t is_first_startup = 1;

	if(is_first_startup){
		generate_sine_tables(TIM1->ARR);
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
	HAL_TIM_Base_Start_IT(&htim4);
}

volatile uint16_t pwm_u =0;
volatile uint16_t pwm_v = 0;
volatile uint16_t pwm_w = 0;
volatile uint16_t sine_u = 0;
volatile uint16_t sine_v = 0;
volatile uint16_t sine_w = 0;

const uint16_t acceleration_time_ref = 140;
void update_pwm_startup_foc() {
    // Actualización del contador de fase con protección contra overflow
    static uint16_t phase_counter = 0;
    static uint16_t it_count = 0;

    phase_counter += sine_ctrl.phase_step;
    if(phase_counter >= SIN_TABLE_SIZE) {
        phase_counter -= SIN_TABLE_SIZE;  // Más eficiente que módulo
    }

    // Calcular la frecuencia relativa (valor menor de ARR = frecuencia mayor)
    TIM4->ARR = sine_ctrl.timer_arr;
    if(sine_ctrl.timer_arr > 2800){
    	sine_ctrl.timer_arr -= 5;
    }
    else {
    	sine_ctrl.phase_step = 3;
    	it_count++;
    	if(it_count == 1500){
    		phase_counter = 0;
    		it_count =0;
    		sine_ctrl.timer_arr = 8000;
    		sine_ctrl.modulation_index = 0.2f;
    		executeTransition();
    	}
    }
    if(sine_ctrl.modulation_index <=0.98){
    	sine_ctrl.modulation_index += 0.0008;
    }

    // Cálculo de índices con despla  zamiento de 120°
    uint16_t idx_u = phase_counter;
    uint16_t idx_v = (idx_u + (SIN_TABLE_SIZE)/3) % SIN_TABLE_SIZE;
    uint16_t idx_w = (idx_u + 2*(SIN_TABLE_SIZE)/3) % SIN_TABLE_SIZE;
    sine_u = (uint16_t)(sin_table_U[idx_u] * sine_ctrl.modulation_index);
    sine_v = (uint16_t)(sin_table_V[idx_v] * sine_ctrl.modulation_index);
    sine_w = (uint16_t)(sin_table_W[idx_w] * sine_ctrl.modulation_index);


    // Actualizar PWM con los duty cycles calculados
    __HAL_TIM_SET_COMPARE(&htim1, IN_U, sine_u);
    __HAL_TIM_SET_COMPARE(&htim1, IN_V,	sine_v);
    __HAL_TIM_SET_COMPARE(&htim1, IN_W, sine_w);

}
volatile bool ready_for_running = false;
void executeTransition(void) {
    // Configurar para six-step
    pwmVal = max_pwm * 0.25f;  // 30% duty cycle inicial
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

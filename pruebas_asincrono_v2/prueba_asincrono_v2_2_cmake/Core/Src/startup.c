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

void generate_sine_tables(uint16_t max_pwm, uint8_t direction) {
    // Calculamos el valor medio (offset) que representa el "cero" de la sinusoide
    // Este será el punto central del ciclo de trabajo.
    uint16_t pwm_offset = max_pwm / 2;

    // Amplitud relativa al offset. 1.0f usa el rango completo desde 0 hasta max_pwm.
    // Podrías reducirla si necesitas margen (ej. 0.95f).
    const float amplitude_factor = 1.0f;
    float scaled_amplitude = amplitude_factor * pwm_offset;

    // Definimos los desfases para las fases V y W según la dirección deseada
    float phase_shift_v;
    float phase_shift_w;

    if (direction == 1) {
        // Dirección "adelante": Secuencia U -> V -> W
        // V está desfasada +120 grados (2*PI/3) respecto a U
        // W está desfasada +240 grados (4*PI/3) respecto a U
        phase_shift_v = 2.0f * M_PI / 3.0f; // +120 grados
        phase_shift_w = 4.0f * M_PI / 3.0f; // +240 grados
    } else {
        // Dirección "atrás": Secuencia U -> W -> V (intercambiamos V y W)
        // W está desfasada +120 grados (2*PI/3) respecto a U
        // V está desfasada +240 grados (4*PI/3) respecto a U
        phase_shift_v = 4.0f * M_PI / 3.0f; // +240 grados (era W)
        phase_shift_w = 2.0f * M_PI / 3.0f; // +120 grados (era V)
    }

    for(int i = 0; i < SIN_TABLE_SIZE; i++) {
        // Ángulo actual en radianes para la fase U (0 a 2*PI)
        float angle = 2.0f * M_PI * (float)i / (float)SIN_TABLE_SIZE;

        // Calculamos los valores sinusoidales base (centrados en 0)
        // Usamos sinf para la versión float de sin()
        float sin_val_u_base = sinf(angle)                 * scaled_amplitude;
        float sin_val_v_base = sinf(angle + phase_shift_v) * scaled_amplitude;
        float sin_val_w_base = sinf(angle + phase_shift_w) * scaled_amplitude;

        // Aplicamos el offset para centrar la sinusoide en pwm_offset
        // y convertimos a uint16_t. El casting truncará decimales.
        // Podrías usar roundf() antes de castear si prefieres redondeo al entero más cercano:
        // sin_table_U[i] = (uint16_t)roundf(pwm_offset + sin_val_u_base);
        sin_table_U[i] = (uint16_t)(pwm_offset + sin_val_u_base);
        sin_table_V[i] = (uint16_t)(pwm_offset + sin_val_v_base);
        sin_table_W[i] = (uint16_t)(pwm_offset + sin_val_w_base);


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
	//////////////// PRUEBAS SIN ARRANQUE ///////////////

	/////////////////////////////////////////////////////
	uint8_t is_first_startup = 1;

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
	if(app_state != FOC_STARTUP) return;
    // Actualización del contador de fase con protección contra overflow
    static uint16_t phase_counter = 0;
    static uint16_t it_count = 0;

    phase_counter += sine_ctrl.phase_step;
    if(phase_counter >= SIN_TABLE_SIZE) {
        phase_counter -= SIN_TABLE_SIZE;  // Más eficiente que módulo
    }

    // Calcular la frecuencia relativa (valor menor de ARR = frecuencia mayor)
    TIM4->ARR = sine_ctrl.timer_arr;
    if(sine_ctrl.timer_arr > 2500){
    	sine_ctrl.timer_arr -= 8;
    }
    else {
    	sine_ctrl.phase_step = 3;
    	it_count++;
    	if(it_count == 2000){
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
//    uint16_t idx_u = phase_counter;
//    uint16_t idx_v = (idx_u + (SIN_TABLE_SIZE)/3) % SIN_TABLE_SIZE;
//    uint16_t idx_w = (idx_u + 2*(SIN_TABLE_SIZE)/3) % SIN_TABLE_SIZE;'
    uint16_t idx_u = phase_counter;
    uint16_t idx_v = phase_counter;
    uint16_t idx_w = phase_counter;
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
    pwmVal = max_pwm * 0.52f;  // 30% duty cycle inicial
    // Desactivar salidas temporalmente
    GPIOB->ODR &= ~EN_U;
    GPIOB->ODR &= ~EN_V;
    GPIOA->ODR &= ~EN_W;

    // Detener PWM sinusoidal
    PWM_STOP();

    // Inicializar configuración six-step

    // Configurar estado de las fases
    float_U = true;
    float_V = true;
    float_W = true;
    TIM3->CCR1 = filtered_speed*1.9f;
    TIM3->CNT = 0;
    // Cambiar estado de la aplicación
    app_state =	ZC_SEARCH;


    // Detener timer de actualización sinusoidal
    HAL_TIM_Base_Stop_IT(&htim4);

    // Resetear variables
    ready_for_running = false;
}

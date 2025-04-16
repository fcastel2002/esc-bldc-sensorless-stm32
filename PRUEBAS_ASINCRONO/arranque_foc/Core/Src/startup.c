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


int16_t sin_table_U[SIN_TABLE_SIZE] = {0};
int16_t sin_table_V[SIN_TABLE_SIZE] = {0};
int16_t sin_table_W[SIN_TABLE_SIZE] = {0};
static uint16_t zero_sine_value = 0;

// Versión optimizada para precisión y performance
// Versión optimizada para rendimiento
void generate_sine_tables(uint16_t max_pwm) {
    // Generamos tablas de seno puro sin offset (valores de -1 a +1 escalados)
    // Esto es conceptualmente más claro y evita confusiones en el procesamiento

    const float amplitude = 0.95f;  // Amplitud máxima para evitar saturación

    for(int i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = 2 * M_PI * i / SIN_TABLE_SIZE;

        // Valores sinusoidales puros (-amplitude a +amplitude)
        sin_table_U[i] = (int16_t)(sinf(angle) * amplitude * max_pwm);
        sin_table_V[i] = (int16_t)(sinf(angle + 2*M_PI/3) * amplitude * max_pwm);
        sin_table_W[i] = (int16_t)(sinf(angle + 4*M_PI/3) * amplitude * max_pwm);
    }

    // Ya no necesitamos zero_sine_value porque ahora el cero real es 0
    zero_sine_value = 0;
}
SineDriveController sine_ctrl = {
    .phase_step = 1,
    .modulation_index = 0.1f,  // Valor inicial bajo
    .timer_arr =1300// Frecuencia inicial baja (~50 Hz)
};

void foc_startup(void){

	generate_sine_tables(TIM1->ARR);
    // 2. Configurar timer para actualización PWM
    TIM4->PSC =  35;
    TIM4->ARR = sine_ctrl.timer_arr;
//    TIM4->CCR2 = 50;           // Valor inicial del comparador
    sine_ctrl.modulation_index = 0.5f;

	app_state = FOC_STARTUP;
	PWM_INIT();
	GPIOB->ODR |= IN_U;
	GPIOB->ODR |= IN_V;
	GPIOA->ODR |= IN_W;
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	TIM3->CNT = 800;
	HAL_TIM_Base_Start_IT(&htim4);
}

volatile uint16_t pwm_u =0;
volatile uint16_t pwm_v = 0;
volatile uint16_t pwm_w = 0;
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
    uint16_t initial_arr = sine_ctrl.timer_arr; // Valor inicial de sine_ctrl.timer_arr
    uint16_t current_arr = TIM4->ARR;
    float frequency_ratio = (float)current_arr / initial_arr;

    // Ajustar el índice de modulación basado en la frecuencia
    // Relación V/f lineal simplificada con término de boost para bajas velocidades
   // Término de boost para baja velocidad

    // Relación V/f modificada con boost para bajas velocidades

    TIM4->ARR = TIM3->CNT;
//    // Aceleración más gradual para mejor enganche a baja velocidad
//    if(it_count <= acceleration_time_ref) {
//        // Rampa más lenta al inicio
//        if(it_count < acceleration_time_ref * 0.5f) {
//            if(TIM4->ARR >= 700) TIM4->ARR -= 2;
//        }
//        // Rampa más pronunciada después
//        else {
//            if(TIM4->ARR >= 400) TIM4->ARR -= 1;
//        }
//        it_count++;
//    }
//
//    // Ajuste de phase_step en punto específico de la aceleración
//    if(it_count == 0.8f * acceleration_time_ref) sine_ctrl.phase_step = 2;

    // Cálculo de índices con desplazamiento de 120°
    uint16_t idx_u = phase_counter;
    uint16_t idx_v = (idx_u + SIN_TABLE_SIZE/3) % SIN_TABLE_SIZE;
    uint16_t idx_w = (idx_u + 2*SIN_TABLE_SIZE/3) % SIN_TABLE_SIZE;

    // Aplicación de modulación con suavizado
    static uint16_t prev_pwm[3] = {0};

    // Obtener valores sinusoidales y aplicar modulación
    int16_t sine_u = (int16_t)(sin_table_U[idx_u] * sine_ctrl.modulation_index);
    int16_t sine_v = (int16_t)(sin_table_V[idx_v] * sine_ctrl.modulation_index);
    int16_t sine_w = (int16_t)(sin_table_W[idx_w] * sine_ctrl.modulation_index);

    // Determinar dirección (high-side o low-side) basado en el signo
    bool dir_u = sine_u >= 0;
    bool dir_v = sine_v >= 0;
    bool dir_w = sine_w >= 0;

    // Calcular duty cycle basado en el valor absoluto
    // Usar valor absoluto para que el duty cycle sea el mismo independientemente de la dirección
    uint16_t duty_u = abs(sine_u);
    uint16_t duty_v = abs(sine_v);
    uint16_t duty_w = abs(sine_w);


    // Configurar dirección en los pines GPIO
    // Esto determina si se activa high-side o low-side
    GPIOB->ODR = (GPIOB->ODR & ~IN_U) | (dir_u ? IN_U : 0);
    GPIOB->ODR = (GPIOB->ODR & ~IN_V) | (dir_v ? IN_V : 0);
    GPIOA->ODR = (GPIOA->ODR & ~IN_W) | (dir_w ? IN_W : 0);

    // Actualizar PWM con los duty cycles calculados
    __HAL_TIM_SET_COMPARE(&htim1, EN_U, duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, EN_V, duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, EN_W, duty_w);

    // Guardar valores para siguiente iteración (filtro)
    prev_pwm[0] = duty_u;
    prev_pwm[1] = duty_v;
    prev_pwm[2] = duty_w;

    // Almacenar para debug o monitoreo
    pwm_u = duty_u;
    pwm_v = duty_v;
    pwm_w = duty_w;
}
volatile bool ready_for_running = false;
void executeTransition(void) {
    // Configurar para six-step
    pwmVal = max_pwm * 0.3f;  // 30% duty cycle inicial

    // Desactivar salidas temporalmente
    GPIOB->ODR &= ~IN_U;
    GPIOB->ODR &= ~IN_V;
    GPIOA->ODR &= ~IN_W;

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

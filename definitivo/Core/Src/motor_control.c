/*
 * motor_control.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */


#include "main.h"
#include <stdlib.h>
#include "motor_control.h"
#define KP 0.75f
#define KI 1.35f //real Ki = KI * 2/SCALE
#define SCALE 1
#define dt 0.002


/*
 *@brief: Macros para la medicion y mapeo de velocidad
 * unidades: PERIODO.
 * TIMER CLOCK: 72MHz
 * PSC: 400 (399 + 1)
 * ARR: 0xFFFF
 *
 * VELOCIDAD MINIMA RPM: 380
 * VELOCIDAD MAXIMA RPM: 12000
 *
 */

#define SPEED_MAX 200	//pwm
#define SPEED_MIN 14000
#define SPEED_RANGE (SPEED_MIN - SPEED_MAX)
////////////

// limites PWM


//STALL HANDLING
#define TIMEOUT_MOTOR_STALL_MS 200
#define STALL_CHECK_TIME_MS 25
////////

#define  ZCP_TO_CHECK 4
#define SPEED_TOLERANCE_PCT 25



// MANEJO DE CONMUTACION
	// Variables
volatile uint16_t pwmVal = 0;
volatile int8_t commutationStep = 0;
bool float_W = false;
bool float_U = false;
bool float_V = false;
	//Funciones
static void commutation(int8_t step);


// CONFIGURACION DE MOTOR
	// Variables
volatile uint8_t motor_control_config_done = 0;
volatile uint16_t max_pwm = 0;
volatile bool motor_stalled = false;
	// Funciones
	

// VARIABLES PARA CONTROL PI DE VELOCIDAD
volatile uint16_t speed_setpoint =  0;
volatile uint16_t speed_setpoint_rpm = 0; // RPM
static volatile int32_t speed_prev_error = 0;
static volatile uint16_t max_limit_pwm = 0;
static volatile uint16_t min_limit_pwm = 0;
static volatile float pwm_speed_range_relation = 0.0f;
static volatile uint16_t speed_measure;
static volatile int32_t speed_error;
static volatile int32_t speed_output = 0;
static volatile float speed_integral = 0;
static volatile float speed_proportional;
static volatile float max_speed_integral = 0;
static volatile float min_speed_integral = 0;
//====================================================


// VARIABLES PARA MANEJO DE VELOCIDAD
volatile int32_t diff_speed = 0;
volatile uint8_t direction = 0;
//====================================================

// VARIABLES PARA CHEQUEO DE CALIDAD DEL CRUCE POR CERO
static uint16_t last_W_timestamp = 0;
static uint16_t W_periods[ZCP_TO_CHECK];
static uint8_t W_period_idx = 0;
static uint8_t valid_W_zcp = 0;
volatile uint8_t consistent_zero_crossing = 0; //flag
//====================================================

// VARIABLES PARA FILTRO DIGITAL DE VELOCIDAD INPUT CAPTURE
static volatile uint8_t speed_buffer_size = 1;
static volatile uint16_t speed_buffer[2] = {0};
volatile uint16_t filtered_speed = 0;
static volatile uint16_t last_speed_capture = 1;
//====================================================

// VARIABLES PARA MEDICION DE VELOCIDAD DE CONSENSO TRIFASICO
phase_measurement_t phase_measurements[PHASE_COUNT] = {0}; 
volatile uint16_t consensus_speed = 0;
volatile uint8_t active_phases_count = 0;
volatile uint8_t speed_measurement_ready = 0;
//====================================================


// FUNCIONES PARA MEDICION DE VELOCIDAD

static void processPhaseMeasurement(uint8_t phase_idx, uint16_t current_timestamp);
static void calculateConsensusSpeed(void);

int8_t safeMod(int8_t value, int8_t mod) {
    return (value % mod + mod) % mod;
}

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
 * @warning Esta función modifica directamente registros GPIO y TIM para máxima velocidad
 * @warning Debe llamarse con un paso válido para evitar estados indefinidos del motor
 */
static void commutation(int8_t step) {
	switch(step) {

	case POS_UV:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		float_W = true;
		float_V = false;
		float_U = false;

		GPIOA->ODR &= ~EN_W;
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;
	case POS_UW:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		float_W = false;
		float_U = false;
		float_V = true;
		GPIOB->ODR &= ~EN_V;
		GPIOB->ODR |= EN_U;
		GPIOA->ODR |=  EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
		break;
	case POS_VW:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

		float_W = false;
		float_V = false;
		float_U = true;
		GPIOB->ODR &= ~EN_U;

		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);

		break;
	case POS_VU:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

		float_W = true;
		float_V = false;
		float_U = false;
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |=  EN_V;
		GPIOA->ODR &= ~EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
		break;
	case POS_WU:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);

		float_W = false;
		float_U = false;
		float_V = true;
		GPIOB->ODR &= ~EN_V;
		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_U;
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, 0);
		break;
	case POS_WV:
		PWM_STOP();
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

		float_W = false;
		float_V = false;
		float_U = true;
		GPIOB->ODR &= ~EN_U;
		GPIOA->ODR |=  EN_W;
		GPIOB->ODR |= EN_V;
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;

	case POS_INIT:
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		GPIOA->ODR |= EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, 0);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, 0);
		break;
	case POS_SOUND:
		GPIOB->ODR |= EN_U;
		GPIOB->ODR |= EN_V;
		GPIOA->ODR |= EN_W;
		__HAL_TIM_SET_COMPARE(&htim1, IN_U, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_W, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim1, IN_V, pwmVal);
		break;
	default:
		break;
	}
}



void updateAllMotorControl(){
	max_limit_pwm = TIM1 -> ARR;
	max_pwm = max_limit_pwm;
	min_limit_pwm = max_limit_pwm * 0.05f;
	pwm_speed_range_relation = (float)(max_limit_pwm - min_limit_pwm)/(float)SPEED_RANGE;
	speed_setpoint_rpm = 1000;
	motor_control_config_done = 1;


}
uint8_t motor_pole_pairs = 2;
#define TIMER_CLOCK 72000000 // 72 MHz
uint16_t convertSpeedValue(uint16_t value, bool to_ticks)
{
    if (to_ticks) {
        // Conversión RPM -> setpoint (valor para speed_setpoint)
        if (value == 0) return 0;
        
        // Calcular el período entre cruces por cero (en segundos)
        double rpm_mechanical = (double)value;
        double rpm_electrical = rpm_mechanical * motor_pole_pairs;
        double frequency = rpm_electrical / 60.0;  // Hz eléctricos
        double period = 1.0 / frequency;           // Período eléctrico total (s)
        double zc_period = period / 6.0;           // Período entre cruces por cero (s)
        
        // Convertir a ticks del timer de captura (TIM2)
        double timer_ticks = zc_period * (TIMER_CLOCK / (TIM2->PSC + 1));
        
        // Mapear al rango de setpoint usando la misma lógica que mapSpeed()
        if (timer_ticks > SPEED_MIN) timer_ticks = SPEED_MIN;
        else if (timer_ticks < SPEED_MAX) timer_ticks = SPEED_MAX;
        
        uint16_t setpoint = (uint16_t)((SPEED_MIN - timer_ticks) * pwm_speed_range_relation + min_limit_pwm);
        
        return setpoint;
    } else {
        // Conversión setpoint -> RPM
        if (value == 0) return 0;
        
        // Revertir el mapeo de mapSpeed()
        double timer_ticks = SPEED_MIN - ((double)value - min_limit_pwm) / pwm_speed_range_relation;
        
        // Convertir ticks a tiempo real
        double zc_period = timer_ticks / (TIMER_CLOCK / (TIM2->PSC + 1));
        double period = zc_period * 6.0;           // Período eléctrico total (s)
        double frequency = 1.0 / period;           // Hz eléctricos
        
        // Convertir a RPM mecánicos
        double rpm_electrical = frequency * 60.0;
        double rpm_mechanical = rpm_electrical / motor_pole_pairs;
        
        // Limitar a rango operativo del motor
        if (rpm_mechanical < 100) return 100;
        if (rpm_mechanical > 7000) return 7000;
        return (uint16_t)rpm_mechanical;
    }
}
uint16_t filtroMediaMovil(uint16_t measurement){
	int32_t new_speed = 0;
	static uint16_t prev_speed = 0;
	static volatile uint8_t speed_index = 0;
	if(measurement < SPEED_MAX || measurement > SPEED_MIN){
		if(prev_speed >= SPEED_MAX && prev_speed <= SPEED_MIN ){

		speed_buffer[speed_index] = prev_speed;
		
		}else{
			speed_buffer[speed_index] = (SPEED_MAX - SPEED_MIN) / 2; // Valor por defecto si la medición es inválida
		
		}
	
	}else{
		speed_buffer[speed_index] = measurement;
		prev_speed = measurement;
	}
	speed_index  = (speed_index + 1) % speed_buffer_size;

	for (int i = 0; i < speed_buffer_size; i++) {
		new_speed += speed_buffer[i];
	}
	new_speed = new_speed/speed_buffer_size;


	return (uint16_t)new_speed;
}

void zeroCrossing(uint8_t fase){
	static uint8_t speed_calc_counter = 0;

	switch(fase){
	case 1:  // Fase W
		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if (direction == 0) {
				commutationStep = safeMod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safeMod(commutationStep - 1, NUM_POS);
			}
			last_zc_timestamp = HAL_GetTick();
			commutation(commutationStep);
		}
		
		// Procesar medición de velocidad para fase W
		if(app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP){
			uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			processPhaseMeasurement(0, current_timestamp);  // Fase W = índice 0
			
			if(last_W_timestamp != 0){
				uint16_t period;
				if(last_W_timestamp > current_timestamp){
				    period = (0xFFFF - last_W_timestamp) + current_timestamp + 1;
				}
				else{
				    period = current_timestamp - last_W_timestamp;
				}
				W_periods[W_period_idx] = period;
				W_period_idx = (W_period_idx + 1) % ZCP_TO_CHECK;
				if(valid_W_zcp < ZCP_TO_CHECK){
					valid_W_zcp++;
				}
				if(valid_W_zcp == ZCP_TO_CHECK){
					uint16_t avg_period = 0;
					uint8_t is_consistent = 1;
					for(uint8_t i = 0; i< ZCP_TO_CHECK;i++){
						avg_period += W_periods[i];
					}
					avg_period /= ZCP_TO_CHECK;
					if(avg_period > 20){
						for(uint8_t i = 0; i < ZCP_TO_CHECK; i++){
							uint32_t tolerance = avg_period * SPEED_TOLERANCE_PCT / 100;
							if(abs(W_periods[i] - avg_period)> tolerance){
								is_consistent = 0;
								break;
							}
						}
						if(is_consistent){
							valid_W_zcp =0;
							consistent_zero_crossing = 1;
							speed_buffer_size = 1;
						}
						else{
							consistent_zero_crossing = 0;
						}
					}
					valid_W_zcp = 0;
				}
			}
			last_W_timestamp = current_timestamp;
		}
		
		// Actualizar velocidad usando consenso tri-fase cada cierto número de ZC
		if(app_state == CLOSEDLOOP){
		    speed_calc_counter++;
		    if(speed_calc_counter >= 2) {  // Cada 2 zero-crossings
		        calculateConsensusSpeed();
		        
		        // Usar velocidad por consenso si está disponible, sino usar método original
		        if(speed_measurement_ready && consensus_speed > 0) {
		            diff_speed = consensus_speed;
		            filtered_speed = filtroMediaMovil(diff_speed);
		        } else {
		            // Fallback al método original
		            uint16_t current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		            if(last_speed_capture != 0) {
		                uint16_t speed_period;
		                if(last_speed_capture > current_capture) {
		                    speed_period = (0xFFFF - last_speed_capture) + current_capture + 1;
		                } else {
		                    speed_period = current_capture - last_speed_capture;
		                }
		                diff_speed = speed_period;
		                filtered_speed = filtroMediaMovil(diff_speed);
		            }
		            last_speed_capture = current_capture;
		        }
		        speed_calc_counter = 0;
		    }
		}
		break;
		
	case 2:  // Fase V
		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if (direction == 0) {
				commutationStep = safeMod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safeMod(commutationStep - 1, NUM_POS);
			}
			commutation(commutationStep);
		}
		
		// Procesar medición de velocidad para fase V
		if(app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP){
			uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			processPhaseMeasurement(1, current_timestamp);  // Fase V = índice 1
		}
		break;
		
	case 3:  // Fase U
		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if (direction == 0) {
				commutationStep = safeMod(commutationStep + 1, NUM_POS);
			} else {
				commutationStep = safeMod(commutationStep - 1, NUM_POS);
			}
			commutation(commutationStep);
		}
		
		// Procesar medición de velocidad para fase U
		if(app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP){
			uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			processPhaseMeasurement(2, current_timestamp);  // Fase U = índice 2
		}
		break;
	}
}

void motorDetection(){
	TIM1 -> PSC = 7;
	uint16_t arr_pwm = 0;
	arr_pwm = 1000;
	TIM1 -> ARR = arr_pwm;
	pwmVal = arr_pwm * 0.3f;
	uint8_t step = POS_UV;
	for(int k = 0; k <3; k++){
		PWM_STOP();
		HAL_Delay(60);
		PWM_INIT();
		for (int i = 0; i < 140; i++){
			commutation(step);
			step = safeMod(step + 1, NUM_POS);

			HAL_Delay(1);
		}

	}
	PWM_STOP();

	pwmVal = 0;
}

void checkMotorStatus(){
	static uint32_t last_check_time = 0;
	static uint8_t stall_counter = 0;
	static uint8_t running_counter = 0;

	uint32_t current_time = HAL_GetTick();
	if(current_time - last_check_time >= STALL_CHECK_TIME_MS){
		last_check_time = current_time;

		if(app_state == RUNNING || app_state == CLOSEDLOOP){
			if(last_zc_timestamp > 0 && ((current_time - last_zc_timestamp)> TIMEOUT_MOTOR_STALL_MS)){
				stall_counter ++;
				running_counter = 0;
				if(stall_counter >= 3){
					motor_stalled = true;
				}

			}else{
				running_counter++;
				stall_counter = 0;
				if(running_counter >= 2 && motor_stalled){
					motor_stalled = false;
					running_counter = 0;
				}
			}
		}else{
			running_counter = 0;
			stall_counter = 0;

		}
	}
}

static inline uint16_t mapSpeed(uint16_t raw_speed){
	if(raw_speed == 0){
		raw_speed = SPEED_MIN;
	}


	if (raw_speed > SPEED_MIN) {
		raw_speed = SPEED_MIN;
	}
	else if (raw_speed <SPEED_MAX) {
		raw_speed = SPEED_MAX;
	}
	uint16_t mapped_speed  = (SPEED_MIN - raw_speed) * pwm_speed_range_relation + min_limit_pwm;
	return mapped_speed;

}

/**
 * @brief Controlador PI (Proporcional-Integral) para regulación de velocidad del motor
 *
 * Esta función implementa un controlador PI discreto para mantener la velocidad del motor
 * en el valor de referencia (speed_setpoint). El controlador calcula la salida PWM
 * necesaria basándose en el error entre la velocidad deseada y la velocidad medida.
 *
 * @details Algoritmo del controlador PI:
 * 1. Mapea la velocidad filtrada a unidades PWM usando mapSpeed()
 * 2. Calcula el error: error = setpoint - velocidad_medida
 * 3. Término proporcional: P = KP * error / SCALE
 * 4. Término integral: I += KI * (error + error_previo) * dt
 * 5. Anti-windup del integrador: limita la integral según los límites PWM
 * 6. Salida final: PWM = P + I (con saturación en límites PWM)
 *
 * @note Parámetros del controlador:
 * - KP: Ganancia proporcional (0.75f)
 * - KI: Ganancia integral (1.35f)
 * - dt: Período de muestreo (0.002s = 2ms)
 * - SCALE: Factor de escalado (1)
 *
 * @note Variables globales utilizadas:
 * - speed_setpoint: Velocidad deseada en unidades PWM
 * - filtered_speed: Velocidad medida y filtrada en unidades de período
 * - pwmVal: Salida PWM calculada para el motor
 * - speed_integral: Estado del integrador PI
 * - max_limit_pwm, min_limit_pwm: Límites de saturación PWM
 *
 * @details Anti-windup implementado:
 * - Calcula límites dinámicos para el integrador basados en la salida proporcional
 * - max_speed_integral = max_limit_pwm - speed_proportional
 * - min_speed_integral = min_limit_pwm - speed_proportional
 * - Previene saturación del integrador cuando la salida alcanza los límites PWM
 * - Mejora el tiempo de respuesta transitorio del controlador
 *
 * @note Solo opera cuando app_state == CLOSEDLOOP
 * @warning No utiliza protección de interrupciones, debe ser llamada desde contexto seguro
 */
void PIcontrol(){

	//speed_measure =  mapSpeed(filtered_speed);
	speed_measure = periodToPwm(filtered_speed);
	if(app_state == CLOSEDLOOP){
		uint16_t target_period = rpmToPeriod(speed_setpoint_rpm);
        uint16_t target_pwm = periodToPwm(target_period);
		//speed_error = speed_setpoint - speed_measure;
		speed_error = target_pwm - speed_measure;
		speed_proportional = (KP * speed_error)/SCALE;
		speed_integral += KI * (speed_error+speed_prev_error)*dt;
		speed_prev_error = speed_error;
		if(max_limit_pwm > speed_proportional){
			max_speed_integral = (max_limit_pwm - speed_proportional);
		}
		else{
			max_speed_integral = 0;
		}
		if (min_limit_pwm < speed_proportional) {
			min_speed_integral = (min_limit_pwm - speed_proportional);
		} else {
			min_speed_integral = 0;
		}
		if (speed_integral > max_speed_integral) {
			speed_integral = max_speed_integral;
		}
		else if (speed_integral < min_speed_integral) {
			speed_integral = min_speed_integral;
		}
		speed_output = speed_proportional + speed_integral;
		if(speed_output < min_limit_pwm) speed_output = min_limit_pwm;
		if(speed_output > max_limit_pwm) speed_output = max_limit_pwm;
		pwmVal = (uint16_t)speed_output ;
	}
}

static inline void pwm_break(void){
	pwmVal = 0;
	GPIOB->ODR 	&= ~EN_U;
	GPIOB->ODR |= EN_V;
	GPIOA->ODR |= EN_W;
	__HAL_TIM_SET_COMPARE(&htim1, IN_U, max_pwm);
	__HAL_TIM_SET_COMPARE(&htim1, IN_V, max_pwm);
	__HAL_TIM_SET_COMPARE(&htim1, IN_W, max_pwm);

}
void stopMotor(uint8_t mode){
	filtered_speed = 0;
	switch(mode){
		case 0:
		//gradual stop
			PWM_STOP();
			app_state = IDLE;
			break;
		case 1:
		//emergency stop
			app_state = STOPPED;

			direction = !direction;
			pwmVal = max_pwm;
			uint32_t stop_start_time = HAL_GetTick();
			commutation(POS_UV);
			while (HAL_GetTick() - stop_start_time < 5000);
			PWM_STOP();
			direction = !direction;	
			pwmVal = 0;
			app_state = IDLE;
			break;
	}
}

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
static void processPhaseMeasurement(uint8_t phase_idx, uint16_t current_timestamp) {
    phase_measurement_t* phase = &phase_measurements[phase_idx];
    
    if(phase->last_timestamp != 0) {
        uint16_t period;
        if(phase->last_timestamp > current_timestamp) {
            period = (0xFFFF - phase->last_timestamp) + current_timestamp + 1;
        } else {
            period = current_timestamp - phase->last_timestamp;
        }
        
        // Filtrar períodos muy pequeños o muy grandes (ruido)
        if(period > 50 && period < 50000) {
            phase->periods[phase->period_idx] = period;
            phase->period_idx = (phase->period_idx + 1) % ZCP_BUFFER_SIZE;
            
            if(phase->valid_periods < ZCP_BUFFER_SIZE) {
                phase->valid_periods++;
            }
            
            // Calcular promedio y consistencia si tenemos suficientes muestras
            if(phase->valid_periods >= ZCP_BUFFER_SIZE) {
                uint32_t sum = 0;
                for(uint8_t i = 0; i < ZCP_BUFFER_SIZE; i++) {
                    sum += phase->periods[i];
                }
                phase->avg_period = sum / ZCP_BUFFER_SIZE;
                
                // Verificar consistencia
                phase->is_consistent = 1;
                for(uint8_t i = 0; i < ZCP_BUFFER_SIZE; i++) {
                    uint32_t tolerance = phase->avg_period * SPEED_TOLERANCE_PCT / 100;
                    if(abs(phase->periods[i] - phase->avg_period) > tolerance) {
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
static void calculateConsensusSpeed(void) {
    uint16_t valid_speeds[PHASE_COUNT];
    uint8_t valid_count = 0;
    
    // Recopilar velocidades válidas de cada fase
    for(uint8_t i = 0; i < PHASE_COUNT; i++) {
        if(phase_measurements[i].is_consistent && phase_measurements[i].valid_periods >= ZCP_BUFFER_SIZE) {
            valid_speeds[valid_count] = phase_measurements[i].avg_period;
            valid_count++;
        }
    }
    
    active_phases_count = valid_count;
    
    if(valid_count >= 2) {  // Necesitamos al menos 2 fases para consenso
        uint32_t sum = 0;
        uint8_t consensus_count = 0;
        
        // Calcular promedio inicial
        for(uint8_t i = 0; i < valid_count; i++) {
            sum += valid_speeds[i];
        }
        uint16_t avg_speed = sum / valid_count;
        
        // Verificar consenso entre fases
        sum = 0;
        for(uint8_t i = 0; i < valid_count; i++) {
            uint32_t tolerance = avg_speed * SPEED_CONSENSUS_THRESHOLD / 100;
            if(abs(valid_speeds[i] - avg_speed) <= tolerance) {
                sum += valid_speeds[i];
                consensus_count++;
            }
        }
        
        if(consensus_count >= 2) {
            consensus_speed = sum / consensus_count;
            speed_measurement_ready = 1;
        } else {
            speed_measurement_ready = 0;
        }
    } else if(valid_count == 1) {
        // Si solo tenemos una fase válida, la usamos con precaución
        consensus_speed = valid_speeds[0];
        speed_measurement_ready = 1;
    } else {
        speed_measurement_ready = 0;
    }
}

// Función para reiniciar mediciones tri-fase
void reset_phase_measurements(void) {
    for(uint8_t i = 0; i < PHASE_COUNT; i++) {
        phase_measurements[i].last_timestamp = 0;
        phase_measurements[i].period_idx = 0;
        phase_measurements[i].valid_periods = 0;
        phase_measurements[i].avg_period = 0;
        phase_measurements[i].is_consistent = 0;
        for(uint8_t j = 0; j < ZCP_BUFFER_SIZE; j++) {
            phase_measurements[i].periods[j] = 0;
        }
    }
    consensus_speed = 0;
    active_phases_count = 0;
    speed_measurement_ready = 0;
}

// Función de diagnóstico para monitorear estado de mediciones
void get_phase_diagnostics(uint16_t* phase_speeds, uint8_t* phase_status) {
    for(uint8_t i = 0; i < PHASE_COUNT; i++) {
        phase_speeds[i] = phase_measurements[i].avg_period;
        phase_status[i] = phase_measurements[i].is_consistent ? 1 : 0;
    }
}

uint16_t getActualSpeed(void){

	return (uint16_t)filtered_speed;
}
uint16_t rpmToPeriod(uint16_t rpm) {
    if (rpm == 0) return 0xFFFF;
    
    // Calcular frecuencia eléctrica (Hz)
    double f_elec = (rpm * motor_pole_pairs) / 60.0;
    
    // Calcular período entre cruces por cero (segundos)
    double T_zc = 1.0 / (2.0f*f_elec);
    
    // Convertir a ticks (considerando prescaler)
    double ticks = T_zc * (TIMER_CLOCK / (TIM2->PSC + 1));
    
    // Aplicar límites del sistema
    if (ticks < SPEED_MAX) return SPEED_MAX;
    if (ticks > SPEED_MIN) return SPEED_MIN;
    
    return (uint16_t)ticks;
}
uint16_t periodToRpm(uint16_t period) {
    if (period == 0 || period == 0xFFFF) return 0;
    
    // Convertir ticks a segundos
    double T_zc = (double)period / (TIMER_CLOCK / (TIM2->PSC + 1));
    
    // Calcular frecuencia eléctrica (Hz)
    double f_elec = 1.0 / (2.0f*T_zc);
    
    // Calcular RPM mecánicos
    double rpm = (f_elec * 60.0) / motor_pole_pairs;
    
    // Aplicar límites del motor
    if (rpm < 100) return 100;
    if (rpm > 7000) return 7000;
    
    return (uint16_t)rpm;
}
uint16_t periodToPwm(uint16_t period) {
    return mapSpeed(period);  // Usa tu función existente
}
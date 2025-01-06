/*
 * config_datos.h
 *
 *  Created on: Dec 30, 2024
 *      Author: Usuario
 */

#ifndef INC_CONFIG_DATOS_H_
#define INC_CONFIG_DATOS_H_

// CONSTANTES MATEMATICAS
#define E 2.72 // Constante de Euler
#define PI 3.14
// CARACTERISTICAS DEL MOTOR
#define PARES_POLOS 4  // PARES DE POLOS DEL MOTOR



//CONSTANTES DE CONVERSION
#define FACT_TIEMPO_CONMUTACION  5/PARES_POLOS // t_30° =  20/(RPS*PARES_POLOS)

// ARRANQUE

#define VEL_INICIAL 0.08 // RPS
#define VEL_FINAL 2.5 //RPS
#define STARTUP_TIME 2000 // ms
#define ALPHA (VEL_FINAL - VEL_INICIAL) / STARTUP_TIME
#define RESOLUCION 20// ms
#define K_SLOPE 0.1 // Pendiente de la rampa
#define DELTA_N  VEL_FINAL - VEL_INICIAL
#define DUTY_MIN 10
#define DUTY_ALIGN 50
#define DUTY_MAX 90
#define DUTY_SLOPE (DUTY_MAX - DUTY_MIN) / STARTUP_TIME

#endif /* INC_CONFIG_DATOS_H_ */


//FUNCIONES


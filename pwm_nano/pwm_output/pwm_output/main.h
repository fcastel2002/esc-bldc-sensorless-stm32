/*
 * main.h
 *
 * Created: 04/02/2025 15:35:11
 *  Author: Usuario
 */ 


#ifndef MAIN_H_
#define MAIN_H_


#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint8_t adc_low;
volatile uint16_t adc_value;

#endif /* MAIN_H_ */
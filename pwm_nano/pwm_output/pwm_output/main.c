/*
 * pwm_output.c
 *
 * Created: 04/02/2025 15:11:48
 * Author : Usuario
 */ 
#include "main.h"
#define MUESTRAS 16

uint16_t leer_adc() {
	uint32_t suma = 0;
	for (uint8_t i = 0; i < MUESTRAS; i++) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		suma += ADCL | (ADCH << 8);
	}
	return (suma + MUESTRAS/2) / MUESTRAS;  // Redondeo
}

int main(void) {
	ADMUX = (1 << REFS0);                   // AVCC como referencia
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Preescalador 128

	DDRB |= (1 << DDB1);                    // PB1 (OC1A) como salida
	TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Fast PWM, no invertido
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Modo 14 (Fast PWM), TOP=ICR1
	ICR1 = 1023;                            // PWM de 10 bits (0–1023)

	while (1) {
		uint16_t adc_value = leer_adc();    // Lectura promediada
		OCR1A = adc_value;                  // Mapeo directo ADC ? PWM
	}
}


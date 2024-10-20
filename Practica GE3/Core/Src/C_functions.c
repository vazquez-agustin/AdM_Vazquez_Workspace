/* ################## Guía de Ejercicios 3 ################## */

#include "functions.h"

/* Ejercicio 1 */
uint32_t potencia (int16_t * vecIn, uint32_t longitud)
{
	uint32_t accum = 0;
	for (uint8_t i = longitud; i > 0; i --) {
		accum += (uint32_t)(vecIn[i-1] * vecIn[i-1]);
	}
	return accum / longitud;
}

/* Ejercicio 2 */
void medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud) {
	for (uint8_t i = longitud; i > 0; i --) {
		e[i-1]  = (x[i-1] - y[i-1]) / 2;
	}
}

/* Ejercicio 3 */
void eco (int16_t * signal, int16_t *eco, uint32_t longitud) {
	for( uint32_t i=0; i < 882; i++) {
		eco[i] = signal[i];
	}
	for( uint32_t i=882; i < longitud; i++){
		eco[i] = signal[i-882]/2 + signal[i];
	}
}

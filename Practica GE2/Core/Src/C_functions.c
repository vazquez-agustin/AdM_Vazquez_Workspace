/* ################## Guía de Ejercicios 2 ################## */

#include "functions.h"

/* Ejercicio 1 */
void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud) {

	for (uint8_t i = longitud; i > 0; i --) {
		vectorOut[i-1] = (int16_t)(vectorIn[i-1]>>16);
	}
}

/* Ejercicio 2 */
uint32_t max (int32_t * vectorIn, uint32_t longitud) {
	int32_t  maxValor = vectorIn[longitud - 1];
	uint32_t maxIndice = longitud -1;

	for (uint32_t i = longitud; i > 1; i --) {
		if(vectorIn[i-2] > maxValor) {
			maxValor = vectorIn[i-2];
			maxIndice = i-2;
		}
	}
	return maxIndice;
}

/* Ejercicio 3 */
void downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N) {
    uint32_t j = longitud - ( longitud / N );
	for (uint32_t i = longitud; i > 0; i --) {
		if(i % N != 0) {
			vectorOut[j-1] = vectorIn[i-1];
			j--;
		}
	}
}

/* Ejercicio 4 */
void invertir (uint16_t * vector, uint32_t longitud) {
	uint32_t i,j;
	uint16_t tmp = 0;

    for (i=0,j=longitud-1; i < j; i++, j-- ) {
		tmp = vector[j];
		vector[j] = vector[i];
		vector[i] = tmp;
	}
}

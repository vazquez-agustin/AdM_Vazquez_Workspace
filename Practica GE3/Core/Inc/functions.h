/*
 * functions.h
 *
 *  Created on: Oct 3, 2024
 *      Author: raxt
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "inttypes.h"

/* Funciones en implementadas en C */
uint32_t potencia (int16_t * vecIn, uint32_t longitud);
void medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud);
void eco (int16_t * signal, int16_t *eco, uint32_t longitud);

/* Funciones implementadas en Assembly*/
uint32_t asm_potencia (int16_t * vecIn, uint32_t longitud);
uint32_t asm_potencia_DSP (int16_t * vecIn, uint32_t longitud);
void asm_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud);
void asm_medDif_DSP(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud);
void asm_eco (int16_t * signal, int16_t *eco, uint32_t longitud);
void asm_eco_DSP (int16_t * signal, int16_t *eco, uint32_t longitud);

#endif /* INC_FUNCTIONS_H_ */

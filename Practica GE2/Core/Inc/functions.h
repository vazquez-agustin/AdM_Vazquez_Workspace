/*
 * functions.h
 *
 *  Created on: Sep 19, 2024
 *      Author: raxt
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "inttypes.h"

/* Funciones en implementadas en C */
void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
uint32_t max (int32_t * vectorIn, uint32_t longitud);
void downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void invertir (uint16_t * vector, uint32_t longitud);

/* Funciones implementadas en Assembly*/
void asm_pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
uint32_t asm_max (int32_t * vectorIn, uint32_t longitud);
void asm_downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void asm_invertir (uint16_t * vector, uint32_t longitud);

#endif /* INC_FUNCTIONS_H_ */

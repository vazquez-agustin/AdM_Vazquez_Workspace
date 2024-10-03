/* ################## Guía de Ejercicios 2 ################## */
/* ## Repositorio:                                         ## */
/* ##  https://github.com/RobAxt/AdM_workspace/tree/main   ## */
/* ########################################################## */
/* ## Grupo:                                               ## */
/* ##  e2301 Agustín Jesús Vazquez                         ## */
/* ##  e2312 Franco Chiesa Docampo                         ## */
/* ##  i0616 Roberto Oscar Axt                             ## */
/* ########################################################## */

.text
.syntax unified
.thumb

/* Exportar funciones*/
.global asm_pack32to16
.global asm_max
.global asm_downSample
.global asm_invertir

/* Ejercicio 1 */
// Prototipo en archivo .h:
//void asm_pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);

// Parámetros recibidos
//	r0:  vectorIn
//	r1:  vectorOut
//  r2:  longitud

// Valor de retorno: void
.thumb_func
 	asm_pack32to16:
		push {r4 - r7} // Se resguardan los valores de los registros en el stack
		cbz r0, _exit1	// Guarda de seguridad. Salir si vectorIn es una dirección nula
		cbz r1, _exit1	// Guarda de seguridad. Salir si vectorOut es una dirección nula

	_loop1:
		sub  r2, #1	// longitud = longitud - 1
		ldr  r3, [r0, r2, LSL #2] // Se carga desde el último valor hacia el primero
		lsr  r3, r3, #16       	  // r3 = r3>>16
		strh r3, [r1, r2, LSL #1] // Se guarda el resultado
		cbz  r2, _exit1	// Si r2 = 0, la función termina
		b _loop1		// De lo contrario seguimos iterando

	_exit1:
		pop {r4 - r7} // Se recuperan los valores de los registros
		bx lr

/* Ejercicio 2*/
// Prototipo en archivo .h:
// uint32_t asm_max (int32_t * vectorIn, uint32_t longitud);

// Parámetros recibidos
//	r0: vectorIn
//	r1: longitud

// Valor de retorno: uint32_t con el indice del mayor elemento
.thumb_func
	asm_max:
		push {r4 - r7} // Se resguardan los valores de los registros en el stack
		cbz r0, _exit2	// Guarda de seguridad. Salir si vectorIn es una dirección nula

        sub  r1, #1	              // longitud = longitud - 1
		ldr  r3, [r0, r1, LSL #2] // maxValor es r3
        mov  r4, r1               // maxIndice es r4

	_loop2:
		sub  r1, #1	     // longitud = longitud - 1
		ldr  r2, [r0, r1, LSL #2] // Se carga desde el último valor hacia el primero
        cmp  r2, r3     // r2-r3 > 0 --> r2>r3
        itt  gt         // si es mayor que (great than) actualizo
        movgt r3, r2    // actualizo maxValor
        movgt r4, r1    // actualizo maxIndice
		cbz r1, _exit2	// Si r1 = 0, la función termina
		b _loop2        // De lo contrario seguimos iterando

	_exit2:
	    mov r0, r4      // se devuelve maxIndice
		pop {r4 - r7}   // Se recuperan los valores de los registros
		bx lr

/* Ejercicio 3*/
// Prototipo en archivo .h:
// void asm_downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);

// Parámetros recibidos
//	r0: vectorIn
//	r1: vectorOut
//  r2: longitud
//  r3: N

// Valor de retorno: void
.thumb_func
	asm_downSample:
		push {r4 - r7}   // Se resguardan los valores de los registros en el stack
		cbz r0, _exit3	 // Guarda de seguridad. Salir si vectorIn es una dirección nula
		cbz r1, _exit3	 // Guarda de seguridad. Salir si vectorOut es una dirección nula

        udiv r4, r2, r3  // división: r4 = longitud / N
        sub  r4, r2, r4  // r4 = longitud - longitud / N ---> longitud vectorOut

	_loop3:
        udiv  r6, r2, r3             //  r6 = r2 / r3 (i / N)  |
        mul   r7, r6, r3             //  r7 = r6 * r3 (r6 * N) |  r2 % r3
        subs  r7, r2, r7             //  r2 - r7      residuo  |
		sub   r2, #1	             // longitud = longitud - 1
		ittt  ne			         // if != 0
		ldrne r5, [r0, r2, LSL #2]   // Se carga desde el último valor hacia el primero
		subne r4, #1                 // (!=0) r4 = r4 -1
		strne r5, [r1, r4, LSL #2]   // (!=0) r1[r4] = r5
		cbz r2, _exit3	// Si r1 = 0, la función termina
		b _loop3        // De lo contrario seguimos iterando

	_exit3:
		pop {r4 - r7} // Se recuperan los valores de los registros
		bx lr

/* Ejercicio 4 */
// Prototipo en archivo .h:
// void asm_invertir (uint16_t * vector, uint32_t longitud);

// Parámetros recibidos
//	r0: vector
//	r1: longitud   ( indice j)
// Valor de retorno: void
.thumb_func
	asm_invertir:
		push {r4 - r7}  // Se resguardan los valores de los registros en el stack
		cbz r0, _exit4	// Guarda de seguridad. Salir si vectorIn es una dirección nula
        cbz r1, _exit4  // Guarda de seguridad. Salir si longitud es cero

		mov r2, #0      // i = 0

	_loop4:
    	cmp r2, r1                   // Comparar i con j (r2 con r1)
	    bge _exit4                   // Si i >= j, salir del bucle

		sub r1, #1                   // j--
    	ldrh r3, [r0, r1, LSL #1]    // r3 = vector[j]
    	ldrh r4, [r0, r2, LSL #1]    // r4 = vector[i]
    	strh r4, [r0, r1, LSL #1]    // vector[j] = r4
    	strh r3, [r0, r2, LSL #1]    // vector[i] = r3
    	add r2, #1                   // i++

        b _loop4

	_exit4:
		pop {r4 - r7} // Se recuperan los valores de los registros
		bx lr

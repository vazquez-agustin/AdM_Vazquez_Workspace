/* ################## Guía de Ejercicios 1 ################## */
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
.global asm_productoEscalar32
.global asm_productoEscalar16
.global asm_bitfield_clear
.global asm_bitfield_toggle

/* Ejercicio 1 */
// Prototipo en archivo .h:
// void asm_productoEscalar32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar)

// Parámetros recibidos
//	r0: dirección vector de entrada
//	r1: dirección vector de salida
//	r2: longitud de ambos vectores
//	r3: escalar

// Valor de retorno: void
.thumb_func
	asm_productoEscalar32:
		// Como r0 a r4 están los parámetros, se deben guardar los demás registros
		// antes de poder utilizarlos. IMPORTANTE: hacer el push antes de las guardas
		push {r4 - r7} // Se resguardan los valores de los registros en el stack

		// Guarda de seguridad
		cbz r0, _exit1	// Salir si vectorIn es una dirección nula
		cbz r1, _exit1	// Salir si vectorOut es una dirección nula

	_loop1:
		sub r2, #1	// longitud = longitud - 1
		// LSL #2: es para desplazarnos de a 4 bites
		ldr r4, [r0, r2, LSL #2] // Se carga desde el último valor hacia el primero
		mul r4, r4, r3	// r4 = vectorIn*escalar
		str r4, [r1, r2, LSL #2] // Se guarda el resultado
		cbz r2, _exit1	// Si r2 = 0, la función termina
		b _loop1		// De lo contrario seguimos iterando

	_exit1:
		pop {r4 - r7} // Se recuperan los valores de los registros
		bx lr


/* Ejercicio 2 */
// Prototipo en archivo .h:
// void asm_productoEscalar16(uint16_t *vectorIn, uint16_t *vectorOut, uint16_t longitud, uint16_t escalar)

// Parámetros recibidos
//	r0: dirección vector de entrada
//	r1: dirección vector de salida
//	r2: longitud de ambos vectores
//	r3: escalar

// Valor de retorno: void
.thumb_func
	asm_productoEscalar16:

		push {r4 - r7}

		// Guarda de seguridad
		cbz  r0, _exit2
		cbz  r1, _exit2
	_loop2:
		sub  r2, #1	// longitud = longitud - 1

		ldrh r4, [r0, r2, LSL #1] // r4 =  *(r0 +r2*2)
		mul  r4, r4, r3	          // r4 = vectorIn*escalar
		usat r4, #12, r4          // satura r4 en 12 bits
		strh r4, [r1, r2, LSL #1] // *(r1 +r2*2) = r4
		cbz  r2, _exit2	// Si r2 = 0, la función termina
		b _loop2		// De lo contrario seguimos iterando

	_exit2:
		pop {r4 - r7} // Se recuperan los valores de los registros
		bx lr


/* Ejercicio 3 */
// Prototipo en archivo .h:
// uint32_t asm_bitfield_clear(uint32_t dato, uint32_t ancho, uint32_t inicio)

// Parámetros recibidos:
//	r0: dato a procesar
//	r1: ancho del campo de bits
//	r2: posición del bit menos significativo del campo de bits

// Valor de retorno: valor procesado

.thumb_func
	asm_bitfield_clear:
		// Guarda de seguridad
		cbz r0, _exit3	// Salir si el dato es cero
		cbz r1, _exit3	// Salir si el ancho del campo es cero

		mov r3, #1	// Se coloca un 1 en el lsb de r3
		lsl r3, r1	// Se desplaza el 1 según el ancho del campo
		sub r3, #1	// Se genera la máscara. Ej: 001000 - 1 = 000111
		lsl r3, r2	// Se desplaza la máscara según inicio. Ej: 011100
		mvn r3, r3	// Se niega la máscara. Ej: 100011
		and r0, r3	// Se aplica and bit a bit. Se devuelve el resultado en r0

	_exit3:
		bx lr


/* Ejercicio 4 */
// Prototipo en archivo .h:
// uint32_t asm_bitfield_toggle(uint32_t dato, uint32_t ancho, uint32_t inicio)

// Parámetros recibidos:
//	r0: dato a procesar
//	r1: ancho del campo de bits
//	r2: posición del bit menos significativo del campo de bits

// Valor de retorno: valor procesado

.thumb_func
	asm_bitfield_toggle:
		// Guarda de seguridad
		cbz r0, _exit4	// Salir si el dato es cero
		cbz r1, _exit4	// Salir si el ancho del campo es cero

		mov r3, #1	// Se coloca un 1 en el lsb de r3
		lsl r3, r1	// Se desplaza el 1 según el ancho del campo
		sub r3, #1	// Se genera la máscara. Ej: 001000 - 1 = 000111
		lsl r3, r2	// Se desplaza la máscara según inicio. Ej: 011100
		eor r0, r3	// Se aplica xor bit a bit. Se devuelve el resultado en r0

	_exit4:
		bx lr

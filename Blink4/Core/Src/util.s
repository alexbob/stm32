/*

	Utilities functions:
		_setreg - set or reset pins
		_toggle - toggle pins

*/

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global _setreg
.type	_setreg, %function

/**
 * @brief  	This is the code caltulates offset from the base mem addr to the
 * 		register mem address and set/reset bits
 * @param
 *		r0 - Base address
 *		r1 - Offset to the register
 *		r2 - bitmask for output pins
 * @retval
 		r0 - 0x0
*/

_setreg:

	ldr 	r3, [r0]		// load base mem addr
	add 	r3, r1          	// calculate offset

	ldr 	r0, [r3]		// load  reg address
	ldr	r1, [r2]
	orr 	r0, r1			// mask bits offset
	str 	r0, [r3]		// push value to register

	mov 	r0, #0x0		// return 0
	bx 	lr

/**
 * @brief  	This is the code caltulates offset from the base mem addr to the
 * 		register mem address and toggle bits
 * @param
 *		r0 - Base address
 *		r1 - Offset to the register
 *		r2 - bitmask for pins to toggle
 * @retval
 *		r0 - 0x0
*/
.global _toggle
.type	_toggle, %function

_toggle:
	ldr r3, [r0]			// load base mem addr
	add r3, r1          		// calculate offset

	ldr r0, [r3]			// load  reg address
	ldr r1, [r2]
	eor r0, r1
	str r0, [r3]			// push value to register

	mov r0, #0x0			// return 0
	bx lr

.end

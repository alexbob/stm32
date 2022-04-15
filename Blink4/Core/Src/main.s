/**
 *
 *	Test blink
 *
 *		- Enable clock for PortC
 *		- Set type for PortC PIN 13 to output
 *		- Toggle pin with delay in 655350 cycles
*/


  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global main
.type	main, %function

.data
	RCC_BASE: 		.word 	0x40023800  	// RCC start address
	GPIOC_BASE:		.word	0x40020800	// PORC C Base addr
	ModeMask:		.word 	0x04000000
	ClockMask:		.word	0x00000004
	Pin13Mask:		.word	0x00002000


.text

main:
	.fnstart


	/* enable Clock for port C */

	ldr	r0, =RCC_BASE			//  base mem address
	mov 	r1, #0x30			//  register offset
	movw	r2, #:lower16:ClockMask
	movt	r2, #:upper16:ClockMask
	bl 	_setreg

	/* set output type for pin(s) */

	ldr 	r0, =GPIOC_BASE			// base mem for port C
	mov	r1, #0x0			// register offset
	movw	r2, #:lower16:ModeMask		// bitmask
	movt	r2, #:upper16:ModeMask
	bl 	_setreg

	/* toggle */
onoff:
	ldr 	r0, =GPIOC_BASE 		// base mem addr for PORT C
	mov	r1, #0x14			// bit to toggle offset
	movw	r2, #:lower16:Pin13Mask
	movt	r2, #:upper16:Pin13Mask
	bl	_toggle

	/* delay  */
	ldr 	r0, =655350
delay:
	sub 	r0, 1
	cbz 	r0, exit
	b 	delay

exit:
 	b onoff


.end


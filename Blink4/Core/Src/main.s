/*

	Test blink


*/


  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global main
.type	main, %function

.data
	RCC_BASE: 		.word 	0x40023800  // RCC start address
	GPIOC_BASE:		.word	0x40020800

.text

main:
	.fnstart

	// calculate reg enable location for port C

	ldr r0, =RCC_BASE
	ldr r1, [r0]
	add r1, #0x30			// offset for RCC_APB1ENR register

	ldr r0, [r1]			// load RCC reg values
	orr r0, r0, 0x4			// bit offset
	str r0, [r1]			// enable clock for port c


	// set reg output type

	ldr r0, =GPIOC_BASE
	ldr r1, [r0]
	add r1, 0x0				// register offset for output type

	ldr r0, [r1]
	orr r0, r0, 0x04000000	// Port C pin 13 output mode
	str r0, [r1]

	b onoff

	// on / off


onoff:
	ldr r0, =GPIOC_BASE
	ldr r1, [r0]
	add r1, 0x14			// offset

	ldr r0, [r1]
	eor r0, r0, 0x000002000
	str r0, [r1]

	ldr r0, =655350
delay:
	sub r0, r0, 1
	cbz r0, exit
	b delay

exit:
	b onoff


/* GPL-3.0-only. STM32F405 vector map, RM0090 and ST stm32f405xx.h. */
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
.section .isr_vector,"a",%progbits
.balign 512
.global vectors
vectors:
 .word _stack_top, Reset_Handler
 .word Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler
 .word 0,0,0,0
 .word Default_Handler, Default_Handler,0,Default_Handler,SysTick_Handler
 /* external IRQs 0..11 */
 .rept 12
 .word Default_Handler
 .endr
 .word DMA1_Stream1_IRQHandler /* 12 */
 .rept 15 /* 13..27 */
 .word Default_Handler
 .endr
 .word TIM2_IRQHandler /* 28 */
 .word Default_Handler,Default_Handler /* 29,30 */
 .word I2C1_EV_IRQHandler,I2C1_ER_IRQHandler,I2C2_EV_IRQHandler,I2C2_ER_IRQHandler
 .word Default_Handler,Default_Handler /* 35,36 */
 .word USART1_IRQHandler,Default_Handler,USART3_IRQHandler /* 37..39 */
 .rept 10 /* 40..49 */
 .word Default_Handler
 .endr
 .word Default_Handler /* 50: TIM5 unused */
 .rept 7 /* 51..57 */
 .word Default_Handler
 .endr
 .word DMA2_Stream2_IRQHandler /* 58 */
 .rept 23 /* 59..81 (F405 last IRQ is FPU) */
 .word Default_Handler
 .endr
.section .text.Reset_Handler,"ax",%progbits
.thumb_func
.global Reset_Handler
Reset_Handler:
 cpsid i
 /* Enable FPU before any compiler-generated code. */
 ldr r0,=0xe000ed88
 ldr r1,=0x00f00000
 ldr r2,[r0]
 orrs r2,r1
 str r2,[r0]
 dsb
 isb
 ldr r0,=_data_load
 ldr r1,=_data_start
 ldr r2,=_data_end
1:
 cmp r1,r2
 bcs 2f
 ldr r3,[r0],#4
 str r3,[r1],#4
 b 1b
2:
 ldr r1,=_bss_start
 ldr r2,=_bss_end
 movs r3,#0
3:
 cmp r1,r2
 bcs 4f
 str r3,[r1],#4
 b 3b
4:
 ldr r0,=0x40023830
 ldr r2,[r0]
 orr r2,r2,#0x00100000
 str r2,[r0]
 ldr r1,=_ccm_start
 ldr r2,=_ccm_end
5:
 cmp r1,r2
 bcs 6f
 str r3,[r1],#4
 b 5b
6:
 ldr r0,=vectors
 ldr r1,=0xe000ed08
 str r0,[r1]
 bl main
 b Default_Handler
.thumb_func
.global Default_Handler
Default_Handler:
 /* Watchdog resets after a fault. No fake resume and no semihosting. */
 cpsid i
7:
 wfi
 b 7b
.section .note.GNU-stack,"",%progbits

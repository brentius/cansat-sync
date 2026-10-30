/* GPL-3.0-only. STM32F405RG: 1 MiB flash, 128 KiB SRAM, 64 KiB CCM.
 * CCM is CPU-only. Stack lives there; DMA buffers always live in SRAM.
 * Optional .ccm objects must be zero-initialised, never initialised data. */
ENTRY(Reset_Handler)
MEMORY {
 FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
 RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
 CCM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}
_stack_top = ORIGIN(CCM) + LENGTH(CCM);
_stack_bottom = _stack_top - 16K;
SECTIONS {
 .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
 .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
 .ARM.extab : { *(.ARM.extab*) } > FLASH
 .ARM.exidx : { *(.ARM.exidx*) } > FLASH
 .data : {
  . = ALIGN(4); _data_start = .; *(.data*) . = ALIGN(4); _data_end = .;
 } > RAM AT> FLASH
 _data_load = LOADADDR(.data);
 .bss (NOLOAD) : {
  . = ALIGN(8); _bss_start = .; *(.bss*) *(COMMON) . = ALIGN(8); _bss_end = .;
 } > RAM
 .ccm (NOLOAD) : {
  . = ALIGN(8); _ccm_start = .; *(.ccm*) . = ALIGN(8); _ccm_end = .;
 } > CCM
 .stack _stack_bottom (NOLOAD) : { . = . + 16K; } > CCM
 /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
ASSERT(_bss_end <= ORIGIN(RAM)+LENGTH(RAM), "SRAM overflow (DMA/app/core)")
ASSERT(_ccm_end <= _stack_bottom, "CCM overflow: 16 KiB stack reserved")
ASSERT((_stack_top & 7) == 0, "AAPCS stack must be 8-byte aligned")

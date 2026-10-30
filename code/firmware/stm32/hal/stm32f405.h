/* GPL-3.0-only. Own minimal STM32F405 register map, checked against ST's
 * stm32f405xx.h and RM0090; no vendor header is incorporated. See SOURCES.md. */
#ifndef HAL_STM32F405_H
#define HAL_STM32F405_H
#include <stdint.h>
#include <stdbool.h>
#define HAL_REG(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define HAL_RCC(off) HAL_REG(0x40023800u+(off))
#define HAL_PWR(off) HAL_REG(0x40007000u+(off))
#define HAL_FLASH(off) HAL_REG(0x40023c00u+(off))
#define HAL_SCB(off) HAL_REG(0xe000ed00u+(off))
typedef struct {
    volatile uint32_t moder,otyper,ospeedr,pupdr,idr,odr,bsrr,lckr,afr[2];
} hal_gpio_t;
typedef struct {
    volatile uint32_t cr1,cr2,smcr,dier,sr,egr,ccmr1,ccmr2,ccer,cnt,psc,arr,rcr,ccr[4],bdtr,dcr,dmar;
} hal_tim_t;
typedef struct { volatile uint32_t cr1,cr2,sr,dr,crcpr,rxcrcr,txcrcr,i2scfgr,i2spr; } hal_spi_t;
typedef struct { volatile uint32_t sr,dr,brr,cr1,cr2,cr3,gtpr; } hal_usart_t;
typedef struct { volatile uint32_t cr1,cr2,oar1,oar2,dr,sr1,sr2,ccr,trise,fltr; } hal_i2c_regs_t;
typedef struct { volatile uint32_t cr,ndtr,par,m0ar,m1ar,fcr; } hal_dma_stream_t;
#define HAL_GPIOA ((hal_gpio_t *)0x40020000u)
#if __STDC_HOSTED__
/* Register model supplied by host tests; target uses only physical MMIO. */
extern hal_gpio_t hal_host_gpiob;
extern hal_tim_t hal_host_tim2;
#define HAL_GPIOB (&hal_host_gpiob)
#define HAL_TIM2 (&hal_host_tim2)
#else
#define HAL_GPIOB ((hal_gpio_t *)0x40020400u)
#define HAL_TIM2 ((hal_tim_t *)0x40000000u)
#endif
#define HAL_GPIOC ((hal_gpio_t *)0x40020800u)
#define HAL_GPIOD ((hal_gpio_t *)0x40020c00u)
#define HAL_TIM3 ((hal_tim_t *)0x40000400u)
#define HAL_TIM4 ((hal_tim_t *)0x40000800u)
#define HAL_TIM5 ((hal_tim_t *)0x40000c00u)
#define HAL_TIM8 ((hal_tim_t *)0x40010400u)
#define HAL_SPI1 ((hal_spi_t *)0x40013000u)
#define HAL_SPI2 ((hal_spi_t *)0x40003800u)
#define HAL_USART1 ((hal_usart_t *)0x40011000u)
#define HAL_USART3 ((hal_usart_t *)0x40004800u)
#define HAL_I2C1 ((hal_i2c_regs_t *)0x40005400u)
#define HAL_I2C2 ((hal_i2c_regs_t *)0x40005800u)
#define HAL_DMA1 0x40026000u
#define HAL_DMA2 0x40026400u
#define HAL_DMA_STREAM(base,n) ((hal_dma_stream_t *)((base)+0x10u+0x18u*(n)))
#define HAL_BIT(n) (UINT32_C(1)<<(n))
static inline uint32_t hal_irq_save(void)
{
#if __STDC_HOSTED__
    __atomic_signal_fence(__ATOMIC_SEQ_CST); return 0;
#else
    uint32_t x; __asm__ volatile("mrs %0, primask\ncpsid i":"=r"(x)::"memory"); return x;
#endif
}
static inline void hal_irq_restore(uint32_t x)
{
#if __STDC_HOSTED__
    (void)x; __atomic_signal_fence(__ATOMIC_SEQ_CST);
#else
    __asm__ volatile("msr primask, %0"::"r"(x):"memory");
#endif
}
static inline void hal_gpio_write(hal_gpio_t *p,unsigned pin,bool on)
{ p->bsrr=HAL_BIT(pin+(on?0u:16u)); }
static inline bool hal_gpio_read(hal_gpio_t *p,unsigned pin)
{ return (p->idr&HAL_BIT(pin))!=0; }
static inline void hal_nvic(unsigned irq,unsigned priority)
{
    ((volatile uint8_t *)0xe000e400u)[irq]=(uint8_t)(priority<<4);
    HAL_REG(0xe000e100u+4u*(irq/32u))=HAL_BIT(irq%32u);
}
void hal_gpio_mode(hal_gpio_t *,unsigned pin,unsigned mode,unsigned af,bool open_drain,unsigned pull);
/* HSE 8 MHz -> PLL 168 MHz, APB1 42 MHz, APB2 84 MHz.
 * Returns false on oscillator/PLL failure; caller must fail safe. */
bool hal_clock_init(void);
#endif

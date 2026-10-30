/* GPL-3.0-only. RM0090 clock tree for the connection guide's 8 MHz HSE. */
#include "../hal/stm32f405.h"
static bool wait_mask(volatile uint32_t *reg,uint32_t mask,uint32_t expected)
{
    uint32_t remaining=4000000u;
    while((*reg&mask)!=expected) if(--remaining==0) return false;
    return true;
}
bool hal_clock_init(void)
{
    HAL_SCB(0x88)|=0x00f00000u;
    __asm__ volatile("dsb\nisb":::"memory");
    HAL_RCC(0x40)|=HAL_BIT(28); /* PWR */
    HAL_PWR(0)|=HAL_BIT(14); /* voltage scale 1, supply [assumed] 3.3 V */
    HAL_RCC(0)|=HAL_BIT(16);
    if(!wait_mask(&HAL_RCC(0),HAL_BIT(17),HAL_BIT(17))) return false;
    HAL_FLASH(0)=HAL_BIT(10)|HAL_BIT(9)|HAL_BIT(8)|5u;
    /* PLLM=8, PLLN=336, PLLP=2, PLLQ=7; AHB=/1 APB1=/4 APB2=/2. */
    HAL_RCC(4)=8u|(336u<<6)|HAL_BIT(22)|(7u<<24);
    HAL_RCC(8)=(5u<<10)|(4u<<13);
    HAL_RCC(0)|=HAL_BIT(24);
    if(!wait_mask(&HAL_RCC(0),HAL_BIT(25),HAL_BIT(25))) return false;
    HAL_RCC(8)|=2u;
    if(!wait_mask(&HAL_RCC(8),12u,8u)) return false;
    HAL_REG(0xe000e014u)=168000u-1u;
    HAL_REG(0xe000e018u)=0; HAL_REG(0xe000e010u)=7;
    ((volatile uint8_t *)0xe000ed23u)[0]=0xf0; /* SysTick lowest priority */
    return true;
}
void hal_gpio_mode(hal_gpio_t *p,unsigned pin,unsigned mode,unsigned af,bool od,unsigned pull)
{
    uint32_t shift=pin*2u,mask=3u<<shift;
    p->otyper=(p->otyper&~HAL_BIT(pin))|(od?HAL_BIT(pin):0u);
    p->ospeedr=(p->ospeedr&~mask)|(2u<<shift); /* medium/high edge, scope on PCB */
    p->pupdr=(p->pupdr&~mask)|(pull<<shift);
    p->afr[pin/8u]=(p->afr[pin/8u]&~(15u<<(4u*(pin%8u))))|(af<<(4u*(pin%8u)));
    /* Connect the requested mode last: a high GPIO entering AF must not
     * briefly expose the previously selected alternate function (trigger
     * and PMOS gate transitions depend on this ordering). */
    p->moder=(p->moder&~mask)|(mode<<shift);
}

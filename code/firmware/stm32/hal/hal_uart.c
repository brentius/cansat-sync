/* GPL-3.0-only. RM0090 circular RX DMA, normal TX DMA and wire-idle check.
 * DMA buffers MUST reside in SRAM, not the CPU-only CCM memory. */
#include "hal_uart.h"
#include "hal_logic.h"
static unsigned dma_shift(unsigned stream)
{ static const unsigned shifts[4]={0,6,16,22}; return shifts[stream&3u]; }
static uint32_t dma_flags(const hal_uart_t *u,unsigned stream)
{ return HAL_REG(u->dma+(stream<4u?0u:4u))>>dma_shift(stream)&0x3fu; }
static void dma_clear(const hal_uart_t *u,unsigned stream,uint32_t flags)
{ HAL_REG(u->dma+(stream<4u?8u:12u))=flags<<dma_shift(stream); }
static void rx_start(hal_uart_t *u)
{
    u->rx->cr=0; dma_clear(u,u->rx_stream,0x3f);
    u->rx->par=(uint32_t)(uintptr_t)&u->regs->dr;
    u->rx->m0ar=(uint32_t)(uintptr_t)u->rx_data; u->rx->ndtr=HAL_UART_RX_CAP;
    u->rx->fcr=0; u->rx->cr=HAL_BIT(27)|HAL_BIT(16)|HAL_BIT(10)|HAL_BIT(8)|HAL_BIT(4)|HAL_BIT(2)|HAL_BIT(1)|1u;
    u->wraps=0; u->consumed=0;
}
void hal_uart_init(hal_uart_t *u,bool pi,uint32_t *errors)
{
    *u=(hal_uart_t){0}; u->errors=errors; u->regs=pi?HAL_USART1:HAL_USART3;
    u->dma=pi?HAL_DMA2:HAL_DMA1; u->rx_stream=pi?2u:1u; u->tx_stream=pi?7u:3u;
    u->rx=HAL_DMA_STREAM(u->dma,u->rx_stream); u->tx=HAL_DMA_STREAM(u->dma,u->tx_stream);
    u->pclk=pi?84000000u:42000000u; u->baud=pi?230400u:9600u;
    u->regs->cr1=0; u->regs->brr=(u->pclk+u->baud/2u)/u->baud;
    u->regs->cr2=0; u->regs->cr3=HAL_BIT(6)|HAL_BIT(7)|1u;
    u->regs->cr1=HAL_BIT(13)|HAL_BIT(3)|HAL_BIT(2);
    rx_start(u);
    hal_nvic(pi?58u:12u,3); hal_nvic(pi?37u:39u,3);
}
void hal_uart_rx_irq(hal_uart_t *u)
{
    uint32_t flags=dma_flags(u,u->rx_stream);
    if(flags&HAL_BIT(5)) { u->wraps++; dma_clear(u,u->rx_stream,HAL_BIT(5)); }
    if(flags&0x0du) { (*u->errors)++; u->rx->cr&=~1u; }
    dma_clear(u,u->rx_stream,flags&~HAL_BIT(5));
}
void hal_uart_irq(hal_uart_t *u)
{
    uint32_t sr=u->regs->sr;
    if(sr&15u) { (void)u->regs->dr; (*u->errors)++; }
}
void hal_uart_poll(hal_uart_t *u,uint32_t now)
{
    if(!(u->rx->cr&1u)) rx_start(u);
    if(u->transmitting) {
        uint32_t flags=dma_flags(u,u->tx_stream);
        if((flags&0x0du) || (int32_t)(now-u->deadline)>=0) {
            u->tx->cr&=~1u; (*u->errors)++; u->transmitting=false;
            dma_clear(u,u->tx_stream,0x3f);
        } else if(u->tx->ndtr==0 && (u->regs->sr&HAL_BIT(6))) {
            u->transmitting=false; dma_clear(u,u->tx_stream,0x3f);
        }
    }
}
static uint32_t uart_produced(hal_uart_t *u)
{
    uint32_t produced,ndtr,key=hal_irq_save();
    /* Account for a TC pending behind this critical section before NDTR.
     * Recheck once: a reload may occur between the flag and NDTR reads. */
    hal_uart_rx_irq(u); ndtr=u->rx->ndtr;
    if(dma_flags(u,u->rx_stream)&HAL_BIT(5)) { hal_uart_rx_irq(u); ndtr=u->rx->ndtr; }
    produced=u->wraps*HAL_UART_RX_CAP+(HAL_UART_RX_CAP-ndtr);
    hal_irq_restore(key); return produced;
}
void hal_uart_discard(hal_uart_t *u,bool abort_tx)
{
    u->consumed=uart_produced(u);
    if(abort_tx) { u->tx->cr&=~1u; u->transmitting=false; dma_clear(u,u->tx_stream,0x3f); }
}
size_t hal_uart_read(hal_uart_t *u,uint8_t *buf,size_t max)
{
    size_t i,n; uint32_t produced,after;
    if(!buf || !max) return 0;
    produced=uart_produced(u);
    n=hal_dma_available(produced,&u->consumed,HAL_UART_RX_CAP,u->errors);
    if(n>max) n=max;
    /* No global IRQ masking over a 2 KiB copy: DMA keeps receiving. */
    for(i=0;i<n;i++) buf[i]=u->rx_data[(u->consumed+(uint32_t)i)%HAL_UART_RX_CAP];
    after=uart_produced(u);
    if(after-u->consumed>HAL_UART_RX_CAP) {
        /* DMA overwrote the region during the copy: expose no corrupted
         * bytes. Keep the newest ring for next read and count the loss. */
        (void)hal_dma_available(after,&u->consumed,HAL_UART_RX_CAP,u->errors);
        return 0;
    }
    u->consumed+=(uint32_t)n; return n;
}
bool hal_uart_idle(hal_uart_t *u)
{ return !u->transmitting && !(u->tx->cr&1u) && (u->regs->sr&HAL_BIT(6))!=0; }
bool hal_uart_write(hal_uart_t *u,const uint8_t *buf,size_t n,uint32_t now)
{
    size_t i;
    if(!buf || !n || n>HAL_UART_TX_CAP || !hal_uart_idle(u)) return false;
    for(i=0;i<n;i++) u->tx_data[i]=buf[i];
    dma_clear(u,u->tx_stream,0x3f);
    u->tx->par=(uint32_t)(uintptr_t)&u->regs->dr;
    u->tx->m0ar=(uint32_t)(uintptr_t)u->tx_data; u->tx->ndtr=(uint32_t)n; u->tx->fcr=0;
    u->regs->sr&=~HAL_BIT(6);
    u->tx->cr=HAL_BIT(27)|HAL_BIT(10)|HAL_BIT(6)|HAL_BIT(2)|HAL_BIT(1)|1u;
    u->transmitting=true; u->deadline=now+hal_uart_transfer_budget((uint32_t)n,u->baud);
    return true;
}
void hal_uart_baud(hal_uart_t *u,uint32_t baud)
{
    uint32_t key;
    if(baud<1200 || baud>921600 || !hal_uart_idle(u)) return;
    key=hal_irq_save(); u->regs->cr1&=~HAL_BIT(13);
    u->regs->brr=(u->pclk+baud/2u)/baud; u->baud=baud;
    u->regs->cr1|=HAL_BIT(13); hal_irq_restore(key);
}

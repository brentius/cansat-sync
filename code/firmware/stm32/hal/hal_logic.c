/* GPL-3.0-only. Pure arithmetic used by register-level board implementation. */
#include "hal_logic.h"
void hal_ring_init(hal_ring_t *q,void *data,size_t item_size,uint32_t capacity)
{ *q=(hal_ring_t){0}; q->data=data; q->item_size=item_size; q->capacity=capacity; }
bool hal_ring_push(hal_ring_t *q,const void *item)
{
    size_t i; const uint8_t *src=item; uint8_t *dst;
    if(!item || q->head-q->tail>=q->capacity) return false;
    dst=q->data+q->write_index*q->item_size;
    for(i=0;i<q->item_size;i++) dst[i]=src[i];
    if(++q->write_index==q->capacity) q->write_index=0;
    __atomic_thread_fence(__ATOMIC_RELEASE); q->head++;
    return true;
}
bool hal_ring_pop(hal_ring_t *q,void *item)
{
    size_t i; uint8_t *dst=item; const uint8_t *src;
    if(!item || q->head==q->tail) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    src=q->data+q->read_index*q->item_size;
    for(i=0;i<q->item_size;i++) dst[i]=src[i];
    if(++q->read_index==q->capacity) q->read_index=0;
    q->tail++; return true;
}
uint32_t hal_ring_discard(hal_ring_t *q)
{
    uint32_t n=q->head-q->tail;
    q->read_index=q->write_index; q->tail=q->head; return n;
}
size_t hal_dma_available(uint32_t produced,uint32_t *consumed,uint32_t capacity,uint32_t *lost)
{
    uint32_t n=produced-*consumed;
    if(n>capacity) { *lost+=n-capacity; *consumed=produced-capacity; n=capacity; }
    return n;
}
uint32_t hal_uart_transfer_budget(uint32_t n,uint32_t baud)
{ return (uint32_t)(((uint64_t)n*10000000u)/baud)+20000u; }
bool hal_trig_values(uint32_t period,uint16_t pulse,uint16_t *psc,uint16_t *arr,
                     uint16_t *ccr,uint32_t *ap,uint16_t *aw)
{
    uint32_t div,counts,width;
    if(!period || !pulse || pulse>=period || !psc || !arr || !ccr || !ap || !aw) return false;
    div=(uint32_t)(((uint64_t)period+65535u)/65536u);
    if(div>65536u/84u) return false;
    counts=(period+div/2u)/div; width=((uint32_t)pulse+div/2u)/div;
    if(!width || width>=counts || width*div>UINT16_MAX || counts>65536u) return false;
    *psc=(uint16_t)(84u*div-1u); *arr=(uint16_t)(counts-1u); *ccr=(uint16_t)width;
    *ap=counts*div; *aw=(uint16_t)(width*div); return true;
}
void hal_adc_convert(uint16_t battery,uint16_t temperature,uint16_t vref,
                     uint16_t cal_vref,uint16_t cal30,uint16_t cal110,
                     uint16_t *mv,int8_t *tc)
{
    uint32_t vdda; int32_t t;
    *mv=0; *tc=INT8_MIN;
    if(battery>4095 || temperature>4095 || vref<500 || vref>2500) return;
    vdda=cal_vref>=500 && cal_vref<=2500?3300u*cal_vref/vref:1210u*4095u/vref;
    if(vdda<1800 || vdda>3600) return;
    *mv=(uint16_t)((uint32_t)battery*vdda*147u/(4095u*47u));
    if(cal30>=500 && cal110>cal30 && cal110<=4095) {
        t=30+(int32_t)(((int64_t)temperature*vdda-(int64_t)cal30*3300)*80/
                       ((int64_t)(cal110-cal30)*3300));
    } else t=25+((int32_t)((uint32_t)temperature*vdda/4095u)-760)*2/5;
    if(t< -127) t=-127;
    if(t>127) t=127;
    *tc=(int8_t)t;
}

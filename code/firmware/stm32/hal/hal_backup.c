/* GPL-3.0-only. RM0090 PWR/RTC backup access, no calendar or wall-clock use. */
#include "hal_backup.h"
static bool wait_bit(volatile uint32_t *reg,uint32_t mask)
{
    uint32_t remaining=4000000u;
    while(!(*reg&mask)) if(--remaining==0) return false;
    return true;
}
bool hal_backup_epoch(volatile uint32_t *rcc,volatile uint32_t *pwr,
                      volatile uint32_t *bkp,uint8_t *epoch)
{
    rcc[0x40/4]|=1u<<28; (void)rcc[0x40/4]; /* PWREN */
    pwr[0]|=1u<<8; /* DBP: write access can lag the APB write */
    if(!wait_bit(&pwr[0],1u<<8)) return false;
    if(!(rcc[0x70/4]&(3u<<8)) || (rcc[0x70/4]&(3u<<8))==(2u<<8)) {
        rcc[0x74/4]|=1u; /* LSION; no new crystal required */
        if(!wait_bit(&rcc[0x74/4],2u)) return false;
        if(!(rcc[0x70/4]&(3u<<8))) rcc[0x70/4]|=2u<<8;
    }
    rcc[0x70/4]|=1u<<15; (void)rcc[0x70/4]; /* RTCEN before BKP access */
    *epoch=(uint8_t)(*bkp+1u); *bkp=*epoch;
    return *bkp==*epoch;
}

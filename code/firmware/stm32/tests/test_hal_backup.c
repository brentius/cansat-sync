/* GPL-3.0-only. Boot epoch survives warm reset without backup-domain reset.
 * CS_SOURCES: hal/hal_backup.c */
#include "cs_test.h"
#include "../hal/hal_backup.h"
CS_TEST(cold_and_warm_epochs)
{
    uint32_t rcc[32]={0},pwr[2]={0},bkp=254; uint8_t epoch=0;
    rcc[0x74/4]=2; /* model LSI ready */
    CS_CHECK(hal_backup_epoch(rcc,pwr,&bkp,&epoch));
    CS_CHECK_EQ(epoch,255); CS_CHECK_EQ(bkp,255);
    CS_CHECK(rcc[0x40/4]&(1u<<28)); CS_CHECK(pwr[0]&(1u<<8));
    CS_CHECK_EQ(rcc[0x70/4],(2u<<8)|(1u<<15));
    CS_CHECK(hal_backup_epoch(rcc,pwr,&bkp,&epoch));
    CS_CHECK_EQ(epoch,0); CS_CHECK_EQ(bkp,0);
    /* Preserve an existing source and unrelated backup-domain bits. */
    rcc[0x70/4]=(1u<<8)|1u; bkp=42;
    CS_CHECK(hal_backup_epoch(rcc,pwr,&bkp,&epoch));
    CS_CHECK_EQ(epoch,43); CS_CHECK_EQ(rcc[0x70/4],(1u<<8)|(1u<<15)|1u);
}
CS_TEST(lsi_failure_keeps_epoch)
{
    uint32_t rcc[32]={0},pwr[2]={0},bkp=77; uint8_t epoch=99;
    CS_CHECK(!hal_backup_epoch(rcc,pwr,&bkp,&epoch));
    CS_CHECK_EQ(bkp,77); CS_CHECK_EQ(epoch,99);
}
CS_MAIN(cold_and_warm_epochs,lsi_failure_keeps_epoch)

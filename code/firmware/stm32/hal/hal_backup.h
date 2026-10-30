/* GPL-3.0-only. RM0090 backup-domain access sequence for board_epoch(). */
#ifndef HAL_BACKUP_H
#define HAL_BACKUP_H
#include <stdbool.h>
#include <stdint.h>
/* RCC/PWR register blocks and RTC_BKP0R. Does not reset the backup domain.
 * A fresh domain selects LSI; existing RTCSEL is preserved. Boot-only. */
bool hal_backup_epoch(volatile uint32_t *rcc,volatile uint32_t *pwr,
                      volatile uint32_t *bkp,uint8_t *epoch);
#endif

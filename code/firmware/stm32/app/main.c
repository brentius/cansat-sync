/* GPL-3.0-only. Candidate target entry point, same loop as SITL. */
#include "app.h"
static app_t app;
int main(void)
{
    board_init();
    if(!app_init(&app)) { for(;;) { board_led(BOARD_LED_FAULT,true); board_idle(); } }
    for(;;) app_run_once(&app);
}

/* GPL-3.0-only. Host CLI scenarios and binary wire/NOR output for Pi replay. */
#include "scenario.h"
#include <stdio.h>
#include <stdlib.h>
static app_t app;
static app_scenario_t scenario;
typedef struct { FILE *link,*radio; } files_t;
static void capture(void *ctx,int channel,const uint8_t *p,size_t n)
{ files_t *f=ctx; FILE *out=channel==BOARD_SITL_LINK?f->link:channel==BOARD_SITL_RADIO?f->radio:NULL;
  if(out && fwrite(p,1,n,out)!=n) { perror("SITL write"); exit(2); } }
static FILE *open_output(const char *dir,const char *name,const char *suffix)
{ char path[1024]; if(snprintf(path,sizeof path,"%s/%s.%s",dir,name,suffix)>=(int)sizeof path) return NULL; return fopen(path,"wb"); }
int main(int argc,char **argv)
{
    const char *name=argc>1?argv[1]:"flight",*dir=argc>2?argv[2]:"code/firmware/stm32/build/sitl";
    bool failures=strcmp(name,"failures")==0,reset=strcmp(name,"reset")==0,wrap=strcmp(name,"wrap")==0,budget=strcmp(name,"budget")==0;
    if(strcmp(name,"flight") && !failures && !reset && !wrap && !budget) return 2;
    files_t files={open_output(dir,name,"link.bin"),open_output(dir,name,"radio.bin")};
    if(!files.link || !files.radio) { perror("SITL output"); return 2; }
    board_sitl_cold_start(BOARD_SITL_FLASH_MAX); board_init();
    if(wrap) board_sitl_set_time(UINT64_C(4290000000));
    app_scenario_init(&scenario); scenario.capture=capture; scenario.capture_ctx=&files;
    scenario.pi_enabled=!failures; scenario.gnss_enabled=!failures; scenario.sensor_failures=failures;
    board_sitl_set_sink(app_scenario_sink,&scenario); if(!app_init(&app)) return 3;
    bool reset_done=false; uint32_t prelaunch_pages=0,flight_pages=0,landed_pages=0;
    double end=budget?3760.0:failures?350.0:470.0;
    uint64_t elapsed=0;
    while(elapsed<(uint64_t)(end*1000000.0)) {
        double t=(double)elapsed/1000000.0;
        if(reset && !reset_done && t>=136) {
            app_persist_save(&app); board_sitl_warm_reset(BOARD_RESET_WATCHDOG); board_init();
            cs_link_decoder_init(&scenario.decoder,false);
            if(!app_init(&app) || !app.restored || app.fsm.state!=CS_DESCENT) return 4;
            reset_done=true;
        }
        uint64_t before=board_time_us();
        app_scenario_step(&scenario,t); app_run_once(&app);
        elapsed+=board_time_us()-before;
        uint32_t programs=(uint32_t)board_sitl_stats()->flash_programs;
        if(t<120) prelaunch_pages=programs;
        else if(t<156) flight_pages=programs-prelaunch_pages;
        else landed_pages=programs-prelaunch_pages-flight_pages;
    }
    FILE *flash=open_output(dir,name,"flash.bin"); if(!flash) return 2;
    if(fwrite(board_sitl_flash_data(),1,board_flash_size(),flash)!=board_flash_size()) return 2;
    fclose(flash); fclose(files.link); fclose(files.radio);
    const board_sitl_stats_t *s=board_sitl_stats();
    printf("scenario=%s state=%u states=0x%x restored=%u app_size=%zu imu=%zu fsm=%zu duty=%zu log=%zu\n",
        name,(unsigned)app.fsm.state,scenario.state_mask,app.restored,sizeof app,sizeof app.imu,sizeof app.fsm,sizeof app.downlink,sizeof app.log);
    printf("radio=%u bad=%u link_bytes=%llu imu_frames=%llu restart=%u missed=%u queue_drop=%u suppressed_tx=%u prepare=%u shutdown=%u\n",
        scenario.radio_packets,scenario.radio_bad,(unsigned long long)scenario.link_bytes,(unsigned long long)scenario.frames[CS_LINK_IMU_ID],
        scenario.imu_restart,scenario.imu_missed,app.txqueue.tx_dropped,app.tx_suppressed,scenario.prepare,scenario.shutdown);
    printf("pages=%u phase_bytes=%u/%u/%u log_drop=%u unavailable=%u suppressed=%u protection=%u stopped=%u mount_reads=%u\n",
        app.log.pages_written,prelaunch_pages*256u,flight_pages*256u,landed_pages*256u,app.log.dropped,app.log_unavailable,app.log_suppressed,
        app.log.protected_start,app.log.stopped,app.log.mount_reads);
    printf("watchdog_gap_us=%llu nor_errors=%u power_errors=%u max_reads_per_pass=%u max_nor_ops_per_pass=%u temp_on_us=%llu temp_faults=%u fsm_overflow=%u coalesced=%u\n",
        (unsigned long long)s->watchdog_max_gap_us,s->flash_rule_errors,s->power_order_errors,s->max_reads_per_poll,s->max_flash_ops_per_poll,
        (unsigned long long)app.temp_on_us,app.temp_faults,app.fsm.overflow,app.fsm_coalesced);
    if(app.fsm.state!=CS_LANDED || scenario.radio_bad || s->flash_rule_errors || s->power_order_errors ||
       !scenario.radio_packets || app.fsm.overflow || app.log.stopped) return 1;
    if(!failures && (scenario.state_mask!=15 || scenario.imu_missed>scenario.imu_restart || app.temp_faults || !app.origin_valid || !scenario.prepare || !scenario.shutdown)) return 1;
    return 0;
}

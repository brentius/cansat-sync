/* GPL-3.0-only. */
#include "cs_pimgr.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <string.h>
#else
extern void *memset(void *,int,size_t);
#endif
cs_pimgr_config_t cs_pimgr_default_config(void)
{
    /* Protocol timings are candidate assumptions; power settle is [assumed]. */
    /* Keep Linux powered through boot and heartbeat outages on PAD/in flight.
     * Silence cycling is opt-in recovery policy; zero disables it. */
    cs_pimgr_config_t c={100,3000,0,2000,500,2000,30000}; return c;
}
bool cs_pimgr_init(cs_pimgr_t *m,const cs_pimgr_config_t *config,uint32_t now,uint16_t seq)
{
    cs_pimgr_config_t c=config?*config:cs_pimgr_default_config();
    if(!m || !c.alive_ms || (c.cycle_silence_ms && c.cycle_silence_ms<c.alive_ms) || !c.cmd_period_ms ||
       !c.shutdown_timeout_ms || c.shutdown_timeout_ms>65535000u ||
       c.settle_ms>INT32_MAX || c.power_off_ms>INT32_MAX || c.ack_settle_ms>INT32_MAX ||
       c.cycle_silence_ms>INT32_MAX || c.cmd_period_ms>INT32_MAX) return false;
    memset(m,0,sizeof *m); m->cfg=c; m->phase_ms=now; m->last_rx_ms=now; m->cmd_seq=seq; return true;
}
cs_pimgr_output_t cs_pimgr_poll(cs_pimgr_t *m,const cs_pimgr_input_t *in)
{
    cs_pimgr_output_t out; memset(&out,0,sizeof out); out.oe_high=true; out.stop_triggers=true;
    if(!m || !in) return out;
    uint32_t now=in->now_ms;
    bool was_alive=m->seen_rx && (uint32_t)(now-m->last_rx_ms)<m->cfg.alive_ms;
    if(m->last_flight==CS_PAD && (in->flight==CS_ASCENT || in->flight==CS_DESCENT)) m->launch_silent=!was_alive;
    m->last_flight=in->flight;
    bool connected=m->phase==CS_PI_RUNNING || m->phase==CS_PI_SHUTTING;
    if(in->rx_valid && connected) {
        out.send_hello=!was_alive; m->last_rx_ms=now; m->seen_rx=true; m->launch_silent=false;
    }
    if(in->request_shutdown && m->phase!=CS_PI_STOPPED && m->phase!=CS_PI_SHUTTING) {
        if(m->phase==CS_PI_RUNNING) {
            m->phase=CS_PI_SHUTTING; m->phase_ms=now; m->acked=false; m->done=false; m->sent_cmd=false;
            m->cmd_seq=(uint8_t)(m->cmd_seq+1u);
        } else m->phase=CS_PI_STOPPED;
    }
    switch(m->phase) {
    case CS_PI_WAIT_POWER:
        if(in->pgood) { m->phase=CS_PI_SETTLE; m->phase_ms=now; }
        break;
    case CS_PI_SETTLE:
        if(!in->pgood) { m->phase=CS_PI_WAIT_POWER; m->phase_ms=now; }
        else if((uint32_t)(now-m->phase_ms)>=m->cfg.settle_ms) {
            m->phase=CS_PI_RUNNING; m->last_rx_ms=now; m->seen_rx=false;
        }
        break;
    case CS_PI_RUNNING:
        if(m->cfg.cycle_silence_ms && (uint32_t)(now-m->last_rx_ms)>=m->cfg.cycle_silence_ms &&
           (in->flight==CS_PAD || ((in->flight==CS_ASCENT || in->flight==CS_DESCENT) && m->launch_silent && !m->flight_cycle_used))) {
            if(in->flight!=CS_PAD) m->flight_cycle_used=true;
            m->phase=CS_PI_CYCLE_OFF; m->phase_ms=now; m->seen_rx=false;
        }
        break;
    case CS_PI_CYCLE_OFF:
        if((uint32_t)(now-m->phase_ms)>=m->cfg.power_off_ms) {
            m->phase=CS_PI_WAIT_POWER;
            if(in->pgood) { m->phase=CS_PI_SETTLE; m->phase_ms=now; }
        }
        break;
    case CS_PI_SHUTTING:
        if(in->ack && in->ack->cmd==CS_CMD_SHUTDOWN && in->ack->cmd_seq==(uint8_t)m->cmd_seq) {
            if(in->ack->result==CS_ACK_ACCEPTED || in->ack->result==CS_ACK_DONE) m->acked=true;
            if(in->ack->result==CS_ACK_DONE && !m->done) { m->done=true; m->done_ms=now; }
        }
        if((m->done && (uint32_t)(now-m->done_ms)>=m->cfg.ack_settle_ms) ||
           (uint32_t)(now-m->phase_ms)>=m->cfg.shutdown_timeout_ms) m->phase=CS_PI_STOPPED;
        else if(!m->acked && (!m->sent_cmd || (uint32_t)(now-m->last_cmd_ms)>=m->cfg.cmd_period_ms)) {
            out.send_cmd=true; out.cmd.tick=now*1000u; out.cmd.cmd_seq=(uint8_t)m->cmd_seq;
            out.cmd.cmd=CS_CMD_SHUTDOWN; out.cmd.arg=(uint16_t)((m->cfg.shutdown_timeout_ms+999u)/1000u);
            m->last_cmd_ms=now; m->sent_cmd=true;
        }
        break;
    default: break;
    }
    out.pwr_en=m->phase==CS_PI_SETTLE || m->phase==CS_PI_RUNNING || m->phase==CS_PI_SHUTTING;
    out.oe_high=!(m->phase==CS_PI_RUNNING || m->phase==CS_PI_SHUTTING);
    out.alive=out.pwr_en && m->seen_rx && (uint32_t)(now-m->last_rx_ms)<m->cfg.alive_ms;
    out.stop_triggers=m->phase!=CS_PI_RUNNING;
    return out;
}

/* GPL-3.0-only. Bounded TX frames with non-preemptive IMU priority. */
#include "cs_txqueue.h"
/* The bare Zig target supplies no libc headers; these are standard C99
 * runtime declarations, resolved by the final firmware runtime/libm. */
#if __STDC_HOSTED__
#include <string.h>
#else
extern void *memcpy(void *,const void *,size_t);
extern void *memset(void *,int,size_t);
extern void *memmove(void *,const void *,size_t);
#endif
void cs_txqueue_init(cs_txqueue_t *q) { memset(q,0,sizeof *q); }
static void remove_frame(cs_txqueue_t *q,size_t at) {
    if(at+1<q->count) memmove(q->frames+at,q->frames+at+1,(q->count-at-1)*sizeof q->frames[0]);
    q->count--;
}
bool cs_txqueue_push(cs_txqueue_t *q,const uint8_t *wire,size_t len,bool imu) {
    size_t at, first=q->active ? 1u : 0u;
    cs_txqueue_frame_t *f;
    if(!wire || !len || len>CS_LINK_MAX_WIRE) { q->tx_dropped++;return false; }
    if(q->count==CS_TXQUEUE_CAPACITY) {
        size_t victim=q->count;
        if(imu) {
            size_t i=q->count;
            while(i>first) { i--; if(!q->frames[i].imu) { victim=i;break; } }
        }
        q->tx_dropped++;
        if(victim==q->count) return false;
        remove_frame(q,victim);
    }
    at=q->count;
    if(imu) { at=first;while(at<q->count && q->frames[at].imu) at++; }
    if(at<q->count) memmove(q->frames+at+1,q->frames+at,(q->count-at)*sizeof q->frames[0]);
    f=&q->frames[at]; memcpy(f->data,wire,len); f->len=len;f->imu=imu;q->count++;
    return true;
}
const uint8_t *cs_txqueue_peek(cs_txqueue_t *q,size_t *len) {
    if(!len) return NULL;
    *len=0;
    if(!q->count) return NULL;
    q->active=true;*len=q->frames[0].len-q->offset;return q->frames[0].data+q->offset;
}
bool cs_txqueue_consume(cs_txqueue_t *q,size_t len) {
    if(!q->active || !q->count || len>q->frames[0].len-q->offset) return false;
    q->offset+=len;
    if(q->offset==q->frames[0].len) { remove_frame(q,0);q->offset=0;q->active=false; }
    return true;
}

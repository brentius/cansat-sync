/* GPL-3.0-only. Protocol section 6 integrals; double u, bounded async histories. */
#include "cs_imu.h"
/* The bare Zig target supplies no libc headers; these are standard C99
 * runtime declarations, resolved by the final firmware runtime/libm. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern void *memcpy(void *,const void *,size_t);
extern void *memset(void *,int,size_t);
extern void *memmove(void *,const void *,size_t);
extern double floor(double);
extern double fmod(double,double);
extern double sqrt(double);
extern double sin(double);
extern double cos(double);
extern double acos(double);
#define isfinite(x) __builtin_isfinite(x)
#endif

typedef char cs_imu_history_power_of_two[(CS_IMU_HISTORY&(CS_IMU_HISTORY-1u))==0u?1:-1];
/* i-th oldest retained node of each ring. */
#define GYRO(s,i) ((s)->gyro[((s)->gyro_first+(i))&(CS_IMU_HISTORY-1u)])
#define ACCEL(s,i) ((s)->accel[((s)->accel_first+(i))&(CS_IMU_HISTORY-1u)])
static bool forward(uint32_t a,uint32_t b) { uint32_t d=a-b;return d!=0 && d<0x80000000u; }
static bool finite3(const float v[3]) { return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
static bool reject(cs_imu_t *s,uint8_t flags) { s->rejected++;s->flags|=flags;return false; }
static void normalise(double q[4]) {
    size_t i;double n=0;for(i=0;i<4;i++) n+=q[i]*q[i];n=sqrt(n);for(i=0;i<4;i++) q[i]/=n;
}
static void rotate(const double q[4],const double v[3],double out[3]) {
    double t[3]={2*(q[2]*v[2]-q[3]*v[1]),2*(q[3]*v[0]-q[1]*v[2]),2*(q[1]*v[1]-q[2]*v[0])};
    out[0]=v[0]+q[0]*t[0]+q[2]*t[2]-q[3]*t[1];
    out[1]=v[1]+q[0]*t[1]+q[3]*t[0]-q[1]*t[2];
    out[2]=v[2]+q[0]*t[2]+q[1]*t[1]-q[2]*t[0];
}
static void advance(cs_imu_t *s,const float rate[3],double h) {
    double b[3],p[3],d[4],q[4],angle,k=h*h/12.0;size_t i;
    for(i=0;i<3;i++) b[i]=(double)rate[i];
    p[0]=h*(s->rate[0]+b[0])/2+k*(s->rate[1]*b[2]-s->rate[2]*b[1]);
    p[1]=h*(s->rate[1]+b[1])/2+k*(s->rate[2]*b[0]-s->rate[0]*b[2]);
    p[2]=h*(s->rate[2]+b[2])/2+k*(s->rate[0]*b[1]-s->rate[1]*b[0]);
    angle=sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
    k=angle<1e-12 ? 0.5 : sin(angle/2)/angle;
    d[0]=cos(angle/2);for(i=0;i<3;i++) d[i+1]=p[i]*k;
    memcpy(q,s->q,sizeof q);
    s->q[0]=q[0]*d[0]-q[1]*d[1]-q[2]*d[2]-q[3]*d[3];
    s->q[1]=q[0]*d[1]+q[1]*d[0]+q[2]*d[3]-q[3]*d[2];
    s->q[2]=q[0]*d[2]-q[1]*d[3]+q[2]*d[0]+q[3]*d[1];
    s->q[3]=q[0]*d[3]+q[1]*d[2]-q[2]*d[1]+q[3]*d[0];
    normalise(s->q);
    memcpy(s->rate,b,sizeof b);
}
void cs_imu_init(cs_imu_t *s,uint8_t epoch) {
    memset(s,0,sizeof *s);s->q[0]=1;s->epoch=epoch;s->flags=CS_IMU_RESTART;
}
bool cs_imu_add_sample(cs_imu_t *s,const float rate[3],const float force[3],double h,uint8_t flags) {
    size_t i;double start[3],end[3];
    if(!s) return false;
    if(s->mode==2 || !finite3(rate) || !finite3(force) || !isfinite(h) || h<0 || h>=2147.483648 || (s->primed && h==0))
        return reject(s,CS_IMU_GYRO_FAULT|CS_IMU_ACCEL_FAULT);
    s->mode=1;s->flags|=flags;s->n_gyro++;s->n_accel++;
    if(!s->primed) {
        for(i=0;i<3;i++) { s->rate[i]=(double)rate[i];s->force[i]=(double)force[i]; }
        s->primed=true;return true;
    }
    rotate(s->q,s->force,start);advance(s,rate,h);
    for(i=0;i<3;i++) s->force[i]=(double)force[i];
    rotate(s->q,s->force,end);
    for(i=0;i<3;i++) s->u[i]+=h*(start[i]+end[i])/2;
    return true;
}
static bool attitude_at(const cs_imu_t *s,uint32_t tick,double q[4]) {
    size_t i=s->gyro_count,j;
    /* Ticks strictly increase, so at most one node matches or brackets; scan
     * from the newest, where accel samples normally fall. */
    while(i) {
        const cs_imu_gyro_node_t *g0,*g1;
        i--;g0=&GYRO(s,i);g1=&GYRO(s,i+1); /* g1 is read only when i+1<gyro_count */
        if(g0->tick==tick) { memcpy(q,g0->q,4*sizeof(double));return true; }
        if(i+1<s->gyro_count && forward(tick,g0->tick) && forward(g1->tick,tick)) {
            const double *a=g0->q,*b=g1->q;
            double f=(double)(uint32_t)(tick-g0->tick)/(double)(uint32_t)(g1->tick-g0->tick);
            double dot=0,sign=1,wa,wb;
            for(j=0;j<4;j++) dot+=a[j]*b[j];
            if(dot<0) { sign=-1;dot=-dot; }
            if(dot>0.9995) { wa=1-f;wb=f; }
            else { double angle=acos(dot);wa=sin((1-f)*angle)/sin(angle);wb=sin(f*angle)/sin(angle); }
            for(j=0;j<4;j++) q[j]=wa*a[j]+wb*sign*b[j];
            normalise(q);return true;
        }
    }
    return false;
}
static void pending_remove(cs_imu_t *s) {
    s->pending_count--;
    if(s->pending_count) memmove(s->pending,s->pending+1,s->pending_count*sizeof s->pending[0]);
}
static void process_accel(cs_imu_t *s) {
    while(s->pending_count && s->gyro_count) {
        cs_imu_pending_t sample=s->pending[0];double q[4],f[3];size_t i;
        cs_imu_accel_node_t *node;
        if(forward(GYRO(s,0).tick,sample.tick)) {
            (void)reject(s,CS_IMU_ACCEL_MISSED);pending_remove(s);continue;
        }
        if(!attitude_at(s,sample.tick,q)) return;
        if(s->accel_count==CS_IMU_HISTORY) {
            /* Preserve unreported health information while retiring history. */
            if(ACCEL(s,0).count>s->sent_accel) s->flags|=ACCEL(s,0).flags;
            s->accel_first=(s->accel_first+1u)&(CS_IMU_HISTORY-1u);s->accel_count--;
        }
        for(i=0;i<3;i++) f[i]=(double)sample.force[i];
        node=&ACCEL(s,s->accel_count);node->tick=sample.tick;node->flags=sample.flags;
        rotate(q,f,node->f);
        if(s->accel_count) {
            const cs_imu_accel_node_t *prev=&ACCEL(s,s->accel_count-1u);
            double dt=(double)(uint32_t)(node->tick-prev->tick)*1e-6;
            for(i=0;i<3;i++) s->u[i]+=dt*(prev->f[i]+node->f[i])/2;
        }
        memcpy(node->u,s->u,sizeof node->u);
        node->count=++s->n_accel;s->accel_count++;pending_remove(s);
    }
}
bool cs_imu_add_gyro(cs_imu_t *s,uint32_t tick,const float rate[3],uint8_t flags) {
    cs_imu_gyro_node_t *node;size_t i;
    if(!s) return false;
    if(s->mode==1 || !finite3(rate)) return reject(s,CS_IMU_GYRO_FAULT);
    if(s->primed && !forward(tick,s->last_gyro_tick)) return reject(s,CS_IMU_GYRO_MISSED);
    s->mode=2;
    if(s->primed) advance(s,rate,(double)(uint32_t)(tick-s->last_gyro_tick)*1e-6);
    else { for(i=0;i<3;i++) s->rate[i]=(double)rate[i];s->primed=true; }
    s->last_gyro_tick=tick;
    if(s->gyro_count==CS_IMU_HISTORY) {
        if(GYRO(s,0).count>s->sent_gyro) s->flags|=GYRO(s,0).flags;
        s->gyro_first=(s->gyro_first+1u)&(CS_IMU_HISTORY-1u);s->gyro_count--;
    }
    node=&GYRO(s,s->gyro_count);s->gyro_count++;node->tick=tick;node->count=++s->n_gyro;node->flags=flags;
    memcpy(node->q,s->q,sizeof node->q);process_accel(s);return true;
}
bool cs_imu_add_accel(cs_imu_t *s,uint32_t tick,const float force[3],uint8_t flags) {
    cs_imu_pending_t *node;
    if(!s) return false;
    if(s->mode==1 || !finite3(force)) return reject(s,CS_IMU_ACCEL_FAULT);
    if(s->have_accel_tick && !forward(tick,s->last_accel_tick)) return reject(s,CS_IMU_ACCEL_MISSED);
    s->mode=2;
    if(s->pending_count==CS_IMU_HISTORY) return reject(s,CS_IMU_ACCEL_MISSED);
    node=&s->pending[s->pending_count++];node->tick=tick;node->flags=flags;memcpy(node->force,force,sizeof node->force);
    s->last_accel_tick=tick;s->have_accel_tick=true;process_accel(s);return true;
}
static int32_t wrap_u(double value) {
    double raw=fmod(floor(value*65536.0+0.5),4294967296.0);
    if(raw<0) raw+=4294967296.0;
    if(raw>=2147483648.0) return -1-(int32_t)(4294967295.0-raw);
    return (int32_t)raw;
}
static void wire_snapshot(cs_imu_t *s,uint32_t tick,const double q[4],const double u[3],uint64_t ng,uint64_t na,uint8_t flags,cs_link_imu_t *out) {
    size_t i;uint64_t count;
    out->tick=tick;out->epoch=s->epoch;out->flags=flags;
    for(i=0;i<4;i++) {
        double value=floor(q[i]*1073741824.0+0.5);
        if(value>1073741824.0) value=1073741824.0;
        if(value< -1073741824.0) value=-1073741824.0;
        out->q[i]=(int32_t)value;
    }
    for(i=0;i<3;i++) out->u[i]=wrap_u(u[i]);
    count=ng-s->sent_gyro;out->n_gyro=(uint8_t)(count>255 ? 255 : count);
    count=na-s->sent_accel;out->n_accel=(uint8_t)(count>255 ? 255 : count);
    s->sent_gyro=ng;s->sent_accel=na;s->flags=0;s->snapshot_tick=tick;s->have_snapshot=true;
}
bool cs_imu_snapshot(cs_imu_t *s,uint32_t tick,cs_link_imu_t *out) {
    size_t i,j;const cs_imu_gyro_node_t *g=NULL;uint8_t flags;double u[3];uint64_t na;
    if(!s || !out || !s->primed || (s->have_snapshot && !forward(tick,s->snapshot_tick))) return false;
    if(s->mode==1) { wire_snapshot(s,tick,s->q,s->u,s->n_gyro,s->n_accel,s->flags,out);return true; }
    flags=s->flags;
    for(i=0;i<s->gyro_count;i++) {
        const cs_imu_gyro_node_t *n=&GYRO(s,i);
        if(n->count>s->sent_gyro && !forward(n->tick,tick)) flags|=n->flags;
        if(n->tick==tick) g=n;
    }
    if(!g || !s->accel_count) return false;
    for(i=0;i<s->accel_count;i++) {
        const cs_imu_accel_node_t *a=&ACCEL(s,i),*b=&ACCEL(s,i+1); /* b read only when i+1<accel_count */
        if(a->count>s->sent_accel) flags|=a->flags;
        if(a->tick==tick) { memcpy(u,a->u,sizeof u);na=a->count;break; }
        if(i+1<s->accel_count && forward(tick,a->tick) && forward(b->tick,tick)) {
            double dt=(double)(uint32_t)(tick-a->tick)*1e-6;
            double fraction=(double)(uint32_t)(tick-a->tick)/(double)(uint32_t)(b->tick-a->tick);
            for(j=0;j<3;j++) u[j]=a->u[j]+dt*(2*a->f[j]+fraction*(b->f[j]-a->f[j]))/2;
            flags|=b->flags;na=a->count;break;
        }
    }
    if(i==s->accel_count) return false;
    wire_snapshot(s,tick,g->q,u,g->count,na,flags,out);return true;
}
bool cs_imu_snapshot_latest(cs_imu_t *s,cs_link_imu_t *out) {
    size_t i;
    if(!s || s->mode!=2) return false;
    i=s->gyro_count;
    while(i) { i--;if(cs_imu_snapshot(s,GYRO(s,i).tick,out)) return true; }
    return false;
}

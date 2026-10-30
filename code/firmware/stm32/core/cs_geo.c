/* GPL-3.0-only. */
#include "cs_geo.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#else
extern double sin(double);
extern double cos(double);
extern double sqrt(double);
#endif
static void ecef(double lat, double lon, double h, double out[3])
{
    const double f=1/298.257223563, a=6378137, rad=0.017453292519943295769;
    double s=sin(lat*rad), c=cos(lat*rad), n=a/sqrt(1-f*(2-f)*s*s);
    out[0]=(n+h)*c*cos(lon*rad); out[1]=(n+h)*c*sin(lon*rad); out[2]=(n*(1-f*(2-f))+h)*s;
}
bool cs_geo_enu_from_link(int32_t lat,int32_t lon,int32_t h,int32_t lat0,int32_t lon0,int32_t h0,double out[3])
{
    if(!out || lat < -900000000 || lat>900000000 || lat0 < -900000000 || lat0>900000000 ||
        lon < -1800000000 || lon>1800000000 || lon0 < -1800000000 || lon0>1800000000) return false;
    const double rad=0.017453292519943295769;
    double x[3], x0[3];
    ecef((double)lat*1e-7,(double)lon*1e-7,(double)h*.001,x);
    ecef((double)lat0*1e-7,(double)lon0*1e-7,(double)h0*.001,x0);
    double dx=x[0]-x0[0],dy=x[1]-x0[1],dz=x[2]-x0[2];
    double sl=sin((double)lat0*1e-7*rad),cl=cos((double)lat0*1e-7*rad);
    double so=sin((double)lon0*1e-7*rad),co=cos((double)lon0*1e-7*rad);
    out[0]=-so*dx+co*dy; out[1]=-sl*co*dx-sl*so*dy+cl*dz; out[2]=cl*co*dx+cl*so*dy+sl*dz;
    return true;
}

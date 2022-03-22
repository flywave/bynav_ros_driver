#include <bynav_gps_driver/parsers/galephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {
static int decode_galephemerisb(raw_t *raw)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+OEM4HLEN;
    double tow,sqrtA,af0_fnav,af1_fnav,af2_fnav,af0_inav,af1_inav,af2_inav,tt;
    char *msg;
    int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,dvs_e1b,dvs_e5a,dvs_e5b;
    int toc_fnav,toc_inav,week,sel_nav=0;
    
    trace(3,"decode_galephemerisb: len=%d\n",raw->len);
    
    if (raw->len<OEM4HLEN+220) {
        trace(2,"oem4 galephemrisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn       =U4(p);   p+=4;
    rcv_fnav  =U4(p)&1; p+=4;
    rcv_inav  =U4(p)&1; p+=4;
    svh_e1b   =U1(p)&3; p+=1;
    svh_e5a   =U1(p)&3; p+=1;
    svh_e5b   =U1(p)&3; p+=1;
    dvs_e1b   =U1(p)&1; p+=1;
    dvs_e5a   =U1(p)&1; p+=1;
    dvs_e5b   =U1(p)&1; p+=1;
    eph.sva   =U1(p);   p+=1+1; /* SISA index */
    eph.iode  =U4(p);   p+=4;   /* IODNav */
    eph.toes  =U4(p);   p+=4;
    sqrtA     =R8(p);   p+=8;
    eph.deln  =R8(p);   p+=8;
    eph.M0    =R8(p);   p+=8;
    eph.e     =R8(p);   p+=8;
    eph.omg   =R8(p);   p+=8;
    eph.cuc   =R8(p);   p+=8;
    eph.cus   =R8(p);   p+=8;
    eph.crc   =R8(p);   p+=8;
    eph.crs   =R8(p);   p+=8;
    eph.cic   =R8(p);   p+=8;
    eph.cis   =R8(p);   p+=8;
    eph.i0    =R8(p);   p+=8;
    eph.idot  =R8(p);   p+=8;
    eph.OMG0  =R8(p);   p+=8;
    eph.OMGd  =R8(p);   p+=8;
    toc_fnav  =U4(p);   p+=4;
    af0_fnav  =R8(p);   p+=8;
    af1_fnav  =R8(p);   p+=8;
    af2_fnav  =R8(p);   p+=8;
    toc_inav  =U4(p);   p+=4;
    af0_inav  =R8(p);   p+=8;
    af1_inav  =R8(p);   p+=8;
    af2_inav  =R8(p);   p+=8;
    eph.tgd[0]=R8(p);   p+=8; /* BGD: E5A-E1 (s) */
    eph.tgd[1]=R8(p);         /* BGD: E5B-E1 (s) */
    eph.iodc  =eph.iode;
    eph.svh   =(svh_e5b<<7)|(dvs_e5b<<6)|(svh_e5a<<4)|(dvs_e5a<<3)|
               (svh_e1b<<1)|dvs_e1b;
    
    /* ephemeris selection (0:INAV,1:FNAV) */
    if      (strstr(raw->opt,"-GALINAV")) sel_nav=0;
    else if (strstr(raw->opt,"-GALFNAV")) sel_nav=1;
    else if (!rcv_inav&&rcv_fnav) sel_nav=1;
    
    eph.A     =sqrtA*sqrtA;
    eph.f0    =sel_nav?af0_fnav:af0_inav;
    eph.f1    =sel_nav?af1_fnav:af1_inav;
    eph.f2    =sel_nav?af2_fnav:af2_inav;
    
    /* set data source defined in rinex 3.03 */
    eph.code=(sel_nav==0)?((1<<0)|(1<<9)):((1<<1)|(1<<8));
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d iod=%3d toes=%6.0f",prn,eph.iode,eph.toes);
    }
    if (!(eph.sat=satno(SYS_GAL,prn))) {
        trace(2,"oemv galephemeris satellite error: prn=%d\n",prn);
        return -1;
    }
    tow=time2gpst(raw->time,&week);
    eph.week=week; /* gps-week = gal-week */
    eph.toe=gpst2time(eph.week,eph.toes);
    
    /* for week-handover problem */
    tt=timediff(eph.toe,raw->time);
    if      (tt<-302400.0) eph.week++;
    else if (tt> 302400.0) eph.week--;
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=adjweek(eph.toe,sel_nav?toc_fnav:toc_inav);
    eph.ttr=adjweek(eph.toe,tow);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].code==eph.code) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;
    return 2;
}
} // namespace bynav_gps_driver
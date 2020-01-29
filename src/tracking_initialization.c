/**
 * @file tracking_initialization.c
 * @date January 29, 2020
 * @author Shu Wang <shuwang1@outlook.com>
 * @brief initialize the acquisition data structure
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * \copyright Private Uses are permitted but commercial uses shall be licensed. See LICENSE file.
 ***/
#include "measurement_engine.h"

/* initialize tracking struct --------------------------------------------------
* set value to tracking struct
* args   : int    sat       I   satellite number
*          int    ctype     I   code type (CTYPE_L1CA...)
*          double ctime     I   code period (s)
*          sdrtrk_t *trk    I/0 tracking struct
* return : int                  0:okay -1:error
*-----------------------------------------------------------------------------*/
int initialize_tracking_structure( int sat, int ctype, double ctime, sdrtrk_t *trk )
{

    int i,prn;
    int ctimems=(int)(ctime*1000);
    int sys=satsys(sat,&prn);

    /* set tracking parameter */
    inittrkprmstruct(trk);

    /* correlation point */
    trk->corrn=sdrini.trkcorrn;
    trk->corrp=(int *)malloc(sizeof(int)*trk->corrn);
    for (i=0;i<trk->corrn;i++) {
        trk->corrp[i]=sdrini.trkcorrd*(i+1);
        if (trk->corrp[i]==sdrini.trkcorrp){
            trk->ne=2*(i+1)-1; /* Early */
            trk->nl=2*(i+1);   /* Late */
        }
    }
    /* correlation point for plot */
    (trk->corrx=(double *)calloc(2*trk->corrn+1,sizeof(double)));
    for (i=1;i<=trk->corrn;i++) {
        trk->corrx[2*i-1]=-sdrini.trkcorrd*i;
        trk->corrx[2*i  ]= sdrini.trkcorrd*i;
    }

    trk->II     =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->QQ     =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->oldI   =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->oldQ   =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->sumI   =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->sumQ   =(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->oldsumI=(double*)calloc(1+2*trk->corrn,sizeof(double));
    trk->oldsumQ=(double*)calloc(1+2*trk->corrn,sizeof(double));

    if (ctype==CTYPE_L1CA)   trk->loop=LOOP_L1CA;
    if (ctype==CTYPE_G1)     trk->loop=LOOP_G1;
    if (ctype==CTYPE_E1B)    trk->loop=LOOP_E1B;
    if (ctype==CTYPE_L1SAIF) trk->loop=LOOP_SBAS;
    if (ctype==CTYPE_L1SBAS) trk->loop=LOOP_SBAS;
    if (ctype==CTYPE_B1I&&prn>5 ) trk->loop=LOOP_B1I;
    if (ctype==CTYPE_B1I&&prn<=5) trk->loop=LOOP_B1IG;
    
    /* for LEX */
    if (sys==SYS_QZS&&ctype==CTYPE_L1CA&&sdrini.nchL6) trk->loop=LOOP_LEX;
    
    /* loop interval (ms) */
    trk->loopms=trk->loop*ctimems;

    if (!trk->II||!trk->QQ||!trk->oldI||!trk->oldQ||!trk->sumI||!trk->sumQ||
        !trk->oldsumI||!trk->oldsumQ) {
        debug_print("error: inittrkstruct memory allocation\n");
        return -1;
    }
    return 0;


}


/* initialize navigation struct ------------------------------------------------
* set value to navigation struct
* args   : int sys          I   system type (SYS_GPS...)
*          int ctype        I   code type (CTYPE_L1CA...)
*          int prn          I   PRN (or SV) number
*          navigation_t *nav    I/0 navigation struct
* return : int                  0:okay -1:error
*-----------------------------------------------------------------------------*/
int initialize_navigation_structure(int sys, int ctype, int prn, navigation_t *nav)
{
    int i;
    int pre_l1ca[8]= { 1,-1,-1,-1, 1,-1, 1, 1}; /* L1CA preamble*/
    int pre_e1b[10]= { 1,-1, 1,-1,-1, 1, 1, 1, 1, 1}; /* E1B preamble */
    int pre_g1[30]=  {-1,-1,-1,-1,-1, 1, 1, 1,-1,-1,
                       1,-1,-1,-1, 1,-1, 1,-1, 1, 1,
                       1, 1,-1, 1, 1,-1, 1,-1,-1, 1}; /* G1 preamble */
    int pre_b1i[11]= {-1,-1,-1, 1, 1, 1,-1, 1, 1,-1, 1}; /* B1I preamble */
    int pre_sbs[24]= { 1,-1, 1,-1, 1, 1,-1,-1,-1, 1,
                       1,-1,-1, 1,-1, 1,-1,-1, 1, 1,
                       1 -1,-1, 1}; /* SBAS L1/QZS L1SAIF preamble */

    int poly[2]={V27POLYA,V27POLYB};

    nav->ctype=ctype;
    nav->sdreph.ctype=ctype;
    nav->sdreph.prn=prn;
    nav->sdreph.eph.iodc=-1;

    /* GPS/QZS L1CA */
    if (ctype==CTYPE_L1CA) {
        nav->rate=NAVRATE_L1CA;
        nav->flen=NAVFLEN_L1CA;
        nav->addflen=NAVADDFLEN_L1CA;
        nav->prelen=NAVPRELEN_L1CA;
        nav->sdreph.cntth=NAVEPHCNT_L1CA;
        nav->update=(int)(nav->flen*nav->rate);
        memcpy(nav->prebits,pre_l1ca,sizeof(int)*nav->prelen);

        /* overlay code (all 1) */
        nav->ocode=(short *)calloc(nav->rate,sizeof(short));
        for (i=0;i<nav->rate;i++) nav->ocode[i]=1;
    }
    /* SBAS/QZS L1SAIF */
    if (ctype==CTYPE_L1SAIF||ctype==CTYPE_L1SBAS) {
        nav->rate=NAVRATE_SBAS;
        nav->flen=NAVFLEN_SBAS;
        nav->addflen=NAVADDFLEN_SBAS;
        nav->prelen=NAVPRELEN_SBAS;
        nav->sdreph.cntth=NAVEPHCNT_SBAS;
        nav->update=(int)(nav->flen/3*nav->rate);
        memcpy(nav->prebits,pre_sbs,sizeof(int)*nav->prelen);

        /* create fec */
        if((nav->fec=create_viterbi27_port(NAVFLEN_SBAS/2))==NULL) {
            debug_print("error: create_viterbi27 failed\n");
            return -1;
        }
        /* set polynomial */
        set_viterbi27_polynomial_port(poly);

        /* overlay code (all 1) */
        nav->ocode=(short *)calloc(nav->rate,sizeof(short));
        for (i=0;i<nav->rate;i++) nav->ocode[i]=1;
    }
    /* GLONASS G1 */
    if (ctype==CTYPE_G1) {
        nav->rate=NAVRATE_G1;
        nav->flen=NAVFLEN_G1;
        nav->addflen=NAVADDFLEN_G1;
        nav->prelen=NAVPRELEN_G1;
        nav->sdreph.cntth=NAVEPHCNT_G1;
        nav->update=(int)(nav->flen*nav->rate);
        memcpy(nav->prebits,pre_g1,sizeof(int)*nav->prelen);
        nav->sdreph.geph.frq=prn; /* glonass frequency number */

        /* overlay code (all 1) */
        nav->ocode=(short *)calloc(nav->rate,sizeof(short));
        for (i=0;i<nav->rate;i++) nav->ocode[i]=1;
    }
    /* Galileo E1B */
    if (ctype==CTYPE_E1B) {
        nav->rate=NAVRATE_E1B;
        nav->flen=NAVFLEN_E1B;
        nav->addflen=NAVADDFLEN_E1B;
        nav->prelen=NAVPRELEN_E1B;
        nav->sdreph.cntth=NAVEPHCNT_E1B;
        nav->update=(int)(nav->flen*nav->rate);
        memcpy(nav->prebits,pre_e1b,sizeof(int)*nav->prelen);

        /* create fec */
        if((nav->fec=create_viterbi27_port(120))==NULL) {
            debug_print("error: create_viterbi27 failed\n");
            return -1;
        }
        /* set polynomial */
        set_viterbi27_polynomial_port(poly);

        /* overlay code (all 1) */
        nav->ocode=(short *)calloc(nav->rate,sizeof(short));
        for (i=0;i<nav->rate;i++) nav->ocode[i]=1;
    }
    /* BeiDou B1I */
    if (ctype==CTYPE_B1I) {
        /* MEO/IGSO (D1 NAV) */
        if (prn>5) {
            nav->rate=NAVRATE_B1I;
            nav->flen=NAVFLEN_B1I;
            nav->addflen=NAVADDFLEN_B1I;
            nav->prelen=NAVPRELEN_B1I;
            nav->sdreph.cntth=NAVEPHCNT_B1I;
            nav->update=(int)(nav->flen*nav->rate);
            memcpy(nav->prebits,pre_b1i,sizeof(int)*nav->prelen);
            
            /* secondary code generation */
            nav->ocode=gencode(-1,CTYPE_NH20,NULL,NULL);

        /* GEO (D2 NAV) */
        } else {
            nav->rate=NAVRATE_B1IG;
            nav->flen=NAVFLEN_B1IG;
            nav->addflen=NAVADDFLEN_B1IG;
            nav->prelen=NAVPRELEN_B1IG;
            nav->sdreph.cntth=NAVEPHCNT_B1IG;
            nav->update=(int)(nav->flen*nav->rate);
            memcpy(nav->prebits,pre_b1i,sizeof(int)*nav->prelen);

            /* overlay code (all 1) */
            nav->ocode=(short *)calloc(nav->rate,sizeof(short));
            for (i=0;i<nav->rate;i++) nav->ocode[i]=1;
        }
    }

    if (!(nav->bitsync= (int *)calloc(nav->rate,sizeof(int))) || 
        !(nav->fbits=   (int *)calloc(nav->flen+nav->addflen,sizeof(int))) ||
        !(nav->fbitsdec=(int *)calloc(nav->flen+nav->addflen,sizeof(int)))) {
            debug_print("error: initnavstruct memory alocation\n");
            return -1;
    }
    return 0;
}
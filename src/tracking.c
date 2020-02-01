/*------------------------------------------------------------------------------
* sdrtrk.c : SDR tracking functions
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
*-----------------------------------------------------------------------------*/
#include "measurement_engine.h"

/* sdr tracking function -------------------------------------------------------
* sdr tracking function called from sdr channel thread
* args   : sdrch_t *sdr      I/O sdr channel struct
*          uint64_t buffloc  I   buffer location
*          uint64_t cnt      I   counter of sdr channel thread
* return : uint64_t              current buffer location
*-----------------------------------------------------------------------------*/
extern uint64_t tracking(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt)
{
    char *data=NULL;
    uint64_t bufflocnow;

    sdr->flagtrk=OFF;

    /* memory allocation */
    data=(char*)malloc(sizeof(char)*(sdr->nsamp+100)*sdr->dtype);

    /* current buffer location */
    mlock(hreadmtx);
    bufflocnow=sdrstat.fendbuffsize*sdrstat.buffcnt-sdr->nsamp; 
    unmlock(hreadmtx);

    if (bufflocnow>buffloc) {
        sdr->currnsamp=(int)((sdr->clen-sdr->trk.remcode)/
            (sdr->trk.codefreq/sdr->f_sf));
        rcvgetbuff(&sdrini,buffloc,sdr->currnsamp,sdr->ftype,sdr->dtype,data);

        memcpy(sdr->trk.oldI,sdr->trk.II,1+2*sdr->trk.corrn*sizeof(double));
        memcpy(sdr->trk.oldQ,sdr->trk.QQ,1+2*sdr->trk.corrn*sizeof(double));
        sdr->trk.oldremcode=sdr->trk.remcode;
        sdr->trk.oldremcarr=sdr->trk.remcarr;
        /* correlation */
        correlator(data,sdr->dtype,sdr->ti,sdr->currnsamp,sdr->trk.carrfreq,
            sdr->trk.oldremcarr,sdr->trk.codefreq, sdr->trk.oldremcode,
            sdr->trk.corrp,sdr->trk.corrn,sdr->trk.QQ,sdr->trk.II,
            &sdr->trk.remcode,&sdr->trk.remcarr,sdr->code,sdr->clen);

        /* navigation data */
        navigation(sdr,buffloc,cnt);

        sdr->flagtrk=ON;
    } else {
        sleepms(1);
    }
    free(data);
    return bufflocnow;
}
/* cumulative sum of correlation output ----------------------------------------
* phase/frequency lock loop (2nd order PLL with 1st order FLL)
* carrier frequency is computed
* args   : sdrtrk_t trk     I/0 sdr tracking struct
*          int    polarity  I   polarity caused by secondary code
*          int    flag1     I   reset flag 1
*          int    flag2     I   reset flag 2
* return : none
*-----------------------------------------------------------------------------*/
extern void cumsumcorr(sdrtrk_t *trk, int polarity)
{
    int i;
    for (i=0;i<1+2*trk->corrn;i++) {
        trk->II[i]*=polarity;
        trk->QQ[i]*=polarity;

        trk->oldsumI[i]+=trk->oldI[i];
        trk->oldsumQ[i]+=trk->oldQ[i];
        trk->sumI[i]+=trk->II[i];
        trk->sumQ[i]+=trk->QQ[i];
    }
}
extern void clearcumsumcorr(sdrtrk_t *trk)
{
    int i;
    for (i=0;i<1+2*trk->corrn;i++) {
        trk->oldsumI[i]=0;
        trk->oldsumQ[i]=0;
        trk->sumI[i]=0;
        trk->sumQ[i]=0;
    }
}
/* phase/frequency lock loop ---------------------------------------------------
* phase/frequency lock loop (2nd order PLL with 1st order FLL)
* carrier frequency is computed
* args   : sdrch_t *sdr     I/0 sdr channel struct
*          sdrtrkprm_t *prm I   sdr tracking prameter struct
* return : none
*-----------------------------------------------------------------------------*/
extern void pll(sdrch_t *sdr, sdrtrkprm_t *prm, double dt)
{
    double carrErr,freqErr;
    double IP=sdr->trk.sumI[0],QP=sdr->trk.sumQ[0];
    double oldIP=sdr->trk.oldsumI[0],oldQP=sdr->trk.oldsumQ[0];
    double f1,f2;

    /* PLL discriminator */
    if (IP>0)
        carrErr=atan2(QP,IP)/PI;
    else
        carrErr=atan2(-QP,-IP)/PI;

    /* FLL discriminator */
    f1=(IP==0)?    PI/2:atan(QP/IP);
    f2=(oldIP==0)? PI/2:atan(oldQP/oldIP);
    freqErr=f1-f2;

    if (freqErr>PI/2)
        freqErr = PI-freqErr;
    if (freqErr<-PI/2)
        freqErr = -PI-freqErr;
    //freqErr/=DPI;

    /* 2nd order PLL with 1st order FLL */
    sdr->trk.carrNco+=prm->pllaw*(carrErr-sdr->trk.carrErr)+
        prm->pllw2*dt*carrErr+prm->fllw*dt*freqErr;

    sdr->trk.carrfreq=sdr->acq.acqfreq+sdr->trk.carrNco;
    sdr->trk.carrErr=carrErr;
    sdr->trk.freqErr=freqErr;
}
/* delay lock loop -------------------------------------------------------------
* delay lock loop (2nd order DLL)
* code frequency is computed
* args   : sdrch_t *sdr     I/0 sdr channel struct
*          sdrtrkprm_t *prm I   sdr tracking prameter struct
* return : none
*-----------------------------------------------------------------------------*/
extern void dll(sdrch_t *sdr, sdrtrkprm_t *prm, double dt)
{
    double codeErr;
    double IE=sdr->trk.sumI[sdr->trk.ne],IL=sdr->trk.sumI[sdr->trk.nl];
    double QE=sdr->trk.sumQ[sdr->trk.ne],QL=sdr->trk.sumQ[sdr->trk.nl];

    codeErr=(sqrt(IE*IE+QE*QE)-
        sqrt(IL*IL+QL*QL))/(sqrt(IE*IE+QE*QE)+sqrt(IL*IL+QL*QL));

    /* 2nd order DLL */
    sdr->trk.codeNco+=prm->dllaw*(codeErr-sdr->trk.codeErr)+
        prm->dllw2*dt*codeErr;

    /* carrier aiding */
    sdr->trk.codefreq=sdr->crate-sdr->trk.codeNco+
        (sdr->trk.carrfreq-sdr->f_if-sdr->foffset)/(sdr->f_cf/sdr->crate);
    sdr->trk.codeErr=codeErr;
}
/* set observation data --------------------------------------------------------
* calculate doppler/carrier phase/SNR
* args   : sdrch_t *sdr     I   sdr channel struct
*          uint64_t buffloc I   current buffer location
*          uint64_t cnt     I   current counter of sdr channel thread
*          sdrtrk_t trk     I/0 sdr tracking struct
*          int    snrflag   I   SNR calculation flag
* return : none
*-----------------------------------------------------------------------------*/
extern void setobsdata(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt, 
                       sdrtrk_t *trk, int snrflag)
{
    shiftdata(&trk->tow[1],&trk->tow[0],sizeof(double),OBSINTERPN-1);
    shiftdata(&trk->L[1],&trk->L[0],sizeof(double),OBSINTERPN-1);
    shiftdata(&trk->D[1],&trk->D[0],sizeof(double),OBSINTERPN-1);
    shiftdata(&trk->codei[1],&trk->codei[0],sizeof(uint64_t),OBSINTERPN-1);
    shiftdata(&trk->cntout[1],&trk->cntout[0],sizeof(uint64_t),OBSINTERPN-1);
    shiftdata(&trk->remcout[1],&trk->remcout[0],sizeof(double),OBSINTERPN-1);

    trk->tow[0]=sdr->nav.firstsftow+
        (double)(cnt-sdr->nav.firstsfcnt)*sdr->ctime;
    trk->codei[0]=buffloc;
    trk->cntout[0]=cnt;
    trk->remcout[0]=trk->oldremcode*sdr->f_sf/trk->codefreq;

    /* doppler */
    trk->D[0]=-(trk->carrfreq-sdr->f_if-sdr->foffset);

    /* carrier phase */
    if (!trk->flagremcarradd) {
        trk->L[0]-=trk->remcarr/DPI;
        //debug_print("%s cnt=%llu inicarrier=%f m\n",sdr->satstr,cnt,CLIGHT/FREQ1*trk->remcarr/DPI);
        trk->flagremcarradd=ON;
    }

    if (sdr->nav.flagsyncf&&!trk->flagpolarityadd) {
        if (sdr->nav.polarity==1) {
            trk->L[0]+=0.5;
            //debug_print("%s cnt=%llu polarity=0.5\n",sdr->satstr,cnt);
        } else {
            //debug_print("%s cnt=%llu polarity=0.0\n",sdr->satstr,cnt);
        }
        trk->flagpolarityadd=ON;
    }

    trk->L[0]+=trk->D[0]*(trk->loopms*sdr->currnsamp/sdr->f_sf);

    trk->Isum+=fabs(trk->sumI[0]);
    if (snrflag) {
        shiftdata(&trk->S[1],&trk->S[0],sizeof(double),OBSINTERPN-1);
        shiftdata(&trk->codeisum[1],&trk->codeisum[0],
            sizeof(uint64_t),OBSINTERPN-1);

        /* signal to noise ratio */
        trk->S[0]=10*log(trk->Isum/100.0/100.0)+log(500.0)+5;
        trk->codeisum[0]=buffloc;
        trk->Isum=0;
    }
}

/* dot products: d1={dot(a1,b1),dot(a1,b2)},d2={dot(a2,b1),dot(a2,b2)} ---------
* args   : short  *a1       I   input short array
*          short  *a2       I   input short array
*          short  *b1       I   input short array
*          short  *b2       I   input short array
*          int    n         I   number of input data
*          short  *d1       O   output short array
*          short  *d2       O   output short array
* return : none
*-----------------------------------------------------------------------------*/
extern void dot_22(const short *a1, const short *a2, const short *b1,
                   const short *b2, int n, double *d1, double *d2)
{
    const short *p1=a1,*p2=a2,*q1=b1,*q2=b2;


    d1[0]=d1[1]=d2[0]=d2[1]=0.0;

    for (;p1<a1+n;p1++,p2++,q1++,q2++) {
        d1[0]+=(*p1)*(*q1);
        d1[1]+=(*p1)*(*q2);
        d2[0]+=(*p2)*(*q1);
        d2[1]+=(*p2)*(*q2);
    }

}
/* dot products: d1={dot(a1,b1),dot(a1,b2),dot(a1,b3)},d2={...} ----------------
* args   : short  *a1       I   input short array
*          short  *a2       I   input short array
*          short  *b1       I   input short array
*          short  *b2       I   input short array
*          short  *b3       I   input short array
*          int    n         I   number of input data
*          short  *d1       O   output short array
*          short  *d2       O   output short array
* return : none
*-----------------------------------------------------------------------------*/
extern void dot_23(const short *a1, const short *a2, const short *b1,
                   const short *b2, const short *b3, int n, double *d1,
                   double *d2)
{
    const short *p1=a1,*p2=a2,*q1=b1,*q2=b2,*q3=b3;

    d1[0]=d1[1]=d1[2]=d2[0]=d2[1]=d2[2]=0.0;

    for (;p1<a1+n;p1++,p2++,q1++,q2++,q3++) {
        d1[0]+=(*p1)*(*q1);
        d1[1]+=(*p1)*(*q2);
        d1[2]+=(*p1)*(*q3);
        d2[0]+=(*p2)*(*q1);
        d2[1]+=(*p2)*(*q2);
        d2[2]+=(*p2)*(*q3);
    }

}

/* correlator ------------------------------------------------------------------
* multiply sampling data and carrier (I/Q), multiply code (E/P/L), and integrate
* args   : char   *data     I   sampling data vector (n x 1 or 2n x 1)
*          int    dtype     I   sampling data type (1:real,2:complex)
*          double ti        I   sampling interval (s)
*          int    n         I   number of samples
*          double freq      I   carrier frequency (Hz)
*          double phi0      I   carrier initial phase (rad)
*          double crate     I   code chip rate (chip/s)
*          double coff      I   code chip offset (chip)
*          int    s         I   correlator points (sample)
*          short  *I,*Q     O   correlation power I,Q
*                                 I={I_P,I_E1,I_L1,I_E2,I_L2,...,I_Em,I_Lm}
*                                 Q={Q_P,Q_E1,Q_L1,Q_E2,Q_L2,...,Q_Em,Q_Lm}
* return : none
* notes  : see above for data
*-----------------------------------------------------------------------------*/
extern void correlator(const char *data, int dtype, double ti, int n, 
                       double freq, double phi0, double crate, double coff, 
                       int* s, int ns, double *II, double *QQ, double *remc, 
                       double *remp, short* codein, int coden)
{
#define CSCALE        (1.0/32.0)       /* carrier lookup table scale (LSB) */

    short *dataI=NULL,*dataQ=NULL,*code_e=NULL,*code;
    int i;
    int smax=s[ns-1];

    /* 8 is treatment of remainder in SSE2 */
    if( !( dataI = (short *) malloc(sizeof(short)*(n+64)) ) ) {
            debug_print("error: correlator dataI memory allocation\n");
            return;
    }

    if( !( dataQ = (short *) malloc(sizeof(short)*(n+64)) ) ) {
            debug_print("error: correlator dataQ memory allocation\n");
            return;
    }

    if( !( code_e = (short *) malloc(sizeof(short)*(n+2*smax)) ) ) {
            debug_print("error: correlator code_e memory allocation\n");
            return;
    }
    code=code_e+smax;

    /* mix local carrier */
    *remp=mixcarr(data,dtype,ti,n,freq,phi0,dataI,dataQ);

    /* resampling code */
    *remc=rescode(codein,coden,coff,smax,ti*crate,n,code_e);

    /* multiply code and integrate */
    dot_23(dataI,dataQ,code,code-s[0],code+s[0],n,II,QQ);
    for (i=1;i<ns;i++) {
        dot_22(dataI,dataQ,code-s[i],code+s[i],n,II+1+i*2,QQ+1+i*2);
    }
    for (i=0;i<1+2*ns;i++) {
        II[i]*=CSCALE;
        QQ[i]*=CSCALE;
    }
    free(dataI); free(dataQ); free(code_e);
    dataI=dataQ=code_e=NULL;

#undef CSALE
}
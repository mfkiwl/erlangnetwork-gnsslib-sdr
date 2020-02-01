/**
 * @date January 29, 2020
 * @author Shu Wang <shuwang1@outlook.com>
 * @brief statemachine for each satellite under acquisition and tracking
 * 
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * \copyright Private Uses are permitted but commercial uses shall be licensed. See LICENSE file.
 **/

#include "measurement_engine.h"

/* sdr channel thread ----------------------------------------------------------
* sdr channel thread for signal acquisition and tracking  
* args   : void   *arg      I   sdr channel struct
* return : none
* note : This thread handles the acquisition and tracking of one of the signals. 
*        The thread is created at startsdr function.
*-----------------------------------------------------------------------------*/
#define ACQSLEEP      200             /* acquisition process interval (ms) */

extern void *statemachinethread(void *arg)
{
    sdrch_t *sdr=(sdrch_t*)arg;
    sdrplt_t pltacq={0},plttrk={0};
    uint32_t nsamp = sdr->nsamp ;
    uint64_t buffloc=0,bufflocnow=0,cnt=0,loopcnt=0;
    double *acqpower = NULL;
    FILE* fp=NULL;
    char fname[128];
    
    /* create tracking log file */
    if (sdrini.log) {
        sprintf(fname,"%s/log%s.csv", sdrini.logpath, sdr->satstr);
        if((fp=createlog(fname,&sdr->trk))==NULL) {
            debug_print("error: invailed log file: %s\n",fname);
            return THRETVAL;
        }
    }

    /* plot setting */
    if (initpltstruct(&pltacq,&plttrk,sdr)<0) {
        sdrstat.stopflag=ON;
    }
    sleepms( (sdr->no-1) *5000 + 500 );
    debug_print("**** %s aquisition thread %02d coherence=%d fftlen=%d start! ****\n",sdr->satstr, sdr->no, sdr->acq.coherencelen_code[0], sdr->acq.fftlen[0] );

    sdr->flagacq = 0 ;
    while (!sdrstat.stopflag) {

        /* acquisition */
        if( sdr->flagacq < 1 ) {

            /* memory allocation */
            if( acqpower == NULL ) {
                acqpower = (double*) calloc( sizeof(double), nsamp * sdr->acq.numfreq[0] );
            }else{
                memset( acqpower, 0, sizeof(double) * nsamp * sdr->acq.numfreq[0] );
            }

            /* fft correlation */
            buffloc = acquisition( sdr, acqpower );
        
        }

        if( sdr->flagacq > 0 ) {

            if( sdrini.pltacq ) { /* plot aquisition result */

                pltacq.z = acqpower;
                plot( &pltacq ); 

            }

        }else{

            /* display acquisition results */
            if( !sdr->flagtrk ) debug_print("%s, C/N0=%4.1f, peakr=%3.1f, codei=%5d, freq=%8.1f\n",
                sdr->satstr,
                sdr->acq.cn0,
                sdr->acq.peakr,
                sdr->acq.acqcodei,
                sdr->acq.acqfreq - sdr->f_if - sdr->foffset );

            sdr->flagacq = sdr->flagacq > -3 ? -10 : ++(sdr->flagacq) ;
            sleepms( - sdr->flagacq * ACQSLEEP );
        }

        /* tracking */
        if( sdr->flagacq > 0 ) {
            bufflocnow=tracking(sdr,buffloc,cnt);
            if (sdr->flagtrk) {
                
                /* correlation output accumulation */
                cumsumcorr(&sdr->trk,sdr->nav.ocode[sdr->nav.ocodei]);

                sdr->trk.flagloopfilter=0;
                if (!sdr->nav.flagsync) {
                    pll(sdr,&sdr->trk.prm1,sdr->ctime); /* PLL */
                    dll(sdr,&sdr->trk.prm1,sdr->ctime); /* DLL */
                    sdr->trk.flagloopfilter=1;
                }
                else if (sdr->nav.swloop) {
                    pll(sdr,&sdr->trk.prm2,(double)sdr->trk.loopms/1000);
                    dll(sdr,&sdr->trk.prm2,(double)sdr->trk.loopms/1000);
                    sdr->trk.flagloopfilter=2;

                    mlock(hobsmtx);

                    /* calculate observation data */
                    if (loopcnt%(SNSMOOTHMS/sdr->trk.loopms)==0)
                        setobsdata(sdr,buffloc,cnt,&sdr->trk,1);
                    else
                        setobsdata(sdr,buffloc,cnt,&sdr->trk,0);

                    unmlock(hobsmtx);

                    /* plot correator output */
                    if (loopcnt%((int)(plttrk.pltms/sdr->trk.loopms))==0&&
                        sdrini.plttrk&&loopcnt>0) {
                        plttrk.x=sdr->trk.corrx;
                        memcpy(plttrk.y,sdr->trk.sumI,
                            sizeof(double)*(sdr->trk.corrn*2+1));
                        plotthread(&plttrk);
                    }
                    
                    loopcnt++;
                }

                if (sdr->no==1&&cnt%(1000*10)==0)
                    debug_print("process %d sec...\n",(int)cnt/(1000));

                /* write tracking log */
                if (sdrini.log) writelog(fp,&sdr->trk,&sdr->nav);

                if (sdr->trk.flagloopfilter) clearcumsumcorr(&sdr->trk);
                cnt++;
                buffloc+=sdr->currnsamp;
            }
        }
        sdr->trk.buffloc=buffloc;
    }
        
    /* plot termination */
    quitpltstruct(&pltacq,&plttrk);

    /* cloase tracking log file */
    if (sdrini.log) closelog(fp);

    if (sdr->flagacq) {
        debug_print("SDR channel %s thread finished! Delay=%d [ms]\n",
            sdr->satstr, (int) (bufflocnow-buffloc) / nsamp );
    } else {
        debug_print("SDR channel %s thread finished!\n",sdr->satstr);
    }

    return THRETVAL;
}

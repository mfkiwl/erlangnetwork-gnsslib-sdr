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
#define ACQSLEEP      2000             /* acquisition process interval (ms) */

#ifdef WIN32
extern void statemachinethread(void *arg)
#else
extern void *statemachinethread(void *arg)
#endif
{
    sdrch_t *sdr=(sdrch_t*)arg;
    sdrplt_t pltacq={0},plttrk={0};
    uint64_t buffloc=0,bufflocnow=0,cnt=0,loopcnt=0;
    double *acqpower=NULL;
    FILE* fp=NULL;
    char fname[100];
    
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
    sleepms(sdr->no*500);
    debug_print("**** %s sdr thread %d start! ****\n",sdr->satstr,sdr->no);

    sdr->flagacq = 0 ;
    while (!sdrstat.stopflag) {

        /* acquisition */
        if( sdr->flagacq < 1 ) {

            /* memory allocation */
            acqpower = (double*) calloc( sizeof(double), sdr->nsamp * sdr->acq.nfreq );

            /* fft correlation */
            buffloc = acquisition( sdr, acqpower );

            if( acqpower!=NULL ){
                free( acqpower );
                acqpower == NULL;
            }
        
        }

        if( sdr->flagacq > 0 ) {

            if( sdrini.pltacq ) { /* plot aquisition result */

                pltacq.z = acqpower;
                plot( &pltacq ); 

            }

        }else{

            sdr->flagacq = sdr->flagacq > -3 ? -10 : ++(sdr->flagacq) ;
            sleepms( - sdr->flagacq * ACQSLEEP );
        
        }

        /* tracking */
        if( sdr->flagacq > 0 ) {
            bufflocnow=sdrtracking(sdr,buffloc,cnt);
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
                    
                    /* LEX thread */
                    if (sdrini.nchL6!=0&&sdr->no==sdrini.nch+1&&loopcnt>250) 
                        setevent(hlexeve);

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
    
    if (sdrini.nchL6!=0&&sdr->no==sdrini.nch+1) 
        setevent(hlexeve);
    
    /* plot termination */
    quitpltstruct(&pltacq,&plttrk);

    /* cloase tracking log file */
    if (sdrini.log) closelog(fp);

    if (sdr->flagacq) {
        debug_print("SDR channel %s thread finished! Delay=%d [ms]\n",
            sdr->satstr,(int)(bufflocnow-buffloc)/sdr->nsamp);
    } else {
        debug_print("SDR channel %s thread finished!\n",sdr->satstr);
    }

    return THRETVAL;
}

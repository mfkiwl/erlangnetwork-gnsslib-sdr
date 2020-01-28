/*------------------------------------------------------------------------------
* sdrmain.c : SDR main functions
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
*-----------------------------------------------------------------------------*/
#include "sdr.h"

/* global variables ----------------------------------------------------------*/
#ifdef GUI
GCHandle hform;
#endif

/* thread handle and mutex */
thread_t hmainthread;
thread_t hsyncthread;
thread_t hspecthread;
thread_t hkeythread;
mlock_t hbuffmtx;
mlock_t hreadmtx;
mlock_t hfftmtx;
mlock_t hpltmtx;
mlock_t hobsmtx;
mlock_t hlexmtx;
event_t hlexeve;

/* sdr structs */
sdrini_t sdrini={0};
sdrstat_t sdrstat={0};
sdrch_t sdrch[MAXSAT]={{0}};
sdrspec_t sdrspec={0};
sdrout_t sdrout={0};

/* initsdrgui ------------------------------------------------------------------
* initialize sdr gui application  
* args   : maindlg^ form       I   main dialog class
*          sdrini_t* sdrinigui I   sdr init struct
* return : none
* note : This function is only used in GUI application 
*-----------------------------------------------------------------------------*/
#ifdef GUI
extern void initsdrgui(maindlg^ form, sdrini_t* sdrinigui)
{
    /* initialization global structs */
    memset(&sdrini,0,sizeof(sdrini));
    memset(&sdrstat,0,sizeof(sdrstat));
    memset(&sdrch,0,sizeof(sdrch));

    hform=GCHandle::Alloc(form);
    memcpy(&sdrini,sdrinigui,sizeof(sdrini_t)); /* copy setting from GUI */
}
#else
/* keyboard thread -------------------------------------------------------------
* keyboard thread for program termination  
* args   : void   *arg      I   not used
* return : none
* note : this thread is only created in CLI application
*-----------------------------------------------------------------------------*/
#ifdef WIN32
extern void keythread(void * arg) 
#else
extern void *keythread(void * arg) 
#endif
{
    do {
        switch(getchar()) {
        case 'q':
        case 'Q':
            sdrstat.stopflag=1;
            break;
        default:
            SDRPRINTF("press 'q' to exit...\n");
            break;
        }
    } while (!sdrstat.stopflag);

    return THRETVAL;
}
/* main function ---------------------------------------------------------------
* main entry point in CLI application  
* args   : none
* return : none
* note : This function is only used in CLI application 
*-----------------------------------------------------------------------------*/
int main( int argc, char **argv )
{
    if( argc < 2 ){
        char inifilename[] = "./gnss-sdrcli.ini";
        /* read ini file */
        if (readinifile( &sdrini, inifilename )<0) {
            return -1; 
        }

    }else{

        if( readinifile( &sdrini, argv[1] )<0) {
            return -1; 
        }

    }
    
    cratethread(hkeythread,keythread,NULL);

    startsdr();

    return 0;
}
#endif
/* sdr start -------------------------------------------------------------------
* start sdr function  
* args   : void   *arg      I   not used
* return : none
* note : This function is called as thread in GUI application and is called as
*        function in CLI application
*-----------------------------------------------------------------------------*/
#ifdef GUI
extern void startsdr(void *arg) /* call as thread */
#else
extern void startsdr(void) /* call as function */
#endif
{
    int i;
    SDRPRINTF("ERLANG NETWORK gnss-sdrlib start!\n");

    /* check initial value */
    if (chk_initvalue(&sdrini)<0) {
        SDRPRINTF("error: chk_initvalue\n");
        quitsdr(&sdrini,1);
        return;
    }

    /* receiver initialization */
    if (rcvinit(&sdrini)<0) {
        SDRPRINTF("error: rcvinit\n");
        quitsdr(&sdrini,1);
        return;
    }
    /* initialize sdr channel struct */
    for (i=0;i<sdrini.nch;i++) {
        if (initsdrch(i+1,sdrini.sys[i],sdrini.prn[i],sdrini.ctype[i],
            sdrini.dtype[sdrini.ftype[i]-1],sdrini.ftype[i],
            sdrini.f_cf[sdrini.ftype[i]-1],sdrini.f_sf[sdrini.ftype[i]-1],
            sdrini.f_if[sdrini.ftype[i]-1],&sdrch[i])<0) {
            
            SDRPRINTF("error: initsdrch\n");
            quitsdr(&sdrini,2);
            return;
        }
    }

    /* mutexes and events */
    openhandles();

    /* create threads */
    cratethread(hsyncthread,syncthread,NULL); /* synchronization thread */

    /* sdr channel thread */
    for (i=0;i<sdrini.nch;i++) {
        /* GPS/QZS/GLO/GAL/CMP L1 */
        if (sdrch[i].ctype==CTYPE_L1CA  || sdrch[i].ctype==CTYPE_G1  ||
            sdrch[i].ctype==CTYPE_E1B   || sdrch[i].ctype==CTYPE_B1I ||
            sdrch[i].ctype==CTYPE_L1SBAS|| sdrch[i].ctype==CTYPE_L1SAIF)
            cratethread(sdrch[i].hsdr,sdrthread,&sdrch[i]);
        /* QZSS LEX */
        if (sdrch[i].sys==SYS_QZS&&sdrch[i].ctype==CTYPE_LEXS) {
            sdrini.nchL6++;
            cratethread(sdrch[i].hsdr,lexthread,&sdrch[i]);

            /* create QZSS L1CA channel */
            initsdrch(sdrini.nch+1,SYS_QZS,193,CTYPE_L1CA,DTYPEI,FTYPE1,
               sdrini.f_cf[0],sdrini.f_sf[0],sdrini.f_if[0],&sdrch[sdrini.nch]);
            cratethread(sdrch[sdrini.nch].hsdr,sdrthread,&sdrch[sdrini.nch]);
        }
        //SDRPRINTF("initialize %s receiption!\n", sdrch[i].satstr );
    }
#ifndef GUI
    /* sdr spectrum analyzer */
    if (sdrini.pltspec) {
        sdrspec.dtype=sdrini.dtype[sdrini.pltspec-1];
        sdrspec.ftype=sdrini.ftype[sdrini.pltspec-1];
        sdrspec.nsamp=(int)(sdrini.f_sf[sdrini.pltspec-1]/1000); /* 1ms */
        sdrspec.f_sf=sdrini.f_sf[sdrini.pltspec-1];
        cratethread(hspecthread,specthread,&sdrspec);
    }
#endif

    /* start grabber */
    SDRPRINTF("start receiver hardware!\n");
    if (rcvgrabstart(&sdrini)<0) {
        quitsdr(&sdrini,4);
        return;
    }

    /* data grabber loop */
    while (!sdrstat.stopflag) {
        if (rcvgrabdata(&sdrini)<0) {
            sdrstat.stopflag=ON;
            break;
        }
    }
    /* wait thereds */
    waitthread(hsyncthread);
    for (i=0;i<sdrini.nch;i++) 
        waitthread(sdrch[i].hsdr);

    /* sdr termination */
    quitsdr(&sdrini,0);

    SDRPRINTF("ERLANG NETWORK gnss-sdrlib is finished!\n");
}
/* sdr termination -------------------------------------------------------------
* sdr termination process  
* args   : sdrini_t *ini    I   sdr init struct
* args   : int    stop      I   stop position in function 0: run all
* return : none
*-----------------------------------------------------------------------------*/
extern void quitsdr(sdrini_t *ini, int stop)
{
    int i;
#ifdef GUI
    maindlg^form=static_cast<maindlg^>(hform.Target);
    form->guistop();
#endif
    if (stop==1) return;

    /* sdr termination */
    rcvquit(ini);
    if (stop==2) return;

    /* free memory */
    for (i=0;i<ini->nch;i++) freesdrch(&sdrch[i]);
    if (stop==3) return;

    /* mutexes and events */
    closehandles();
    if (stop==4) return;
}
/**
 * \file sdrmain.c
 * \brief GNSS engines main functions or entrance point
 * 
 * \copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * */
#include "measurement_engine.h"

/* global variables ----------------------------------------------------------*/

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

/* sdr structs */
sdrini_t sdrini={0};
sdrstat_t sdrstat={0};
sdrch_t sdrch[MAXSAT]={{0}};
sdrspec_t sdrspec={0};
sdrout_t sdrout={0};

/* keyboard thread -------------------------------------------------------------
* keyboard thread for program termination  
* args   : void   *arg      I   not used
* return : none
* note : this thread is only created in CLI application
*-----------------------------------------------------------------------------*/
extern void *keythread(void * arg) 
{
    do {
        switch(getchar()) {
        case 'q':
        case 'Q':
            sdrstat.stopflag=1;
            break;
        default:
            debug_print("press 'q' to exit...\n");
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
            return -3; 
        }

    }
    
    cratethread( hkeythread, keythread, NULL );

    /* FFT initialization */
    fftw_init_threads();

    start_erlangnetwork_gnssmeasurementengine();

    return 0;
}

/* sdr start -------------------------------------------------------------------
* start sdr function  
* args   : void   *arg      I   not used
* return : none
* note : This function is called as thread in GUI application and is called as
*        function in CLI application
*-----------------------------------------------------------------------------*/
extern void start_erlangnetwork_gnssmeasurementengine(void) /* call as function */
{
    int i;
    debug_print("Erlang Network GNSSLib start!\n");

    /* check initial value */
    if (chk_initvalue(&sdrini)<0) {
        debug_print("error: chk_initvalue\n");
        quit_erlangnetwork_gnssmeasurementengine( &sdrini, 1 );
        return;
    }

    /* receiver initialization */
    if (rcvinit(&sdrini)<0) {
        debug_print("error: rcvinit\n");
        quit_erlangnetwork_gnssmeasurementengine( &sdrini, 1 );
        return;
    }

    /* start grabber */
    debug_print("start receiver hardware!\n");
    if (rcvgrabstart(&sdrini)<0) {
        quit_erlangnetwork_gnssmeasurementengine( &sdrini, 4 );
        return;
    }

    /* initialize sdr channel struct */
    for( i=0;i<sdrini.nch;i++) {

        if( initialize_measurement_channel( i+1,
                        sdrini.sys[i],
                        sdrini.prn[i],
                        sdrini.ctype[i],
                        sdrini.dtype[sdrini.ftype[i]-1],
                        sdrini.ftype[i],
                        sdrini.f_cf[sdrini.ftype[i]-1],
                        sdrini.f_sf[sdrini.ftype[i]-1],
                        sdrini.f_if[sdrini.ftype[i]-1],
                        &sdrch[i] ) < 0 ) {           
            debug_print( "error: initalize_measurement_channel, chno=%d, sys=%d, prn=%d f_sf=%f, %f\n", i+1, sdrini.sys[i], sdrini.prn[i], sdrini.f_sf[0], sdrini.f_sf[1] );
            quit_erlangnetwork_gnssmeasurementengine( &sdrini, 2 );
            return;
        }
    }

    /* mutexes and events */
    openhandles();

    /* create threads */
    cratethread(hsyncthread,syncthread,NULL); /* synchronization thread */

    /* sdr channel thread */
    for (i=0;i<sdrini.nch;i++) {

        if(1) debug_print("%s sys=%d prn=%02d ctype=%d dtype=%d ftype=%d f_cf=%.0f f_sf=%.0f f_if=%.0f clen=%d crate=%.0f nsamp=%d coherencelen_code[0]=%d\n", 
            sdrch[i].satstr,
            sdrch[i].sys,
            sdrch[i].prn,
            sdrch[i].ctype,
            sdrch[i].dtype,
            sdrch[i].ftype,
            sdrch[i].f_cf,
            sdrch[i].f_sf,
            sdrch[i].f_if,
            sdrch[i].clen,
            sdrch[i].crate,
            sdrch[i].nsamp,
            sdrch[i].acq.coherencelen_code[0] );

        if (sdrch[i].ctype==CTYPE_L1CA  || sdrch[i].ctype==CTYPE_G1  ||
            sdrch[i].ctype==CTYPE_L1SBAS ){   /* GPS/GLO/GAL/CMP L1 */

            cratethread( sdrch[i].hsdr, statemachinethread, &sdrch[i] ) ;

        }
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
    quit_erlangnetwork_gnssmeasurementengine( &sdrini, 0 );

    debug_print("Erlang Network GNSSLib is finished!\n");
}
/* sdr termination -------------------------------------------------------------
* sdr termination process  
* args   : sdrini_t *ini    I   sdr init struct
* args   : int    stop      I   stop position in function 0: run all
* return : none
*-----------------------------------------------------------------------------*/
extern void quit_erlangnetwork_gnssmeasurementengine(sdrini_t *ini, int stop)
{
    int i;

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
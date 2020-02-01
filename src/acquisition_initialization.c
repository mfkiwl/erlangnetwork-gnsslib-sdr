/**
 * @file acquisition_initialization.c
 * @date January 29, 2020
 * @author Shu Wang <shuwang1@outlook.com>
 * @brief initialize the data structure for GNSS Signal acquisition.
 * 
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * \copyright Private Uses are permitted but commercial uses shall be licensed. See LICENSE file.
 **/
#include "measurement_engine.h"

#define  _numcoherentcodes_   2
/*!
 * initialize acquisition data struct
 * set value to acquisition struct
 * 
 * @param[in/out]   acquisition_t *acq    I/0 acquisition struct
 * @param[in]   int ctype        I   code type (CTYPE_L1CA...)
 * @param[in]   int prn          I   PRN
 * @param[in]   int sys          I   system type (SYS_GPS...)
 * @return int
 **/
int initialize_acqusition_struct( 
                acquisition_t *acq, 
                int ctype,
                uint16_t *coherencelen_code,
                uint16_t codelen_sample,
                double intfreq_Hz,
                double offset_Hz )
{

#define FAST_REACQUISITION_CHANNEL_NUMBER   9

    int ii, jj;
    acq->hband  = ACQHBAND;
    acq->step   = ACQSTEP;
    acq->numfreq[0] = 2*(ACQHBAND/ACQSTEP)+1;

    switch( ctype ){
        case CTYPE_L1CA:
            acq->noncoherentintegration[0] = ACQINTG_L1CA;
            acq->noncoherentintegration[1] = ACQINTG_L1CA;
            acq->noncoherentintegration[2] = ACQINTG_L1CA;
            break;

        case CTYPE_G1:
            acq->noncoherentintegration[0]=ACQINTG_G1;
            acq->noncoherentintegration[1]=ACQINTG_G1;
            acq->noncoherentintegration[2]=ACQINTG_G1;
            break;

        case CTYPE_L1SBAS:
            acq->noncoherentintegration[0]=ACQINTG_SBAS;
            acq->noncoherentintegration[1]=ACQINTG_SBAS;
            acq->noncoherentintegration[2]=ACQINTG_SBAS;
            break;

        default:
            acq->noncoherentintegration[0]=ACQINTG_L1CA;
            acq->noncoherentintegration[1]=ACQINTG_L1CA;
            acq->noncoherentintegration[2]=ACQINTG_L1CA;
    }

    switch( coherencelen_code[0] ){

        case 2:
            acq->hband  = 5000;
            acq->step   = 250;
            acq->numfreq[0]     = 41;
            acq->numfreq[1]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            acq->numfreq[2]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            break;
        
        case 8:
            acq->hband  = 3968.75;
            acq->step   = 62.5;
            acq->numfreq[0]     = 128;
            acq->numfreq[1]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            acq->numfreq[2]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            break;
        
        case 16:
            acq->hband  = 3968.75;
            acq->step   = 31.25;
            acq->numfreq[0]     = 256;
            acq->numfreq[1]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            acq->numfreq[2]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            break;

        case 15:
            acq->hband  = 4233.291;
            acq->step   = 200/3.0;
            acq->numfreq[0]     = 256;
            acq->numfreq[1]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            acq->numfreq[2]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            break;

        default:
            acq->hband  = 5000;
            acq->step   = 500/coherencelen_code[0];

            acq->numfreq[0]     = 2*( (uint16_t) acq->hband/acq->step ) + 1;
            acq->numfreq[1]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            acq->numfreq[2]     = FAST_REACQUISITION_CHANNEL_NUMBER;
            /** break; **/
    }

     /* memory allocation */
    if( !(acq->freq = (double*) malloc(sizeof(double) * acq->numfreq[0] )) ) {
        debug_print("error: acq->freq memory alocation\n"); return -1;
    }

    /* doppler search frequency */
    for( ii=0; ii<acq->numfreq[0]; ii++ )
        acq->freq[ii] = intfreq_Hz +((ii-(acq->numfreq[0]-1)/2)*acq->step) + offset_Hz;

    if( !( acq->freq_4thHz[0] = (int16_t*) malloc( sizeof(int16_t) * acq->numfreq[0] ) ) ) {
        debug_print("memory allocation error: acq->freq0_4thHz. hband=%d, step=%d, numfreq=%d\n", 
                acq->hband, acq->step, acq->numfreq[0] ); 
        return -1;
    }
    for( ii = 0; ii < acq->numfreq[0]; ii++ ){
        acq->freq_4thHz[0][ii] = ( intfreq_Hz + offset_Hz ) * 4 + ( ii - acq->numfreq[0]/2.0 + 0.5 ) * acq->step * 4;                            
    }


//if( 1 ){ int16_t * y=(int16_t*) acq->freq_4thHz[0]; printf( "\nnumfreq[0]=%d freq_4thHz[0]=", acq->numfreq[0]); for(int zz=0; zz<acq->numfreq[0]; zz+=2) printf(" %d ", y[zz] ); printf("\n"); }

    if( !( acq->freq_4thHz[1] = (int16_t*) malloc( sizeof(int16_t) * acq->numfreq[1] ) ) ) {
        debug_print("memory allocation error: acq->freq1_4thHz. halfband_Hz=%d, step_128thHz=%d, numfreq=%d\n", 
                -acq->freq_4thHz[1][0], 100000 / 30, acq->numfreq[1] );
        return -3;
    }
    for( ii = 0; ii < acq->numfreq[1]; ii++ ){
        acq->freq_4thHz[1][ii] = ( ii - acq->numfreq[1]/2.0 + 0.5 ) * acq->step * 4 ;
    }


    switch( coherencelen_code[0] ){
        
        case 15:
            acq->fftlen_bit[0] = ceil( log( 15 * codelen_sample ) / log(2) );
            acq->fftlen[0]     = 1 << acq->fftlen_bit[0]; /*! fftlen = pow( 2, acq->fftlen_bit ) ; **/

            acq->fftlen_bit[1] = ceil( log( coherencelen_code[1] * codelen_sample ) / log(2) );
            acq->fftlen[1]     = 1 << acq->fftlen_bit[1]; /*! fftlen = pow( 2, acq->fftlen_bit ) ; **/

            break;

        default:
            acq->fftlen_bit[0] = ceil( log( coherencelen_code[0] * codelen_sample ) / log(2) );
            acq->fftlen[0]     = 1 << acq->fftlen_bit[0]; /*! fftlen = pow( 2, acq->fftlen_bit ) ; **/

            acq->fftlen_bit[1] = ceil( log( coherencelen_code[1] * codelen_sample ) / log(2) );
            acq->fftlen[1]     = 1 << acq->fftlen_bit[1]; /*! fftlen = pow( 2, acq->fftlen_bit ) ; **/
    
    }

//debug_print("coherencelen_code=[%d %d] acq->fftlen=[%d %d]\n", acq->coherencelen_code[0], acq->coherencelen_code[1], acq->fftlen[0], acq->fftlen[1] );
 
    return 0;

#undef FAST_REACQUISITION_CHANNEL_NUMBER    
}

/**
 * @brief initialize measurement engine channel struct
 * set value to sdr channel struct
 * @param[in]   int    chno      I   channel number (1,2,...)
 * @param[in]   int    sys       I   system type (SYS_***)
 * @param[in]   int    prn       I   PRN number
 * @param[in]   int    ctype     I   code type (CTYPE_***)
 * @param[in]   int    dtype     I   data type (DTYPEI or DTYPEIQ)
 * @param[in]   int    ftype     I   front end type (FTYPE1 or FTYPE2)
 * @param[in]   double f_cf      I   center (carrier) frequency (Hz)
 * @param[in]   double f_sf      I   sampling frequency (Hz)
 * @param[in]   double f_if      I   intermidiate frequency (Hz)
 * @param[in,out]   sdrch_t *sdr     I/0 sdr channel struct
 * @return : int                  0:okay -1:error
 **/
extern int initialize_measurement_channel(int chno,
                        int sys, 
                        int prn, 
                        int ctype, 
                        int dtype,
                        int ftype, 
                        double f_cf, 
                        double f_sf, 
                        double f_if,
                        sdrch_t *sdr )
{
    int i, ii, retvalue;

    sdr->no=chno;
    sdr->sys=sys;
    sdr->prn=prn;
    sdr->sat=satno(sys,prn);
    sdr->ctype=ctype;
    sdr->dtype=dtype;
    sdr->ftype=ftype;
    sdr->f_sf=f_sf;
    sdr->f_if=f_if;
    sdr->ti=1/f_sf;
    
    /* code generation */
    if (!(sdr->code = gencode( prn,ctype,&sdr->clen,&sdr->crate))) {
        debug_print("error: gencode\n");
        return -1;
    }
    sdr->ci = sdr->ti*sdr->crate;
    sdr->ctime = sdr->clen/sdr->crate;

    sdr->nsamp = (int)( f_sf * sdr->ctime );  
    const unsigned int nsamp = sdr->nsamp ;

    sdr->nsampchip=(int)(nsamp/sdr->clen);
    satno2id(sdr->sat,sdr->satstr);


    /* set carrier frequency */
    if (ctype==CTYPE_G1) {
        sprintf(sdr->satstr,"R%d",prn); /* frequency number instead of PRN */
        sdr->f_cf=FREQ1_GLO+DFRQ1_GLO*prn; /* FDMA */
        sdr->foffset=DFRQ1_GLO*prn; /* FDMA */
    } else if (sdrini.fend==FEND_FRTLSDR) {
        sdr->foffset=f_cf*sdrini.rtlsdrppmerr*1e-6;
    } else {
        sdr->f_cf=f_cf; /* carrier frequency */
        sdr->foffset=0.0; /* frequency offset */
    }

    uint32_t coherencelen_sample[3];

    switch( _numcoherentcodes_ ){
        case 15:
            sdr->acq.coherencelen_code[0] = 15;
            sdr->acq.coherencelen_code[1] = 15;
            coherencelen_sample[0] = nsamp * sdr->acq.coherencelen_code[0];
            coherencelen_sample[1] = nsamp * sdr->acq.coherencelen_code[1];
            break;
        default:
            sdr->acq.coherencelen_code[0] = _numcoherentcodes_;
            sdr->acq.coherencelen_code[1] = 15;
            coherencelen_sample[0] = nsamp * sdr->acq.coherencelen_code[0];
            coherencelen_sample[1] = nsamp * sdr->acq.coherencelen_code[1];
    }

    if( initialize_acqusition_struct( &sdr->acq, ctype, sdr->acq.coherencelen_code,
                nsamp, sdr->f_if, sdr->foffset ) < 0 ){
        debug_print( "error:initialize_acqusition_struct\n");
    }

    assert( sdr->acq.fftlen[0] >= coherencelen_sample[0] ) ;

//if( 1 ){ int16_t * y = (int16_t *) sdr->acq.freq_4thHz[0]; printf( "\nacq.numfreq[0]=%d acq.freq_4thHz[0]=[",  sdr->acq.numfreq[0]); for(int zz=0; zz<8; zz++) printf("%d ", y[zz] ); printf("]\n"); }
//if( 1 ){ double * y = (double *) sdr->acq.freq; printf( "\nacq.numfreq[0]=%d acq.freq=[",  sdr->acq.numfreq[0]); for(int zz=0; zz<8; zz++) printf("%f ", y[zz] ); printf("]\n"); }
    
    /* tracking struct */
    if( inittrkstruct(sdr->sat,ctype,sdr->ctime,&sdr->trk) < 0 ){
        debug_print("error: inittrkstruct failed \n"); 
        return -3 ;
    }
    /* navigation struct */
    if (initnavstruct(sys,ctype,prn,&sdr->nav)<0) {
        debug_print("error: initnavstruct failed \n"); 
        return -5 ;
    }

/**
debug_print("fftlen_bit=[%d %d] fftlen=[%d %d] numcoherentsamples=%d numcoherentcodes=%d nfreq=[%d %d] sdr->ci=%f\n", 
        sdr->acq.fftlen_bit[0], sdr->acq.fftlen_bit[1], sdr->acq.fftlen[0], sdr->acq.fftlen[1], 
        coherencelen_sample[0], sdr->acq.coherencelen_code[0], sdr->acq.numfreq[0], sdr->acq.numfreq[1], sdr->ci ); 
**/

    unsigned int fftlength = sdr->acq.fftlen[0];
    double scalefactor = 1.0 / ( 1 )  ;

    int zeropadding_sample ;
    switch( sdr->acq.coherencelen_code[0] ){
        case 15:
            zeropadding_sample = nsamp ;
            break;
        default:
            zeropadding_sample = nsamp ;
    }

    float *rcode, t, *p;
    /* memory allocation */
    if ( !( rcode = ( double *) malloc( sizeof( float ) * ( fftlength + zeropadding_sample ) + 32 ) ) ) {
        debug_print("error: resampled PN code memory alocation\n"); 
        return -7;
    }

    /*! resampling for a PN code with a length inequal to 1023 and with repetitiion */
    for( ii=0; ii < zeropadding_sample ; ii++ ) rcode[ii] = 0.0 ;
    //for( t=0; ii < fftlength; ii++, t += sdr->ci ) rcode[ii] = sdr->code[ ( (int) t) % sdr->clen ] * scalefactor ;
    for( t=0; ii < fftlength; ii++, t += sdr->ci ) rcode[ii] = sdr->code[ ( (int) t) % sdr->clen ] * scalefactor ;
    for(    ; ii < (fftlength + zeropadding_sample ); ii++ ) rcode[ii] = 0.0 ;


if( 1 ){ float *y = (float *) rcode; printf("\n%d rcode=", sdr->no); for(int x=zeropadding_sample; x<zeropadding_sample+80; x++) printf( " %.0f", y[x]  ); printf("\n"); }

    if( !(sdr->acq.xcode[0]) ){
        if( !( sdr->acq.xcode[0] = fftwf_malloc( sizeof(fftwf_complex) * fftlength + 32 ) ) ){
            debug_print("error: sdr->acq.xcode[0] memory alocation\n"); 
            return -13;
        }
    }

    if( !(sdr->acq.xcode[1]) ){
        if( !( sdr->acq.xcode[1] = fftwf_malloc( sizeof(fftwf_complex) * fftlength + 32 ) ) ){
            debug_print("error: sdr->acq.xcode[1] memory alocation\n"); 
            return -15;
        }
    }

//if( sdr->no > 0 ){ int16_t *y = (int16_t*) sdr->code; printf( "\nsdr->code = [" ); for(int zz=0; zz<20; zz++) printf(" %d ", y[zz] ); printf("]\n"); }
//if( sdr->no > 0 ){ double *y = (double*) rcode; printf( "zeropadding_sample=%d rcode = [", zeropadding_sample ); for(int zz=0; zz<20; zz++) printf(" %.2e ", y[zz] ); printf("]\n"); }
   
    p = (float*) sdr->acq.xcode[0];
    for( ii = zeropadding_sample; ii < fftlength + zeropadding_sample; ii++, p+=2 ) {
        p[0] = rcode[ii] ;
        p[1] = 0.0;
    }
    p = (float*) sdr->acq.xcode[1];
    for( ii=0; ii<fftlength; ii++, p+=2 ) {
        p[0] = rcode[ii] ;
        p[1] = 0.0;
    }


//if( sdr->no > 0 ){ double * y = (double*) sdr->acq.xcode[0]; printf( "%d 1 sdr->acq.xcode[0] = [", sdr->no ); for(int zz=0; zz<10; zz++) printf(" %.1e ", y[zz] ); printf("]\n"); }
if( sdr->no > 0 ){ float * y = (float*) sdr->acq.xcode[0]; printf( "%d 1 sdr->acq.xcode[0] = [", sdr->no ); for(int zz=0; zz<20; zz++) printf(" %.2e ", y[zz] ); printf("]\n"); }

    fftwf_plan plan_f;
    fftw_plan plan_d;
#ifdef FFTMTX
    mlock(hfftmtx);
#endif

    //fftwf_plan_with_nthreads(NFFTTHREAD);
    plan_f = fftwf_plan_dft_1d( fftlength, sdr->acq.xcode[0], sdr->acq.xcode[0], FFTW_FORWARD, FFTW_ESTIMATE );   /**Once the plan has been created, you can use it as many times as you like for transforms on arrays of the same size.**/                                                                                                                  /**When you are done with the plan, you deallocate it by calling fftw_destroy_plan(plan).**/
    fftwf_execute_dft( plan_f, sdr->acq.xcode[0], sdr->acq.xcode[0] ); /* fft */
    fftwf_destroy_plan( plan_f ); 
    
    plan_f = fftwf_plan_dft_1d( fftlength, sdr->acq.xcode[1], sdr->acq.xcode[1], FFTW_FORWARD, FFTW_ESTIMATE );
    fftwf_execute_dft( plan_f, sdr->acq.xcode[1], sdr->acq.xcode[1] ); /* fft */
    fftwf_destroy_plan( plan_f );

#ifdef FFTMTX
    unmlock(hfftmtx);
#endif


if( sdr->no > 0 ){ float * y = (float*) sdr->acq.xcode[0]; printf( "%d 2 sdr->acq.xcode[0] = [", sdr->no ); for(int zz=0; zz<20; zz++) printf(" %.2e ", y[zz] ); printf("]\n"); }
if( sdr->no > 0 ){ float * y = (float*) sdr->acq.xcode[1]; printf( "%d 2 sdr->acq.xcode[1] = [", sdr->no ); for(int zz=0; zz<20; zz++) printf(" %.2e ", y[zz] ); printf("]\n"); }

    if( !( sdr->xcode ) ){
        if( !( sdr->xcode = fftw_malloc( sizeof(fftw_complex)*fftlength + 32 ) ) ){
            debug_print("error: sdr->xcode memory alocation\n"); 
            return -17;
        }
    }

    if( !( sdr->codex ) ){
        if( !( sdr->codex = fftw_malloc( sizeof(fftw_complex)*fftlength + 32 ) ) ){
            debug_print("error: sdr->codex memory alocation\n"); 
            return -19;
        }
    }

    double *pp = (double*) sdr->codex;
    for( ii=zeropadding_sample; ii<fftlength+zeropadding_sample; ii++, pp+=2 ) {
        pp[0] = (double) rcode[ii];
        pp[1] = (double) 0.0;
    }

    pp = (double*) sdr->xcode;
    for( ii=0; ii<fftlength; ii++, pp+=2 ) {
        pp[0] = (double) rcode[ii];
        pp[1] = (double) 0.0;
    }

//if( sdr->no > 0 ){ double * y = (double*) sdr->codex; printf( "\n%s fftlength=%d codex = [", sdr->satstr, fftlength); for(int zz=0; zz<20; zz++) printf(" %.3e ", y[zz] ); printf("]\n"); }

#ifdef FFTMTX
    mlock(hfftmtx);
#endif

    fftw_plan_with_nthreads(NFFTTHREAD);

    plan_d = fftw_plan_dft_1d( fftlength, sdr->xcode, sdr->xcode, FFTW_FORWARD, FFTW_ESTIMATE);  /**Once the plan has been created, you can use it as many times as you like for transforms on arrays of the same size.**/
                                                                                                /**When you are done with the plan, you deallocate it by calling fftw_destroy_plan(plan).**/
    fftw_execute_dft( plan_d, sdr->xcode, sdr->xcode ); /* fft */

    fftw_destroy_plan( plan_d ); 
    plan_d = fftw_plan_dft_1d( fftlength, sdr->codex, sdr->codex, FFTW_FORWARD, FFTW_ESTIMATE);

    fftw_execute_dft( plan_d, sdr->codex, sdr->codex ); /* fft */
    fftw_destroy_plan( plan_d );

#ifdef FFTMTX
    unmlock(hfftmtx);
#endif

//if( sdr->no > 0 ){ double * y = (double*) sdr->acq.xcode[0]; printf( "%s xcode[0] = [", sdr->satstr ); for(int zz=0; zz<20; zz++) printf(" %.1e ", y[zz] ); printf("]\n"); }
//if( sdr->no > 0 ){ double * y = (double*) sdr->codex; printf( "\n%s fftlength=%d codex = [", sdr->satstr, fftlength); for(int zz=0; zz<20; zz++) printf(" %.3e ", y[zz] ); printf("]\n"); }

    double d[] = {0.0, 0.0};
    p = (float*) sdr->acq.xcode[1] ;
    float *q = (float*) sdr->acq.xcode[0] ;
    pp = (double*) sdr->codex;
    double *qq = (double*) sdr->xcode;
    for( int ii=0; ii<fftlength; ii++, p+=2, q+=2, pp+=2, qq+=2 ) {
        d[0] += (p[0] - pp[0])*(p[0] - pp[0]) + (p[1] - pp[1])*(p[1] - pp[1]);
        d[1] += (q[0] - qq[0])*(q[0] - qq[0]) + (q[1] - qq[1])*(q[1] - qq[1]);
    }
  
    
//debug_print("no=%2d d[0:1] = [%.3f %.3f] p[0:1] = [%.3f %f] pp[0:1]=[%.3f %.3f] q[0:1]=[%.3f %.3f] qq[0:1]=[%.3f %.3f]\n", sdr->no, d[0], d[1], p[0], p[1], pp[0], pp[1], q[0], q[1], qq[0], qq[1] );

    if(rcode) free( rcode );
    
    uint8_t  fftlen_bit = 15;
    fftlength = 1 << fftlen_bit;
    scalefactor = 1.0 / ( 1 );

    const unsigned int numcoherentcodes = 15;
    zeropadding_sample = fftlength - numcoherentcodes * nsamp ;
    zeropadding_sample =  ( zeropadding_sample > nsamp ) ? zeropadding_sample : nsamp ;
    /* memory allocation */
    if( !( rcode = (float *) malloc( sizeof(float) * ( fftlength + zeropadding_sample ) + 32 ) )  ) {
        debug_print("error:  rcode, sdr->acq.xcode[2],  sdr->acq.xcode[3] memory alocation\n"); 
        return -21;
    }

    /* resampling */
    for( i=0; i<zeropadding_sample ; i++ ) rcode[i] = 0 ;
    for( t=0; i<fftlength; i++, t += sdr->ci ) rcode[i] = sdr->code[ ((int) t) % sdr->clen ] * scalefactor ;
    for(    ; i<(fftlength + zeropadding_sample ); i++ ) rcode[i] = 0 ;
    

    if( !( sdr->acq.xcode[2] = fftwf_malloc( sizeof(fftwf_complex) * fftlength + 32 ) ) ){
        debug_print("error: sdr->acq.xcode[0] memory alocation\n"); 
        return -13;
    }

    if( !( sdr->acq.xcode[3] = fftwf_malloc( sizeof(fftwf_complex) * fftlength + 32 ) ) ){
        debug_print("error: sdr->acq.xcode[1] memory alocation\n"); 
        return -15;
    }

    p = (float*) sdr->acq.xcode[2] ;
    for( ii=zeropadding_sample; ii<fftlength+zeropadding_sample; ii++, p+=2 ) {
        p[0] = rcode[ii];
        p[1] = 0.0f;
    }
    p = (float*) sdr->acq.xcode[3];
    for( ii=0; ii<fftlength; ii++, p+=2 ) {
        p[0] = rcode[ii];
        p[1] = 0.0;
    }


#ifdef FFTMTX
    mlock(hfftmtx);
#endif

    plan_f = fftwf_plan_dft_1d( fftlength, sdr->acq.xcode[2], sdr->acq.xcode[2], FFTW_FORWARD, FFTW_ESTIMATE);   /**Once the plan has been created, you can use it as many times as you like for transforms on arrays of the same size.**/
                                                                                                                /**When you are done with the plan, you deallocate it by calling fftw_destroy_plan(plan).**/

    fftwf_execute_dft( plan_f, sdr->acq.xcode[2], sdr->acq.xcode[2] ); /* fft */

    fftwf_destroy_plan( plan_f ); plan_f = fftwf_plan_dft_1d( fftlength, sdr->acq.xcode[3], sdr->acq.xcode[3], FFTW_FORWARD, FFTW_ESTIMATE);

    fftwf_execute_dft( plan_f, sdr->acq.xcode[3], sdr->acq.xcode[3] ); /* fft */
    fftwf_destroy_plan( plan_f );   

#ifdef FFTMTX
    unmlock(hfftmtx);
#endif

    if( rcode ) free( rcode );

for( ii=0; ii < 4; ii++) ( (float *) sdr->acq.xcode[ii] )[0] = ( (float *) sdr->acq.xcode[ii] )[1] = 0 ;
    ( (double *) sdr->xcode )[0] = ( (double *) sdr->xcode )[1] = 0 ;
    ( (double *) sdr->codex )[0] = ( (double *) sdr->codex )[1] = 0 ;

    return 0;
}
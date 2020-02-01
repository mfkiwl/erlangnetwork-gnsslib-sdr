/**
 * @file acquisition.c
 * @date January 29, 2020
 * @author Shu Wang <shuwang1@outlook.com>
 * @brief GNSS Signal acquisition functions.
 * 
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * \copyright Private Uses are permitted but commercial uses shall be licensed. See LICENSE file.
 * **/
#include "measurement_engine.h"
#define _DEBUG      0
/* FFT convolution -------------------------------------------------------------
* conv=sqrt(abs(ifft(fft(cpxa).*conj(cpxb))).^2) 
* args   : fftw_plan plan  I   fftw plan (NULL: create new plan)
*          fftw_plan iplan I   ifftw plan (NULL: create new plan)
*          cpx_t  *cpxa     I   input complex data array
*          cpx_t  *cpxb     I   input complex data array
*          int    m         I   number of input data
*          int    n         I   number of output data
*          int    flagsum   I   cumulative sum flag (conv+=conv)
*          double *conv     O   output convolution data
* return : none
*-----------------------------------------------------------------------------*/
void _cpxconv( fftw_plan iplan, fftw_complex *cpxa, fftwf_complex *cpxb,
                    int m, int n, int flagsum, double *conv)
{
    double *p, *q;
    double real, m2 = (double) m*m;
    int i;

if(_DEBUG){ float *y = (float *) cpxa; printf( "codex=["); for( int x=0; x< 20; x++ ){ printf( " %.1e ", y[x] ); }; printf("\n"); }

    for( i = 0, p = (double *) cpxa, q = (float *)cpxb; i<m; i++,p+=2,q+=2) {
        real = -p[0]*q[0] - p[1]*q[1];
        p[1] =  p[0]*q[1] - p[1]*q[0];
        p[0] =  real;
    }

    cpxifft( iplan, cpxa, m ); /* ifft */

    if (flagsum) { /* cumulative sum */
        for( i=0, p=(double *)cpxa; i<n; i++,p+=2 ) conv[i] += ( p[0]*p[0] + p[1]*p[1] ) / m2;
    } else {
        for( i=0, p=(double *)cpxa; i<n; i++,p+=2 ) conv[i] = ( p[0]*p[0] + p[1]*p[1] ) /  m2;
    }
}

/* parallel correlator ---------------------------------------------------------
* fft based parallel correlator
* args   : char   *data     I   sampling data vector (n x 1 or 2n x 1)
*          int    dtype     I   sampling data type (1:real,2:complex)
*          double ti        I   sampling interval (s)
*          int    n         I   number of samples
*          double *freq     I   doppler search frequencies (Hz)
*          int    nfreq     I   number of frequencies
*          double crate     I   code chip rate (chip/s)
*          int    m         I   number of resampling data
*          cpx_t  codex     I   frequency domain code
*          double *P        O   normalized correlation power vector
* return : none
* notes  : P=abs(ifft(conj(fft(code)).*fft(data.*e^(2*pi*freq*t*i)))).^2
*-----------------------------------------------------------------------------*/
void _pcorrelator( const char *data, int dtype, double ti, int n, double *freq, 
                int nfreq, double crate, int m, fftwf_complex * codex, double *P )
{
#define CSCALE        (1.0/32.0)       /* carrier lookup table scale (LSB) */

    int ii, jj;
    fftw_complex *datax;
    short *dataI, *dataQ;
    char *dataR;

    if (!(dataR=(char  *) malloc( sizeof(*dataR) * m * dtype + 32 ))||
        !(dataI=(short *) malloc( sizeof(*dataI) * m + 32 )) ||
        !(dataQ=(short *) malloc( sizeof(*dataQ) * m + 32 )) ||
        !(datax = fftw_malloc( m * sizeof( fftw_complex ) + 32 )) ) {
            debug_print("error: pcorrelator memory allocation\n");
            return;
    }

    fftw_plan plan = NULL;

#ifdef FFTMTX
    mlock( hfftmtx );
#endif
    fftw_plan_with_nthreads( NFFTTHREAD ); /* fft execute in multi threads */
    plan = fftw_plan_dft_1d( m, datax, datax, FFTW_FORWARD, FFTW_ESTIMATE);
#ifdef FFTMTX
    unmlock( hfftmtx );
#endif

    /* zero padding */
    memset( dataR, 0, m*dtype ); /* zero paddinng */
    memcpy( dataR, data, 2*n*dtype ); /* for zero padding FFT */

    double scale1 = CSCALE/m, scale2 = 1.0/m/m, real;
    double *p ;
    float *q ;
    for( ii=0; ii<nfreq; ii++ ) {

        /* mix local carrier */
        mixcarr( dataR, dtype, ti, m, freq[ii], 0.0, dataI, dataQ );

        /* to complex */
        for( jj=0, p = (double*) datax; jj<m; jj++, p+=2 ) {

            p[0] = dataI[jj] * scale1 ;
            p[1] = dataQ[jj] * scale1 ;

        }

if(_DEBUG){ double *y = (double *) datax; printf( "m=%d datax_t=[", m); for( int x=0; x< 40; x++ ){ printf( " %.1e ", y[x] ); }; printf("]\n"); }

#ifdef FFTMTX
        mlock(hfftmtx);
#endif
        fftw_execute( plan ); /* fft */
#ifdef FFTMTX
        unmlock(hfftmtx);
#endif

if(_DEBUG){ double *y = (double *) datax; printf( "m=%d datax_f=[", m); for( int x=0; x< 40; x++ ){ printf( " %.1e ", y[x] ); }; printf("]\n"); }

        for( jj = 0, p = (double *) datax, q = (float *) codex; jj<m; jj++, p+=2, q+=2 ) {
            real = -p[0]*q[0] - p[1]*q[1];
            p[0] =  p[0]*q[1] - p[1]*q[0];
            p[1] =  real;
        }

#ifdef FFTMTX
        mlock(hfftmtx);
#endif
        fftw_execute( plan ); /* fft */
#ifdef FFTMTX
        unmlock(hfftmtx);
#endif

        for( jj=0, p = (double *) datax; jj<n; jj++, p+=2 ) P[ii*n + jj] += ( p[0]*p[0] + p[1]*p[1] ) * scale2 ;
    }


#ifdef FFTMTX
    mlock(hfftmtx);
#endif
    fftw_destroy_plan( plan );
#ifdef FFTMTX
    unmlock(hfftmtx);
#endif

    free(dataR);
    free(dataI);
    free(dataQ);
    fftw_free( datax );

#undef CSCALE
}

int checkacquisition( double *P, sdrch_t *sdr );

/* sdr acquisition function ----------------------------------------------------
* sdr acquisition function called from sdr channel thread
* args   : sdrch_t *sdr     I/O sdr channel struct
*          double *power    O   normalized correlation power vector (2D array)
* return : uint64_t             current buffer location
*-----------------------------------------------------------------------------*/
extern uint64_t acquisition( sdrch_t *sdr, double *power )
{
    int ii;
    char *data;
    uint64_t buffloc;

    uint32_t nsamp = sdr->nsamp;
    /* memory allocation */
    data = (char*) malloc( sizeof(char) * 2 * nsamp * sdr->dtype );

    /* current buffer location */
    mlock(hreadmtx);
    buffloc = ( sdrstat.fendbuffsize * sdrstat.buffcnt) - (sdr->acq.noncoherentintegration[0]+1)*nsamp;
    unmlock(hreadmtx);
    /* acquisition integration */
    for( ii=0; ii<sdr->acq.noncoherentintegration[0]; ii++ ) {
        /* get current 1ms data */
        rcvgetbuff( &sdrini, buffloc, 2*nsamp, sdr->ftype, sdr->dtype, data );
        buffloc += nsamp;

//if(1){ float *y = (float *) sdr->acq.xcode[0]; printf( "xcode=["); for( int x=0; x< 20; x++ ){ printf( " %f ", y[x] ); }; printf("\n"); }

        /* fft correlation */
        _pcorrelator( data, sdr->dtype, sdr->ti, nsamp, sdr->acq.freq, sdr->acq.numfreq[0], 
                    sdr->crate, sdr->acq.fftlen[0], sdr->acq.xcode[0], power );

        /* check acquisition result */
        if( _checkacquisition( power, sdr ) ) {
            sdr->flagacq = ii + 1;
            break;
        }
    }

    /* set acquisition result */
    if( sdr->flagacq > 0 ) {
        /* set buffer location at top of code */
        buffloc += -(ii+1)*nsamp + sdr->acq.acqcodei;
        sdr->trk.carrfreq = sdr->acq.acqfreq;
        sdr->trk.codefreq = sdr->crate;

        /* display acquisition results */
        debug_print( "%s, C/N0=%4.1f, peak=%3.1f, codei=%5d, freq=%8.1f **\n",
            sdr->satstr, sdr->acq.cn0, sdr->acq.peakr, sdr->acq.acqcodei,
            sdr->acq.acqfreq-sdr->f_if-sdr->foffset );

    }

    free(data);
    return buffloc;
}
/* check acquisition result ----------------------------------------------------
* check GNSS signal exists or not
* carrier frequency is computed
* args   : sdrch_t *sdr     I/0 sdr channel struct
*          double *P        I   normalized correlation power vector
* return : int                  acquisition flag (0: not acquired, 1: acquired) 
* note : first/second peak ratio and c/n0 computation
*-----------------------------------------------------------------------------*/
int _checkacquisition( double *P, sdrch_t *sdr )
{
    int maxi,codei,freqi,exinds,exinde;
    double maxP,maxP2,meanP;

    maxP = maxvd( P, sdr->nsamp*sdr->acq.numfreq[0], -1, -1, &maxi );
    ind2sub( maxi, sdr->nsamp, sdr->acq.numfreq[0], &codei, &freqi );

    /* C/N0 calculation */
    /* excluded index */
    exinds=codei-2*sdr->nsampchip; if(exinds<0) exinds+=sdr->nsamp;
    exinde=codei+2*sdr->nsampchip; if(exinde>=sdr->nsamp) exinde-=sdr->nsamp;
    meanP=meanvd(&P[freqi*sdr->nsamp],sdr->nsamp,exinds,exinde); /* mean */
    sdr->acq.cn0=10*log10(maxP/meanP/sdr->ctime);

    /* peak ratio */
    maxP2=maxvd(&P[freqi*sdr->nsamp],sdr->nsamp,exinds,exinde,&maxi);

    sdr->acq.peakr=maxP/maxP2;
    sdr->acq.acqcodei=codei;
    sdr->acq.freqi=freqi;
    sdr->acq.acqfreq=sdr->acq.freq[freqi];

    return sdr->acq.peakr>ACQTH;
}

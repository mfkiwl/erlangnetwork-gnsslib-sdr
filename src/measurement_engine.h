/**
 * \file measurement_engine.h
 * \brief constants, types and function for GNSS mesurement engine
 * 
 * \copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
 * \copyright (C) 2020 Shu Wang <shuwang1@outlook.com>
 * */
#ifndef MEASUREMENT_ENGINE_H
#define MEASUREMENT_ENGINE_H

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>
#include <ctype.h>
#include <assert.h>

#if defined(DEBUG_ENABLED)
#define DEBUG   1
#else
#define DEBUG   0
#endif

//#define debug_print(fmt, ...)   do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)
#define debug_print(fmt, ...)   do{ if( DEBUG ) fprintf( stderr, "%s:%03d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__ ); } while (0)

#include <pthread.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <inttypes.h>

#include "fec.h"
#include "fftw3.h"
#include "../lib/RTKLIB/src/rtklib.h"
#include "libusb-1.0/libusb.h"
#ifdef STEREO
#include "stereo.h"
#endif
#ifdef GN3S
#include "gn3s.h"
#endif
#ifdef BLADERF
#include "libbladeRF.h"
#endif
#ifdef RTLSDR
#include "rtl-sdr.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* constants -----------------------------------------------------------------*/
#define ROUND(x)      ((int)floor((x)+0.5)) /* round function */
#define PI            3.1415926535897932 /* pi */
#define DPI           (2.0*PI)         /* 2*pi */
#define D2R           (PI/180.0)       /* degree to radian */
#define R2D           (180.0/PI)       /* radian to degree */
#define CLIGHT        299792458.0      /* speed of light (m/s) */
#define ON            1                /* flag ON */
#define OFF           0                /* flag OFF */
#define MAXBITS       3000             /* maximum bit length */

/* front end setting */
#define FEND_STEREO   0                /* front end type: NSL STEREO */
#define FEND_GN3SV2   1                /* front end type: SiGe GN3S v2 */
#define FEND_GN3SV3   2                /* front end type: SiGe GN3S v3 */
#define FEND_RTLSDR   3                /* front end type: RTL-SDR */
#define FEND_BLADERF  4                /* front end type: Nuand BladeRF */
#define FEND_FSTEREO  5                /* front end type: STEREO binary file */
#define FEND_FGN3SV2  6                /* front end type: GN3Sv2 binary file */
#define FEND_FGN3SV3  7                /* front end type: GN3Sv3 binary file */
#define FEND_FRTLSDR  8                /* front end type: RTL-SDR binary file */
#define FEND_FBLADERF 9                /* front end type: BladeRF binary file */
#define FEND_FILE     10               /* front end type: IF file */
#define FTYPE1        1                /* front end number */
#define FTYPE2        2                /* front end number */
#define DTYPEI        1                /* sampling type: real */
#define DTYPEIQ       2                /* sampling type: real+imag */

#ifdef STEREOV26
#define STEREO_DATABUFF_SIZE STEREO_PKT_SIZE
#define MEMBUFFLEN    (STEREO_NUM_BLKS*32)
                                       /* number of temporary buffer */
#else
#define MEMBUFFLEN    5000             /* number of temporary buffer */
#endif

#define FILE_BUFFSIZE 65536            /* buffer size for post processing */

/* acquisition setting */
#define NFFTTHREAD    4               /* number of thread for executing FFT */
#define ACQINTG_L1CA  10               /* number of non-coherent integration */
#define ACQINTG_G1    10               /* number of non-coherent integration */
#define ACQINTG_SBAS  10               /* number of non-coherent integration */

#define ACQHBAND      5000             /* half width for doppler search (Hz) */
#define ACQSTEP       200              /* doppler search frequency step (Hz) */
#define ACQTH         3.0              /* acquisition threshold (peak ratio) */

/* tracking setting */
#define LOOP_L1CA     10               /* loop interval */
#define LOOP_G1       10               /* loop interval */
#define LOOP_SBAS     2                /* loop interval */

/* navigation parameter */
#define NAVSYNCTH       50             /* navigation frame synch. threshold */
/* GPS L1CA */
#define NAVRATE_L1CA    20             /* length (multiples of ranging code) */
#define NAVFLEN_L1CA    300            /* navigation frame data (bits) */
#define NAVADDFLEN_L1CA 2              /* additional bits of frame (bits) */
#define NAVPRELEN_L1CA  8              /* preamble bits length (bits) */
#define NAVEPHCNT_L1CA  3              /* number of eph. contained frame */
/* L1 SBAS */
#define NAVRATE_SBAS    2              /* length (multiples of ranging code) */
#define NAVFLEN_SBAS    1500           /* navigation frame data (bits) */
#define NAVADDFLEN_SBAS 12             /* additional bits of frame (bits) */
#define NAVPRELEN_SBAS  16             /* preamble bits length (bits) */
#define NAVEPHCNT_SBAS  3              /* number of eph. contained frame */
/* GLONASS G1 */
#define NAVRATE_G1      10             /* length (multiples of ranging code) */
#define NAVFLEN_G1      200            /* navigation frame data (bits) */
#define NAVADDFLEN_G1   0              /* additional bits of frame (bits) */
#define NAVPRELEN_G1    30             /* preamble bits length (bits) */
#define NAVEPHCNT_G1    5              /* number of eph. contained frame */

/* observation data generation */
#define PTIMING       68.802           /* pseudo range generation timing (ms) */
#define OBSINTERPN    80               /* # of obs. stock for interpolation */
#define SNSMOOTHMS    100              /* SNR smoothing interval (ms) */

/* code generation parameter */
#define MAXGPSSATNO   210              /* max satellite number */

/* code type */
#define CTYPE_L1CA    1                /* GPS L1C/A */
#define CTYPE_L1CP    2                /* GPS L1C Pilot */
#define CTYPE_L1CD    3                /* GPS L1C Data */
#define CTYPE_L1CO    4                /* GPS L1C overlay */
#define CTYPE_G1      20               /* GLONASS G1 */
#define CTYPE_G2      21               /* GLONASS G2 */
#define CTYPE_L1SBAS  27               /* SBAS compatible L1CA */
#define CTYPE_NH10    28               /* 10 bit Neuman-Hoffman code */
#define CTYPE_NH20    29               /* 20 bit Neuman-Hoffman code */

/* gnuplot plotting setting */
#define PLT_Y         1                /* plotting type: 1D data */
#define PLT_XY        2                /* plotting type: 2D data */
#define PLT_SURFZ     3                /* plotting type: 3D surface data */
#define PLT_BOX       4                /* plotting type: BOX */
#define PLT_WN        5                /* number of figure window column */
#define PLT_HN        4                /* number of figure window row */
#define PLT_W         200              /* window width (pixel) */
#define PLT_H         300              /* window height (pixel) */
#define PLT_MW        100              /* margin (pixel) */
#define PLT_MH        20                /* margin (pixel) */
#define PLT_MS        200              /* plotting interval (ms) */
#define PLT_MS_FILE   200               /* plotting interval (ms) */

/* spectrum analysis */
#define SPEC_MS       200              /* plotting interval (ms) */
#define SPEC_LEN      7                /* number of integration of 1 ms data */
#define SPEC_BITN     8                /* number of bin for histogram */
#define SPEC_NLOOP    100              /* number of loop for smoothing */
#define SPEC_NFFT     16384            /* number of FFT points */
#define SPEC_PLT_W    400              /* window width (pixel) */
#define SPEC_PLT_H    500              /* window height (pixel) */
#define SPEC_PLT_MW   100              /* margin (pixel) */
#define SPEC_PLT_MH   0                /* margin (pixel) */

/* SBAS setting */
#define LENSBASMSG    32               /* SBAS message length 150/8 (byte) */
#define LENSBASNOV    80               /* message length in NovAtel format */

/* thread functions */
#define mlock_t       pthread_mutex_t
#define initmlock(f)  pthread_mutex_init(&f,NULL)
#define mlock(f)      pthread_mutex_lock(&f)
#define unmlock(f)    pthread_mutex_unlock(&f)
#define delmlock(f)   pthread_mutex_destroy(&f)
#define event_t       pthread_cond_t
#define initevent(f)  pthread_cond_init(&f,NULL)
#define setevent(f)   pthread_cond_signal(&f)
#define waitevent(f,m) pthread_cond_wait(&f,&m)
#define delevent(f)   pthread_cond_destroy(&f)
#define waitthread(f) pthread_join(f,NULL)
#define cratethread(f,func,arg) pthread_create(&f,NULL,func,arg)
#define THRETVAL      NULL

/* sdr initialization struct */
typedef struct {
    int fend;            /* front end type */
    double f_cf[2];      /* center frequency (Hz) */
    double f_sf[2];      /* sampling frequency (Hz) */
    double f_if[2];      /* intermediate frequency (Hz) */
    int dtype[2];        /* data type (DTYPEI/DTYPEIQ) */
    FILE *fp1;           /* IF1 file pointer */
    FILE *fp2;           /* IF2 file pointer */
    char file1[1024];    /* IF1 file path */
    char file2[1024];    /* IF2 file path */
    int useif1;          /* IF1 flag */
    int useif2;          /* IF2 flag */
    int nch;             /* number of sdr channels */
    int nchL1;           /* number of L1 channels */
    int nchL2;           /* number of L2 channels */
    int nchL5;           /* number of L5 channels */
    int nchL6;           /* number of L6 channels */
    int prn[MAXSAT];     /* PRN of channels */
    int sys[MAXSAT];     /* satellite system type of channels (SYS_*) */
    int ctype[MAXSAT];   /* code type of channels (CTYPE_* )*/
    int ftype[MAXSAT];   /* front end type of channels (FTYPE1/FTYPE2) */
    int pltacq;          /* plot acquisition flag */
    int plttrk;          /* plot tracking flag */
    int pltspec;         /* plot spectrum flag */
    int outms;           /* output interval (ms) */
    int rinex;           /* rinex output flag */
    int rtcm;            /* rtcm output flag */
    int sbas;            /* SBAS output flag */
    int log;             /* tracking log output flag */
    char rinexpath[1024];/* rinex output path */
    char logpath[1024];     /* log output path, Added by Shu Wang, shuwang1@outlook.com on January 28, 2020 */ 
    int rtcmport;        /* rtcm TCP/IP port */
    int sbasport;        /* SBAS/L1-SAIF TCP/IP port */
    int trkcorrn;        /* number of correlation points */
    int trkcorrd;        /* interval of correlation points (sample) */
    int trkcorrp;        /* correlation points (sample) */
    double trkdllb[2];   /* dll noise bandwidth (Hz) */
    double trkpllb[2];   /* pll noise bandwidth (Hz) */
    double trkfllb[2];   /* fll noise bandwidth (Hz) */
    int rtlsdrppmerr;    /* clock collection for RTL-SDR */
} sdrini_t;

/* sdr current state struct */
typedef struct {
    int stopflag;        /* stop flag */
    int specflag;        /* spectrum flag */
    int buffsize;        /* data buffer size */
    int fendbuffsize;    /* front end data buffer size */
    unsigned char *buff; /* IF data buffer */
    unsigned char *buff2;/* IF data buffer (for file input) */
    unsigned char *tmpbuff; /* USB temporary buffer (for STEREO_V26) */
    uint64_t buffcnt;    /* current buffer location */
} sdrstat_t;

/* sdr observation struct */
typedef struct {
    int prn;             /* PRN */
    int sys;             /* satellite system */
    double tow;          /* time of week (s) */
    int week;            /* week number */
    double P;            /* pseudo range (m) */
    double L;            /* carrier phase (cycle) */
    double D;            /* doppler frequency (Hz) */
    double S;            /* SNR (dB-Hz) */
} sdrobs_t;

/* sdr acquisition struct */
typedef struct {

    double hband;        /* half band of search frequency (Hz) */
    double step;         /* frequency search step (Hz) */
    double *freq;        /* search frequency (Hz) */
    int acqcodei;        /* acquired code phase */
    int freqi;           /* acquired frequency index */
    double acqfreq;      /* acquired frequency (Hz) */
    double cn0;          /* signal C/N0 */ 
    double peakr;        /* first/second peak ratio */

    fftwf_complex *xcode[6];

    uint8_t noncoherentintegration[3];  /* number of integration */

    uint16_t coherencelen_code[3];
    uint32_t fftlen[3];                 /* number of FFT points */
    uint8_t fftlen_bit[3];

    uint16_t numfreq[3];                /*! number of search frequency 0 .. 65,535 */
    int16_t  *freq_4thHz[3];

} acquisition_t;

/* sdr tracking parameter struct */
typedef struct {
    double pllb;         /* noise bandwidth of PLL (Hz) */
    double dllb;         /* noise bandwidth of DLL (Hz) */
    double fllb;         /* noise bandwidth of FLL (Hz) */
    double dllw2;        /* DLL coefficient */
    double dllaw;        /* DLL coefficient */
    double pllw2;        /* PLL coefficient */
    double pllaw;        /* PLL coefficient */
    double fllw;         /* FLL coefficient */
} sdrtrkprm_t;

/* sdr tracking struct */
typedef struct {
    double codefreq;     /* code frequency (Hz) */
    double carrfreq;     /* carrier frequency (Hz) */
    double remcode;      /* remained code phase (chip) */
    double remcarr;      /* remained carrier phase (rad) */
    double oldremcode;   /* previous remained code phase (chip) */
    double oldremcarr;   /* previous remained carrier phase (chip) */
    double codeNco;      /* code NCO */
    double codeErr;      /* code tracking error */
    double carrNco;      /* carrier NCO */
    double carrErr;      /* carrier tracking error */
    double freqErr;      /* frequencyr error in FLL */
    uint64_t buffloc;    /* current buffer location */
    double tow[OBSINTERPN]; /* time of week (s) */
    uint64_t codei[OBSINTERPN]; /* code phase (sample) */
    uint64_t codeisum[OBSINTERPN]; /* code phase for SNR computation (sample) */
    uint64_t cntout[OBSINTERPN]; /* loop counter */
    double remcout[OBSINTERPN]; /* remained code phase (chip)*/
    double L[OBSINTERPN];/* carrier phase (cycle) */
    double D[OBSINTERPN];/* doppler frequency (Hz) */
    double S[OBSINTERPN];/* signal to noise ratio (dB-Hz) */
    double *II;          /* correlation (in-phase) */
    double *QQ;          /* correlation (quadrature-phase) */
    double *oldI;        /* previous correlation (I-phase) */
    double *oldQ;        /* previous correlation (Q-phase) */
    double *sumI;        /* integrated correlation (I-phase) */
    double *sumQ;        /* integrated correlation (Q-phase) */
    double *oldsumI;     /* previous integrated correlation (I-phase) */
    double *oldsumQ;     /* previous integrated correlation (Q-phase) */
    double Isum;         /* correlation for SNR computation (I-phase) */
    int loop;            /* loop filter interval */
    int loopms;          /* loop filter interval (ms) */
    int flagpolarityadd; /* polarity (half cycle ambiguity) add flag */
    int flagremcarradd;  /* remained carrier phase add flag */
    int flagloopfilter;  /* loop filter update flag */
    int corrn;           /* number of correlation points */
    int *corrp;          /* correlation points (sample) */
    double *corrx;       /* correlation points (for plotting) */
    int ne,nl;           /* early/late correlation point */
    sdrtrkprm_t prm1;    /* tracking parameter struct */
    sdrtrkprm_t prm2;    /* tracking parameter struct */
} sdrtrk_t;

/* sdr ephemeris struct */
typedef struct {
    eph_t eph;           /* GPS/GAL/COM ephemeris struct (from rtklib.h) */
    geph_t geph;         /* GLO ephemeris struct (from rtklib.h) */
    int ctype;           /* code type */
    double tow_gpst;     /* ephemeris tow in GPST */
    int week_gpst;       /* ephemeris week in GPST */
    int cnt;             /* ephemeris decode counter */
    int cntth;           /* ephemeris decode count threshold */
    int update;          /* ephemeris update flag */
    int prn;             /* PRN */
    int tk[3],nt,n4,s1cnt; /* temporary variables for decoding GLONASS */
    double toc_gst;      /* temporary variables for decoding Galileo */
    int week_gst;
                         /* temporary variables for decoding BeiDou D1 */
    unsigned int toe1,toe2;
                         /* temporary variables for decoding BeiDou D2 */
    int toe_bds,f1p3,cucp4,ep5,cicp6,i0p7,OMGdp8,omgp9;
    unsigned int f1p4,cucp5,ep6,cicp7,i0p8,OMGdp9,omgp10;
} sdreph_t;

/* sdr SBAS struct */
typedef struct {
    unsigned char msg[LENSBASMSG]; /* SBAS message (250bits/s) */
    unsigned char novatelmsg[LENSBASNOV]; /* NovAtel format (80bytes/s) */
    int id;              /* message ID */
    int week;            /* GPS week */
    double tow;          /* GPS time of week (s) */
} sdrsbas_t;

/* sdr navigation struct */
typedef struct {
    FILE *fpnav;         /* for navigation bit logging */
    int ctype;           /* code type */
    int rate;            /* navigation data rate (ms) */
    int flen;            /* frame length (bits) */
    int addflen;         /* additional frame bits (bits) */
    int prebits[32];     /* preamble bits */
    int prelen;          /* preamble bits length (bits) */
    int bit;             /* current navigation bit */
    int biti;            /* current navigation bit index */
    int cnt;             /* navigation bit counter for synchronization */
    double bitIP;        /* current navigation bit (IP data) */
    int *fbits;          /* frame bits */
    int *fbitsdec;       /* decoded frame bits */
    int update;          /* decode interval (ms) */
    int *bitsync;        /* frame bits synchronization count */
    int synci;           /* frame bits synchronization index */
    uint64_t firstsf;    /* first subframe location (sample) */
    uint64_t firstsfcnt; /* first subframe count */
    double firstsftow;   /* tow of first subframe */
    int polarity;        /* bit polarity */
    int flagpol;         /* bit polarity flag (only used in L1-SAIF) */
    void *fec;           /* FEC (fec.h)  */
    short *ocode;        /* overlay code (secondary code) */
    int ocodei;          /* current overray code index */
    int swsync;          /* switch of frame synchronization (last nav bit) */
    int swreset;         /* switch of frame synchronization (first nav bit) */
    int swloop;          /* switch of loop filter */
    int flagsync;        /* navigation bit synchronization flag */
    int flagsyncf;       /* navigation frame synchronization (preamble found) */
    int flagtow;         /* first subframe found flag */
    int flagdec;         /* navigation data decoded flag */
    sdreph_t sdreph;     /* sdr ephemeris struct */
    sdrsbas_t sbas;      /* SBAS message struct */
} navigation_t;

/* sdr channel struct */
typedef struct {
    thread_t hsdr;       /* thread handle */
    int no;              /* channel number */
    int sat;             /* satellite number */
    int sys;             /* satellite system */
    int prn;             /* PRN */
    char satstr[5];      /* PRN string */
    int ctype;           /* code type */
    int dtype;           /* data type */
    int ftype;           /* front end type */
    double f_cf;         /* carrier frequency (Hz) */
    double f_sf;         /* sampling rate (Hz) */
    double f_if;         /* intermediate frequency (Hz) */
    double foffset;      /* frequency offset (Hz) */
    short *code;         /* original code */
    fftw_complex *xcode;        /* resampled code in frequency domain */
    fftw_complex *codex;        /* resampled code in frequency domain */
    int clen;            /* code length */
    double crate;        /* code chip rate (Hz) */
    double ctime;        /* code period (s) */
    double ti;           /* sampling interval (s) */
    double ci;           /* chip interval (s) */
    int nsamp;           /* number of samples in one code (doppler=0Hz) */
    int currnsamp;       /* current number of samples in one code */
    int nsampchip;       /* number of samples in one code chip (doppler=0Hz) */
    acquisition_t acq;        /* acquisition struct */
    sdrtrk_t trk;        /* tracking struct */
    navigation_t nav;        /* navigation struct */
    int flagacq;         /* acquisition flag */
    int flagtrk;         /* tracking flag */
} sdrch_t;

/* sdr plotting struct */
typedef struct {
    FILE *fp;            /* file pointer (gnuplot pipe) */
    int nx;              /* length of x data */
    int ny;              /* length of y data */
    double *x;           /* x data */
    double *y;           /* y data */
    double *z;           /* z data */
    int type;            /* plotting type (PLT_X/PLT_XY/PLT_SURFZ) */
    int skip;            /* skip data (0: plotting all data) */
    int flagabs;         /* y axis data absolute flag (y=abs(y)) */
    double scale;        /* y axis data scale (y=scale*y) */
    int plth;            /* plot window height */
    int pltw;            /* plot window width */
    int pltmh;           /* plot window margin height */
    int pltmw;           /* plot window margin width */
    int pltno;           /* number of figure */
    double pltms;        /* plot interval (ms) */
} sdrplt_t;

/* sdr socket struct */
typedef struct {
    thread_t hsoc;       /* thread handle */
    int port;            /* port number */
    int s_soc,c_soc;     /* server/client socket */
    int flag;            /* connection flag */
} sdrsoc_t;

/* sdr output struct */
typedef struct {
    int nsat;            /* number of satellite */
    obsd_t *obsd;        /* observation struct (defined in rtklib.h) */
    rnxopt_t opt;        /* rinex option struct (defined in rtklib.h) */
    sdrsoc_t soc_rtcm;   /* sdr socket struct for rtcm output */
    sdrsoc_t soc_sbas;   /* sdr socket struct for sbas output */
    char rinexobs[1024]; /* rinex observation file name */
    char rinexnav[1024]; /* rinex navigation file name */
} sdrout_t;

/* sdr spectrum struct */
typedef struct {
    int dtype;           /* data type (DTYPEI/DTYPEIQ) */
    int ftype;           /* front end type */
    int nsamp;           /* number of samples in one code */
    double f_sf;         /* sampling frequency (Hz) */
    sdrplt_t histI;      /* plot struct for histogram */
    sdrplt_t histQ;      /* plot struct for histogram */
    sdrplt_t pspec;      /* plot struct for spectrum analysis */
} sdrspec_t;

/* global variables ----------------------------------------------------------*/
extern thread_t hmainthread;  /* main thread handle */
extern thread_t hsyncthread;  /* synchronization thread handle */
extern thread_t hspecthread;  /* spectrum analyzer thread handle */
extern thread_t hkeythread;   /* keyboard thread handle */
extern mlock_t hbuffmtx;      /* buffer access mutex */
extern mlock_t hreadmtx;      /* buffloc access mutex */
extern mlock_t hfftmtx;       /* fft function mutex */
extern mlock_t hpltmtx;       /* plot function mutex */
extern mlock_t hobsmtx;       /* observation data access mutex */

extern sdrini_t sdrini;       /* sdr initialization struct */
extern sdrstat_t sdrstat;     /* sdr state struct */
extern sdrch_t sdrch[MAXSAT]; /* sdr channel structs */
extern sdrspec_t sdrspec;     /* sdr spectrum analyzer structs */
extern sdrout_t sdrout;       /* sdr output structs */

/** main.c **/
extern void start_erlangnetwork_gnssmeasurementengine(void);
extern void quit_erlangnetwork_gnssmeasurementengine(sdrini_t *ini, int stop);

/** statemachine.c **/
extern void *statemachinethread(void *arg);

/* sdrsync.c -----------------------------------------------------------------*/
extern void *syncthread(void * arg);

/** acquisition.c **/
extern uint64_t acquisition(sdrch_t *sdr, double *power);

/* tracking.c **/
extern uint64_t tracking(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt);
extern void cumsumcorr(sdrtrk_t *trk, int polarity);
extern void clearcumsumcorr(sdrtrk_t *trk);
extern void pll(sdrch_t *sdr, sdrtrkprm_t *prm, double dt);
extern void dll(sdrch_t *sdr, sdrtrkprm_t *prm, double dt);
extern void setobsdata(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt, 
                       sdrtrk_t *trk, int flag);
extern void dot_22(const short *a1, const short *a2, const short *b1, 
                   const short *b2, int n, double *d1, double *d2);
extern void dot_23(const short *a1, const short *a2, const short *b1, 
                   const short *b2, const short *b3, int n, double *d1, 
                   double *d2);
extern void correlator(const char *data, int dtype, double ti, int n, 
                       double freq, double phi0, double crate, double coff, 
                       int* s, int ns, double *II, double *QQ, double *remc, 
                       double *remp, short* codein, int coden);

/** acquisition_initialization.c **/
int initialize_acqusition_struct( 
                acquisition_t *acq, 
                int ctype,
                uint16_t *coherencelen_code,
                uint16_t codelen_sample,
                double intfreq_Hz,
                double offset_Hz );
extern int initialize_measurement_channel(int chno,
                        int sys, 
                        int prn, 
                        int ctype, 
                        int dtype,
                        int ftype, 
                        double f_cf, 
                        double f_sf, 
                        double f_if,
                        sdrch_t *sdr );

/** tracking_initialization.c  ***/
extern int initialize_tracking_structure( int sat, int ctype, double ctime, sdrtrk_t *trk );
extern int initialize_navigation_structure(int sys, int ctype, int prn, navigation_t *nav);

/* sdrinit.c -----------------------------------------------------------------*/
extern int readinifile( sdrini_t *ini, const char *inifile );  /** Modified by Shu Wang, shuwang1@outlook.com, on January 28, 2020*/
extern int chk_initvalue(sdrini_t *ini);
extern void openhandles(void);
extern void closehandles(void);
extern int initpltstruct(sdrplt_t *acq, sdrplt_t *trk,sdrch_t *sdr);
extern void quitpltstruct(sdrplt_t *acq, sdrplt_t *trk);
extern void initacqstruct(int sys, int ctype, int prn, acquisition_t *acq);
extern void inittrkprmstruct(sdrtrk_t *trk);
extern int inittrkstruct(int sat, int ctype, double ctime, sdrtrk_t *trk);
extern int initnavstruct(int sys, int ctype, int prn, navigation_t *nav);
extern int initsdrch(int chno, int sys, int prn, int ctype, int dtype, 
                     int ftype, double f_cf, double f_sf, double f_if,
                     sdrch_t *sdr);
extern void freesdrch(sdrch_t *sdr);

/* sdrcmn.c ------------------------------------------------------------------*/
extern int getfullpath(char *relpath, char *abspath);
extern unsigned long tickgetus(void);
extern void settimeout(struct timespec *timeout, int waitms);
extern int calcfftnum(double x, int next);
extern void cpxfft( fftw_plan plan, fftw_complex *cpx, int n );
extern void cpxifft( fftw_plan plan, fftw_complex *cpx, int n );
extern void cpxcpx( const short *II, const short *QQ, double scale, int n, fftw_complex *cpx);
extern double maxvd(const double *data, int n, int exinds, int exinde, int *ind);
extern double meanvd(const double *data, int n, int exinds, int exinde);
extern void cpxconv( fftw_plan plan, fftw_plan iplan, fftw_complex *cpxa, fftwf_complex *cpxb, int m, int n, int flagsum, double *conv);
extern void cpxpspec( fftw_plan plan, fftw_complex *cpx, int n, int flagsum, double *pspec);
extern double mixcarr( const char *data, int dtype, double ti, int n,  double freq, double phi0, short *II, short *QQ);
extern double interp1(double *x, double *y, int n, double t);
extern void uint64todouble(uint64_t *data, uint64_t base, int n, double *out);
extern void ind2sub(int ind, int nx, int ny, int *subx, int *suby);
extern void shiftdata(void *dst, void *src, size_t size, int n);
extern double rescode(const short *code, int len, double coff, int smax, 
                      double ci, int n, short *rcode);
/* sdrcode.c -----------------------------------------------------------------*/
extern short *gencode(int prn, int ctype, int *len, double *crate);

/* sdrplot.c -----------------------------------------------------------------*/
extern int updatepltini(int nx, int ny, int posx, int posy);
extern void setsdrplotprm(sdrplt_t *plt, int type, int nx, int ny, int skip, 
                       int abs, double s, int h, int w, int mh, int mw, int no);
extern int initsdrplot(sdrplt_t *plt);
extern void quitsdrplot(sdrplt_t *plt);
extern void setxrange(sdrplt_t *plt, double xmin, double xmax);
extern void setyrange(sdrplt_t *plt, double ymin, double ymax);
extern void setlabel(sdrplt_t *plt, char *xlabel, char *ylabel);
extern void settitle(sdrplt_t *plt, char *title);
extern void ploty(FILE *fp, double *x, int n, int skip, double scale);
extern void plotxy(FILE *fp, double *x, double *y, int n, int skip, double s);
extern void plotsurfz(FILE *fp, double*z, int nx, int ny, int skip, double s);
extern void plotbox(FILE *fp, double *x, double *y, int n, int skip, double s);
extern void plotthread(sdrplt_t *plt);
extern void plot(sdrplt_t *plt);

/* navigation.c ------------------------------------------------------------------*/
extern void navigation(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt);
extern uint32_t getbitu2(const uint8_t *buff, int p1, int l1, int p2, int l2);
extern int32_t getbits2(const uint8_t *buff, int p1, int l1, int p2, int l2);
extern uint32_t getbitu3(const uint8_t *buff, int p1, int l1, int p2, int l2, 
                         int p3, int l3);
extern int32_t getbits3(const uint8_t *buff, int p1, int l1, int p2, int l2,
                        int p3, int l3);
extern uint32_t merge_two_u(const uint32_t a, const uint32_t b, int n);
extern int32_t merge_two_s(const int32_t a, const uint32_t b, int n);
extern void bits2byte(int *bits, int nbits, int nbin, int right, uint8_t *bin);
extern void interleave(const int *in, int row, int col, int *out);
extern int checksync(double IP, double IPold, navigation_t *nav);
extern int checkbit(double IP, int loopms, navigation_t *nav);
extern void predecodefec(navigation_t *nav);
extern int paritycheck(navigation_t *nav);
extern int findpreamble(navigation_t *nav);
extern int decodenav(navigation_t *nav);
extern void check_hamming(int *hamming, int n, int parity, int m);
extern int maxvi(const int *data, int n, int exinds, int exinde, int *ind);
/* sdrnav_gps/gal/glo.c/sbs.c ------------------------------------------------*/
extern int decode_l1ca(navigation_t *nav);
extern int decode_e1b(navigation_t *nav);
extern int decode_g1(navigation_t *nav);
extern int decode_b1i(navigation_t *nav);
extern int decode_l1sbas(navigation_t *nav);
extern int paritycheck_l1ca(int *bits);

/* sdrout.c ------------------------------------------------------------------*/
extern void createrinexopt(rnxopt_t *opt);
extern void sdrobs2obsd(sdrobs_t *sdrobs, int ns, obsd_t *out);
extern int createrinexobs(char *file, rnxopt_t *opt);
extern int writerinexobs(char *file, rnxopt_t *opt, obsd_t *obsd, int ns);
extern int createrinexnav(char *file, rnxopt_t *opt);
extern int writerinexnav(char *file, rnxopt_t *opt, sdreph_t *sdreph);
extern void tcpsvrstart(sdrsoc_t *soc);
extern void tcpsvrclose(sdrsoc_t *soc);
extern void sendrtcmnav(sdreph_t *eph, sdrsoc_t *soc);
extern void sendrtcmobs(obsd_t *obsd, sdrsoc_t *soc, int nsat);
extern void sendsbas(sdrsbas_t *sbas, sdrsoc_t *soc);
extern void writelog(FILE *fp, sdrtrk_t *trk,navigation_t *nav);
extern FILE* createlog(char *filename, sdrtrk_t *trk);
extern void closelog(FILE *fp);

/* sdrrcv.c ------------------------------------------------------------------*/
extern int rcvinit(sdrini_t *ini);
extern int rcvquit(sdrini_t *ini);
extern int rcvgrabstart(sdrini_t *ini);
extern int rcvgrabdata(sdrini_t *ini);
extern int rcvgetbuff(sdrini_t *ini, uint64_t buffloc, int n, int ftype, 
                      int dtype, char *expbuf);
extern void file_pushtomembuf(void);
extern void file_getbuff(uint64_t buffloc, int n, int ftype, int dtype, 
                         char *expbuf);

/* sdrspec.c -----------------------------------------------------------------*/
extern void initsdrspecgui(sdrspec_t* sdrspecgui);
extern void *specthread(void * arg);
extern int initspecpltstruct(sdrspec_t *spec);
extern void quitspecpltstruct(sdrspec_t *spec);
extern void calchistgram(char *data, int dtype, int n, double *xI, double *yI, 
                         double *xQ, double *yQ);
extern void hanning(int n, float *win);
extern int spectrumanalyzer(const char *data, int dtype, int n, double f_sf, 
                            int nfft, double *freq, double *pspec);

#ifdef __cplusplus
}
#endif
#endif /* MEASUREMENT_ENGINE_H */
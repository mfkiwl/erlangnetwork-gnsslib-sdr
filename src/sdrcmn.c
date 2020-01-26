/*------------------------------------------------------------------------------
* sdrcmn.c : SDR common functions
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
* Copyright (C) 2014 T. Takasu <http://www.rtklib.com>
*-----------------------------------------------------------------------------*/
#include "sdr.h"

#define CDIV          32               /* carrier lookup table (cycle) */
#define CMASK         0x1F             /* carrier lookup table mask */
#define CSCALE        (1.0/32.0)       /* carrier lookup table scale (LSB) */

/* get full path from relative path --------------------------------------------
* args   : char *relpath    I   relative path
*          char *fullpath   O   full path
* return : int                  0:success, -1:failure 
*-----------------------------------------------------------------------------*/
extern int getfullpath(char *relpath, char *abspath)
{
#ifdef WIN32
    if (_fullpath(abspath,relpath,_MAX_PATH)==NULL) {
        SDRPRINTF("error: getfullpath %s\n",relpath);
        return -1;
    }
#else
    if (realpath(relpath,abspath)==NULL) {
        SDRPRINTF("error: getfullpath %s\n",relpath);
        return -1;
    }
#endif
    return 0;
}
/* get tick time (micro second) ------------------------------------------------
* get current tick in ms
* args   : none
* return : current tick in us
*-----------------------------------------------------------------------------*/
extern unsigned long tickgetus(void)
{
#ifdef WIN32
    LARGE_INTEGER f,t;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (unsigned long)(t.QuadPart*1000000UL/f.QuadPart);
#else
    struct timespec tp={0};
    struct timeval  tv={0};
    
#ifdef CLOCK_MONOTONIC_RAW
    /* linux kernel > 2.6.28 */
    if (!clock_gettime(CLOCK_MONOTONIC_RAW,&tp)) {
        return tp.tv_sec*1000000UL+tp.tv_nsec/1000UL;
    }
    else {
        gettimeofday(&tv,NULL);
        return tv.tv_sec*1000000UL+tv.tv_usec;
    }
#else
    gettimeofday(&tv,NULL);
    return (unsigned long)(tv.tv_sec*1000000UL+tv.tv_usec);
#endif
#endif
}
/* calculation log2(x) ---------------------------------------------------------
* args   : double x         I   x data
* return : double               log2(x)
*-----------------------------------------------------------------------------*/
extern double log2(double x)  
{  
    return log(x)/log(2.0);  
}
/* calculation FFT number of points (2^bits samples) ---------------------------
* calculation FFT number of points (round up)
* args   : double x         I   number of points (not 2^bits samples)
*          int    next      I   increment multiplier
* return : int                  FFT number of points (2^bits samples)
*-----------------------------------------------------------------------------*/
extern int calcfftnum(double x, int next)
{
    int nn=(int)(log2(x)+0.5)+next;
    return (int)pow(2.0,nn);
}
/* sdr malloc ------------------------------------------------------------------
* memorry allocation
* args   : int    size      I   sizee of allocation
* return : void*                allocated pointer
*-----------------------------------------------------------------------------*/
extern void *sdrmalloc(size_t size)
{
#if defined(WIN32)
    return _aligned_malloc(size,16);
#else
    return malloc(size);
#endif
}
/* sdr free --------------------------------------------------------------------
* free data
* args   : void   *p        I/O input/output complex data
* return : none
*-----------------------------------------------------------------------------*/
extern void sdrfree(void *p)
{
#if defined(WIN32)
    _aligned_free(p);
#else
    free(p);
#endif
}
/* complex malloc --------------------------------------------------------------
* memorry allocation of complex data
* args   : int    n         I   number of allocation
* return : cpx_t*               allocated pointer
*-----------------------------------------------------------------------------*/
extern cpx_t *cpxmalloc(int n)
{
    return (cpx_t *)fftwf_malloc(sizeof(cpx_t)*n+32);
}
/* complex free ----------------------------------------------------------------
* free complex data
* args   : cpx_t  *cpx      I/O input/output complex data
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxfree(cpx_t *cpx)
{
    fftwf_free(cpx);
}
/* complex FFT -----------------------------------------------------------------
* cpx=fft(cpx)
* args   : fftwf_plan plan  I   fftw plan (NULL: create new plan)
*          cpx_t  *cpx      I/O input/output complex data
*          int    n         I   number of input/output data
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxfft(fftwf_plan plan, cpx_t *cpx, int n)
{
#ifdef FFTMTX
        mlock(hfftmtx);
#endif
    if (plan==NULL) {
        fftwf_plan_with_nthreads(NFFTTHREAD); /* fft execute in multi threads */
        plan=fftwf_plan_dft_1d(n,cpx,cpx,FFTW_FORWARD,FFTW_ESTIMATE);
        fftwf_execute_dft(plan,cpx,cpx); /* fft */
        fftwf_destroy_plan(plan);
    } else {
        fftwf_execute_dft(plan,cpx,cpx); /* fft */
    }
#ifdef FFTMTX
        unmlock(hfftmtx);
#endif
}
/* complex IFFT ----------------------------------------------------------------
* cpx=ifft(cpx)
* args   : fftwf_plan plan  I   fftw plan (NULL: create new plan)
*          cpx_t  *cpx      I/O input/output complex data
*          int    n         I   number of input/output data
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxifft(fftwf_plan plan, cpx_t *cpx, int n)
{
#ifdef FFTMTX
        mlock(hfftmtx);
#endif
    if (plan==NULL) {
        fftwf_plan_with_nthreads(NFFTTHREAD); /* fft execute in multi threads */
        plan=fftwf_plan_dft_1d(n,cpx,cpx,FFTW_BACKWARD,FFTW_ESTIMATE);
        fftwf_execute_dft(plan,cpx,cpx); /* fft */
        fftwf_destroy_plan(plan);
    } else {
        fftwf_execute_dft(plan,cpx,cpx); /* fft */
    }
#ifdef FFTMTX
        unmlock(hfftmtx);
#endif

}
/* convert short vector to complex vector --------------------------------------
* cpx=complex(I,Q)
* args   : short  *I        I   input data array (real)
*          short  *Q        I   input data array (imaginary)
*          double scale     I   scale factor
*          int    n         I   number of input data
*          cpx_t *cpx       O   output complex array
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxcpx(const short *II, const short *QQ, double scale, int n, 
                   cpx_t *cpx)
{
    float *p=(float *)cpx;
    int i;

    for (i=0;i<n;i++,p+=2) {
        p[0]=   II[i]*(float)scale;
        p[1]=QQ?QQ[i]*(float)scale:0.0f;
    }
}
/* convert float vector to complex vector --------------------------------------
* cpx=complex(I,Q)
* args   : float  *I        I   input data array (real)
*          float  *Q        I   input data array (imaginary)
*          double scale     I   scale factor
*          int    n         I   number of input data
*          cpx_t *cpx       O   output complex array
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxcpxf(const float *II, const float *QQ, double scale, int n, 
                    cpx_t *cpx)
{
    float *p=(float *)cpx;
    int i;

    for (i=0;i<n;i++,p+=2) {
        p[0]=   II[i]*(float)scale;
        p[1]=QQ?QQ[i]*(float)scale:0.0f;
    }
}
/* FFT convolution -------------------------------------------------------------
* conv=sqrt(abs(ifft(fft(cpxa).*conj(cpxb))).^2) 
* args   : fftwf_plan plan  I   fftw plan (NULL: create new plan)
*          fftwf_plan iplan I   ifftw plan (NULL: create new plan)
*          cpx_t  *cpxa     I   input complex data array
*          cpx_t  *cpxb     I   input complex data array
*          int    m         I   number of input data
*          int    n         I   number of output data
*          int    flagsum   I   cumulative sum flag (conv+=conv)
*          double *conv     O   output convolution data
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxconv(fftwf_plan plan, fftwf_plan iplan, cpx_t *cpxa, cpx_t *cpxb,
                    int m, int n, int flagsum, double *conv)
{
    float *p,*q,real,m2=(float)m*m;
    int i;

    cpxfft(plan,cpxa,m); /* fft */

    for (i=0,p=(float *)cpxa,q=(float *)cpxb;i<m;i++,p+=2,q+=2) {
        real=-p[0]*q[0]-p[1]*q[1];
        p[1]= p[0]*q[1]-p[1]*q[0];
        p[0]=real;
    }

    cpxifft(iplan,cpxa,m); /* ifft */

    if (flagsum) { /* cumulative sum */
        for (i=0,p=(float *)cpxa;i<n;i++,p+=2)
            conv[i]+=(p[0]*p[0]+p[1]*p[1])/m2;
    } else {
        for (i=0,p=(float *)cpxa;i<n;i++,p+=2)
            conv[i]=(p[0]*p[0]+p[1]*p[1])/m2;
    }
}
/* power spectrum calculation --------------------------------------------------
* power spectrum: pspec=abs(fft(cpx)).^2
* args   : fftwf_plan plan  I   fftw plan (NULL: create new plan)
*          cpx_t  *cpx      I   input complex data array
*          int    n         I   number of input data
*          int    flagsum   I   cumulative sum flag (pspec+=pspec)
*          double *pspec    O   output power spectrum data
* return : none
*-----------------------------------------------------------------------------*/
extern void cpxpspec(fftwf_plan plan, cpx_t *cpx, int n, int flagsum,
                     double *pspec)
{
    float *p;
    int i;

    cpxfft(plan,cpx,n); /* fft */

    if (flagsum) { /* cumulative sum */
        for (i=0,p=(float *)cpx;i<n;i++,p+=2)
            pspec[i]+=(p[0]*p[0]+p[1]*p[1]);
    } else {
        for (i=0,p=(float *)cpx;i<n;i++,p+=2)
            pspec[i]=(p[0]*p[0]+p[1]*p[1]);
    }
}
/* dot products: d1=dot(a1,b),d2=dot(a2,b) -------------------------------------
* args   : short  *a1       I   input short array
*          short  *a2       I   input short array
*          short  *b        I   input short array
*          int    n         I   number of input data
*          double *d1       O   output short array
*          double *d2       O   output short array
* return : none
* notes  : -128<=a1[i],a2[i],b[i]<127
*-----------------------------------------------------------------------------*/
extern void dot_21(const short *a1, const short *a2, const short *b, int n,
                   double *d1, double *d2)
{
    const short *p1=a1,*p2=a2,*q=b;

    d1[0]=d2[0]=0.0;

    for (;p1<a1+n;p1++,p2++,q++) {
        d1[0]+=(*p1)*(*q);
        d2[0]+=(*p2)*(*q);
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
/* multiply char/short vectors -------------------------------------------------
* multiply char/short vectors: out=data1.*data2
* args   : char   *data1    I   input char array
*          short  *data2    I   input short array
*          int    n         I   number of input data
*          short  *out      O   output short array
* return : none
*-----------------------------------------------------------------------------*/
extern void mulvcs(const char *data1, const short *data2, int n, short *out)
{
    int i;
    for (i=0;i<n;i++) out[i]=data1[i]*data2[i];
}
/* sum float vectors -----------------------------------------------------------
* sum float vectors: out=data1.+data2
* args   : float  *data1    I   input float array
*          float  *data2    I   input float array
*          int    n         I   number of input data
*          float  *out      O   output float array
* return : none
* note   : AVX command is used if "AVX" is defined
*-----------------------------------------------------------------------------*/
extern void sumvf(const float *data1, const float *data2, int n, float *out)
{
    int i;
    for (i=0;i<n;i++) out[i]=data1[i]+data2[i];
}
/* sum double vectors ----------------------------------------------------------
* sum double vectors: out=data1.+data2
* args   : double *data1    I   input double array
*          double *data2    I   input double array
*          int    n         I   number of input data
*          double *out      O   output double array
* return : none
* note   : AVX command is used if "AVX" is defined
*-----------------------------------------------------------------------------*/
extern void sumvd(const double *data1, const double *data2, int n, double *out)
{
    int i;
    for (i=0;i<n;i++) out[i]=data1[i]+data2[i];
}
/* maximum value and index (int array) -----------------------------------------
* calculate maximum value and index
* args   : double *data     I   input int array
*          int    n         I   number of input data
*          int    exinds    I   exception index (start)
*          int    exinde    I   exception index (end)
*          int    *ind      O   index at maximum value
* return : int                  maximum value
* note   : maximum value and index are calculated without exinds-exinde index
*          exinds=exinde=-1: use all data
*-----------------------------------------------------------------------------*/
extern int maxvi(const int *data, int n, int exinds, int exinde, int *ind)
{
    int i;
    int max=data[0];
    *ind=0;
    for(i=1;i<n;i++) {
        if ((exinds<=exinde&&(i<exinds||i>exinde))||
            (exinds> exinde&&(i<exinds&&i>exinde))) {
                if (max<data[i]) {
                    max=data[i];
                    *ind=i;
                }
        }
    }
    return max;
}
/* maximum value and index (float array) ---------------------------------------
* calculate maximum value and index
* args   : float  *data     I   input float array
*          int    n         I   number of input data
*          int    exinds    I   exception index (start)
*          int    exinde    I   exception index (end)
*          int    *ind      O   index at maximum value
* return : float                maximum value
* note   : maximum value and index are calculated without exinds-exinde index
*          exinds=exinde=-1: use all data
*-----------------------------------------------------------------------------*/
extern float maxvf(const float *data, int n, int exinds, int exinde, int *ind)
{
    int i;
    float max=data[0];
    *ind=0;
    for(i=1;i<n;i++) {
        if ((exinds<=exinde&&(i<exinds||i>exinde))||
            (exinds> exinde&&(i<exinds&&i>exinde))) {
                if (max<data[i]) {
                    max=data[i];
                    *ind=i;
                }
        }
    }
    return max;
}
/* maximum value and index (double array) --------------------------------------
* calculate maximum value and index
* args   : double *data     I   input double array
*          int    n         I   number of input data
*          int    exinds    I   exception index (start)
*          int    exinde    I   exception index (end)
*          int    *ind      O   index at maximum value
* return : double               maximum value
* note   : maximum value and index are calculated without exinds-exinde index
*          exinds=exinde=-1: use all data
*-----------------------------------------------------------------------------*/
extern double maxvd(const double *data, int n, int exinds, int exinde, int *ind)
{
    int i;
    double max=data[0];
    *ind=0;
    for(i=1;i<n;i++) {
        if ((exinds<=exinde&&(i<exinds||i>exinde))||
            (exinds> exinde&&(i<exinds&&i>exinde))) {
                if (max<data[i]) {
                    max=data[i];
                    *ind=i;
                }
        }
    }
    return max;
}
/* mean value (double array) ---------------------------------------------------
* calculate mean value
* args   : double *data     I   input double array
*          int    n         I   number of input data
*          int    exinds    I   exception index (start)
*          int    exinde    I   exception index (end)
* return : double               mean value
* note   : mean value is calculated without exinds-exinde index
*          exinds=exinde=-1: use all data
*-----------------------------------------------------------------------------*/
extern double meanvd(const double *data, int n, int exinds, int exinde)
{
    int i,ne=0;
    double mean=0.0;
    for(i=0;i<n;i++) {
        if ((exinds<=exinde)&&(i<exinds||i>exinde)) mean+=data[i];
        else if ((exinds>exinde)&&(i<exinds&&i>exinde)) mean+=data[i];
        else ne++;
    }
    return mean/(n-ne);
}
/* 1D interpolation ------------------------------------------------------------
* interpolation of 1D data
* args   : double *x,*y     I   x and y data array
*          int    n         I   number of input data
*          double t         I   interpolation point on x data
* return : double               interpolated y data at t
*-----------------------------------------------------------------------------*/
extern double interp1(double *x, double *y, int n, double t)
{
    int i,j,k,m;
    double z,s,*xx,*yy;
    z=0.0;
    if(n<1) return(z);
    if(n==1) {z=y[0];return(z);}
    if(n==2) {
        z=(y[0]*(t-x[1])-y[1]*(t-x[0]))/(x[0]-x[1]);
        return(z);
    }

    xx=(double*)malloc(sizeof(double)*n);
    yy=(double*)malloc(sizeof(double)*n);
    if (x[0]>x[n-1]) {
        for (j=n-1,k=0;j>=0;j--,k++) {
            xx[k]=x[j];
            yy[k]=y[j];
        }
    } else {
        memcpy(xx,x,sizeof(double)*n);
        memcpy(yy,y,sizeof(double)*n);
    }

    if(t<=xx[1]) {k=0;m=2;}
    else if(t>=xx[n-2]) {k=n-3;m=n-1;}
    else {
        k=1;m=n;
        while (m-k!=1) {
            i=(k+m)/2;
            if (t<xx[i-1]) m=i;
            else k=i;
        }
        k=k-1; m=m-1;
        if(fabs(t-xx[k])<fabs(t-xx[m])) k=k-1;
        else m=m+1;
    }
    z=0.0;
    for (i=k;i<=m;i++) {
        s=1.0;
        for (j=k;j<=m;j++)
            if (j!=i) s=s*(t-xx[j])/(xx[i]-xx[j]);
        z=z+s*yy[i];
    }

    free(xx); free(yy);
    return z;
}
/* convert uint64_t to double --------------------------------------------------
* convert uint64_t array to double array (subtract base value)
* args   : uint64_t *data   I   input uint64_t array
*          uint64_t base    I   base value
*          int    n         I   number of input data
*          double *out      O   output double array
* return : none
*-----------------------------------------------------------------------------*/
extern void uint64todouble(uint64_t *data, uint64_t base, int n, double *out)
{
    int i;
    for (i=0;i<n;i++) out[i]=(double)(data[i]-base);
}
/* index to subscribe ----------------------------------------------------------
* 1D index to subscribe (index of 2D array)
* args   : int    *ind      I   input data
*          int    nx, ny    I   number of row and column
*          int    *subx     O   subscript index of x
*          int    *suby     O   subscript index of y
* return : none
*-----------------------------------------------------------------------------*/
extern void ind2sub(int ind, int nx, int ny, int *subx, int *suby)
{
    *subx = ind%nx;
    *suby = ny*ind/(nx*ny);
}
/* vector shift function  ------------------------------------------------------
* shift of vector data
* args   : void   *dst      O   output shifted data
*          void   *src      I   input data
*          size_t size      I   type of input data (byte)
*          int    n         I   number of input data
* return : none
*-----------------------------------------------------------------------------*/
extern void shiftdata(void *dst, void *src, size_t size, int n)
{
    void *tmp;
    tmp=malloc(size*n);
    if (tmp!=NULL) {
        memcpy(tmp,src,size*n);
        memcpy(dst,tmp,size*n);
        free(tmp);
    }
}
/* resample code ---------------------------------------------------------------
* resample code
* args   : char   *code     I   code
*          int    len       I   code length (len < 2^(31-FPBIT))
*          double coff      I   initial code offset (chip)
*          int    smax      I   maximum correlator space (sample) 
*          double ci        I   code sampling interval (chip)
*          int    n         I   number of samples
*          short  *rcode    O   resampling code
* return : double               code remainder
*-----------------------------------------------------------------------------*/
extern double rescode(const short *code, int len, double coff, int smax, 
                      double ci, int n, short *rcode)
{
    short *p;

    coff-=smax*ci;
    coff-=floor(coff/len)*len; /* 0<=coff<len */

    for (p=rcode;p<rcode+n+2*smax;p++,coff+=ci) {
        if (coff>=len) coff-=len;
        *p=code[(int)coff];
    }
    return coff-smax*ci;
}
/* mix local carrier -----------------------------------------------------------
* mix local carrier to data
* args   : char   *data     I   data
*          int    dtype     I   data type (0:real,1:complex)
*          double ti        I   sampling interval (s)
*          int    n         I   number of samples
*          double freq      I   carrier frequency (Hz)
*          double phi0      I   initial phase (rad)
*          short  *I,*Q     O   carrier mixed data I, Q component
* return : double               phase remainder
*-----------------------------------------------------------------------------*/
extern double mixcarr(const char *data, int dtype, double ti, int n, 
                      double freq, double phi0, short *II, short *QQ)
{
    const char *p;
    double phi,ps,prem;

    static short cost[CDIV]={0},sint[CDIV]={0};
    int i,index;

    /* initialize local carrier table */
    if (!cost[0]) { 
        for (i=0;i<CDIV;i++) {
            cost[i]=(short)floor((cos(DPI/CDIV*i)/CSCALE+0.5));
            sint[i]=(short)floor((sin(DPI/CDIV*i)/CSCALE+0.5));
        }
    }
    phi=phi0*CDIV/DPI;
    ps=freq*CDIV*ti; /* phase step */

    if (dtype==DTYPEIQ) { /* complex */
        for (p=data;p<data+n*2;p+=2,II++,QQ++,phi+=ps) {
            index=((int)phi)&CMASK;
            *II=cost[index]*p[0]-sint[index]*p[1];
            *QQ=sint[index]*p[0]+cost[index]*p[1];
        }
    }
    if (dtype==DTYPEI) { /* real */
        for (p=data;p<data+n;p++,II++,QQ++,phi+=ps) {
            index=((int)phi)&CMASK;
            *II=cost[index]*p[0];
            *QQ=sint[index]*p[0];
        }
    }
    prem=phi*DPI/CDIV;
    while(prem>DPI) prem-=DPI;
    return prem;
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
    short *dataI=NULL,*dataQ=NULL,*code_e=NULL,*code;
    int i;
    int smax=s[ns-1];

    /* 8 is treatment of remainder in SSE2 */
    if (!(dataI=(short *)sdrmalloc(sizeof(short)*(n+64)))|| 
        !(dataQ=(short *)sdrmalloc(sizeof(short)*(n+64)))|| 
        !(code_e=(short *)sdrmalloc(sizeof(short)*(n+2*smax)))) {
            SDRPRINTF("error: correlator memory allocation\n");
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
    sdrfree(dataI); sdrfree(dataQ); sdrfree(code_e);
    dataI=dataQ=code_e=NULL;
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
extern void pcorrelator(const char *data, int dtype, double ti, int n,
                        double *freq, int nfreq, double crate, int m,
                        cpx_t* codex, double *P)
{
    int i;
    cpx_t *datax;
    short *dataI,*dataQ;
    char *dataR;

    if (!(dataR=(char  *)sdrmalloc(sizeof(char )*m*dtype))||
        !(dataI=(short *)sdrmalloc(sizeof(short)*(m+64)))||
        !(dataQ=(short *)sdrmalloc(sizeof(short)*(m+64)))||
        !(datax=cpxmalloc(m))) {
            SDRPRINTF("error: pcorrelator memory allocation\n");
            return;
    }

    /* zero padding */
    memset(dataR,0,m*dtype); /* zero paddinng */
    memcpy(dataR,data,2*n*dtype); /* for zero padding FFT */

    for (i=0;i<nfreq;i++) {
        /* mix local carrier */
        mixcarr(dataR,dtype,ti,m,freq[i],0.0,dataI,dataQ);

        /* to complex */
        cpxcpx(dataI,dataQ,CSCALE/m,m,datax);

        /* convolution */
        cpxconv(NULL,NULL,datax,codex,m,n,1,&P[i*n]);
    }
    sdrfree(dataR);
    sdrfree(dataI);
    sdrfree(dataQ);
    cpxfree(datax);
}

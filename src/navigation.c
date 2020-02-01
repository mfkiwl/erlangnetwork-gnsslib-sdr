/*------------------------------------------------------------------------------
* sdrnav.c : navigation data decode functions
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
*-----------------------------------------------------------------------------*/
#include "measurement_engine.h"

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

/* sdr navigation data function ------------------------------------------------
* decide navigation bit and decode navigation data
* args   : sdrch_t *sdr     I/O sdr channel struct
*          uint64_t buffloc I   buffer location
*          uint64_t cnt     I   counter of sdr channel thread
* return : none
*-----------------------------------------------------------------------------*/
extern void navigation(sdrch_t *sdr, uint64_t buffloc, uint64_t cnt)
{
    int sfn;

    sdr->nav.biti=cnt%sdr->nav.rate; /* current bit location for bit sync */
    sdr->nav.ocodei=(sdr->nav.biti-sdr->nav.synci-1); /* overlay code index */
    if (sdr->nav.ocodei<0) sdr->nav.ocodei+=sdr->nav.rate;

    /* navigation bit synchronization */
    /* if synchronization does not need (NH20 case) */
    if (sdr->nav.rate==1&&cnt>2000/(sdr->ctime*1000)) {
        sdr->nav.synci=0;
        sdr->nav.flagsync=ON;
    }
    /* check bit synchronization */
    if (!sdr->nav.flagsync&&cnt>2000/(sdr->ctime*1000))
        sdr->nav.flagsync=checksync(sdr->trk.II[0],sdr->trk.oldI[0],&sdr->nav);
    
    if (sdr->nav.flagsync) {
        /* navigation bit determination */
        if (checkbit(sdr->trk.II[0],sdr->trk.loopms,&sdr->nav)==OFF) {
            //debug_print("%s nav sync error!!\n",sdr->satstr);
        }

        /* check navigation frame synchronization */
        if (sdr->nav.swsync) {
            /* FEC (foward error correction) decoding */
            if (!sdr->nav.flagtow) predecodefec(&sdr->nav);

            /* frame synchronization (preamble search) */
            if (!sdr->nav.flagtow) sdr->nav.flagsyncf=findpreamble(&sdr->nav);

            /* preamble is found */
            if (sdr->nav.flagsyncf&&!sdr->nav.flagtow) {
                /* set reference sample data */
                sdr->nav.firstsf=buffloc;
                sdr->nav.firstsfcnt=cnt;
                debug_print("*** find preamble! %s %d %d ***\n",
                    sdr->satstr,(int)cnt,sdr->nav.polarity);
                sdr->nav.flagtow=ON;
            }
        }
        /* decoding navigation data */
        if (sdr->nav.flagtow&&sdr->nav.swsync) {
            /* if frame bits are stored */
            if ((int)(cnt-sdr->nav.firstsfcnt)%sdr->nav.update==0) {
                predecodefec(&sdr->nav); /* FEC decoding */
                sfn=decodenav(&sdr->nav); /* navigation message decoding */
                
                debug_print("%s ID=%d tow:%.1f week=%d cnt=%d\n",
                    sdr->satstr,sfn,sdr->nav.sdreph.tow_gpst,
                    sdr->nav.sdreph.week_gpst,(int)cnt);

                /* set reference tow data */
                if (sdr->nav.sdreph.tow_gpst==0) {
                    /* reset if tow does not decoded */
                    sdr->nav.flagsyncf=OFF;
                    sdr->nav.flagtow=OFF;
                } else if (cnt-sdr->nav.firstsfcnt==0) {
                    sdr->nav.flagdec=ON;
                    sdr->nav.sdreph.eph.sat=sdr->sat; /* satellite number */
                    sdr->nav.firstsftow=sdr->nav.sdreph.tow_gpst; /* tow */

                    if (sdr->nav.ctype==CTYPE_G1)
                        sdr->prn=sdr->nav.sdreph.prn;
                }
            }
        }
    }
}
/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data (two components case)
* args   : uint8_t *buff    I   byte data
*          int    p1        I   first bit start position (bits)
*          int    l1        I   first bit length (bits)
*          int    p2        I   second bit start position (bits)
*          int    l2        I   seconf bit length (bits)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
extern uint32_t getbitu2(const uint8_t *buff, int p1, int l1, int p2, int l2)
{
    return (getbitu(buff,p1,l1)<<l2)+getbitu(buff,p2,l2);
}
extern int32_t getbits2(const uint8_t *buff, int p1, int l1, int p2, int l2)
{
    if (getbitu(buff,p1,1))
        return (int32_t)((getbits(buff,p1,l1)<<l2)+getbitu(buff,p2,l2));
    else
        return (int32_t)getbitu2(buff,p1,l1,p2,l2);
}
/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data (three components case)
* args   : uint8_t *buff    I   byte data
*          int    p1        I   first bit start position (bits)
*          int    l1        I   first bit length (bits)
*          int    p2        I   second bit start position (bits)
*          int    l2        I   seconf bit length (bits)
*          int    p3        I   third bit start position (bits)
*          int    l3        I   third bit length (bits)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
extern uint32_t getbitu3(const uint8_t *buff, int p1, int l1, int p2, int l2, 
                         int p3, int l3)
{
    return (getbitu(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+
               getbitu(buff,p3,l3);
}
extern int32_t getbits3(const uint8_t *buff, int p1, int l1, int p2, int l2,
                        int p3, int l3)
{
    if (getbitu(buff,p1,1))
        return (int32_t)((getbits(buff,p1,l1)<<(l2+l3))+
                   (getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3));
    else
        return (int32_t)getbitu3(buff,p1,l1,p2,l2,p3,l3);
}
/* merge two variables ---------------------------------------------------------
* merge two unsigned/signed variables
* args   : u/int32_t a      I   first variable (MSB data)
*          uint32_t  b      I   second variable (LSB data)
* return : merged unsigned/signed data
*-----------------------------------------------------------------------------*/
extern uint32_t merge_two_u(const uint32_t a, const uint32_t b, int n)
{
    return (a<<n)+b;
}
extern int32_t merge_two_s(const int32_t a, const uint32_t b, int n)
{
    return (int32_t)((a<<n)+b);
}
/* convert binary bits to byte data --------------------------------------------
* pack binary -1/1 bits to uint8_t array
* args   : int    *bits     I   binary bits (1 or -1)
*          int    nbits     I   number of bits (nbits<MAXBITS)
*          int    nbin      I   number of byte data
*          int    right     I   flag of right-align bits (if nbits<8*nbin)
*          uint8_t *bin     O   converted byte data
* return : none
*-----------------------------------------------------------------------------*/
extern void bits2byte(int *bits, int nbits, int nbin, int right, uint8_t *bin)
{
    int i,j,rem,bitscpy[MAXBITS]={0};
    unsigned char b;
    rem=8*nbin-nbits;

    memcpy(&bitscpy[right?rem:0],bits,sizeof(int)*nbits);

    for (i=0;i<nbin;i++) {
        b=0;
        for (j=0;j<8;j++) {
            b<<=1;
            if (bitscpy[i*8+j]<0) b|=0x01; /* -1=>1, 1=>0 */
        }
        bin[i]=b;
    }
}
/* block interleave ------------------------------------------------------------
* block interleave of input vector
* args   : int    *in       I   input bits
*          int    row       I   rows (where data is read)
*          int    col       I   columns (where data is written)
*          int    *out      O   interleaved bits
* return : none
* note : row*col<MAXBITS
*-----------------------------------------------------------------------------*/
extern void interleave(const int *in, int row, int col, int *out)
{
    int r,c;
    int tmp[MAXBITS];
    memcpy(tmp,in,sizeof(int)*row*col);
    for (r=0;r<row;r++) {
        for(c=0;c<col;c++) {
            out[r*col+c]=tmp[c*row+r];
        }
    }
}
/* navigation bit synchronization ----------------------------------------------
* check synchronization of navigation bit
* args   : double IP        I   correlation output (IP data)
*          double oldIP     I   previous correlation output
*          navigation_t *nav    I/O navigation struct
* return : int                  1:synchronization 0: not synchronization
*-----------------------------------------------------------------------------*/
extern int checksync(double IP, double IPold, navigation_t *nav)
{
    int i,corr=0,maxi;
    if (IPold*IP<0) {
        nav->bitsync[nav->biti]+=1; /* voting bit position */
        /* check vote count */
        maxi=maxvi(nav->bitsync,nav->rate,-1,-1,&nav->synci);
        
        /* if synchronization success */
        if (maxi>NAVSYNCTH) {
            /* synchronization bit index */
            nav->synci--; /* minus 1 index*/
            if (nav->synci<0) nav->synci=nav->rate-1;
            return 1;
        }
    }

    return 0;
}
/* navigation data bit decision ------------------------------------------------
* navigation data bit is determined using accumulated IP data
* args   : double IP        I   correlation output (IP data)
*          int    loopms    I   interval of loop filter (ms) 
*          navigation_t *nav    I/O navigation struct
* return : int                  synchronization status 1:sync 0: not sync
*-----------------------------------------------------------------------------*/
extern int checkbit(double IP, int loopms, navigation_t *nav)
{
    int diffi=nav->biti-nav->synci,syncflag=ON,polarity=1;

    nav->swreset=OFF;
    nav->swsync=OFF;

    /* if synchronization is started */
    if (diffi==1||diffi==-nav->rate+1) {
        nav->bitIP=IP; /* reset */
        nav->swreset=ON;
        nav->cnt=1;
    } 
    /* after synchronization */
    else {
        nav->bitIP+=IP; /* cumsum */
        if (nav->bitIP*IP<0) syncflag=OFF;
    }

    /* genetaing loop filter timing */
    if (nav->cnt%loopms==0) nav->swloop=ON;
    else nav->swloop=OFF;

    /* if synchronization is finished */
    if (diffi==0) {
        if (nav->flagpol) {
            polarity=-1;
        } else {
            polarity=1;
        }
        nav->bit=(nav->bitIP<0)?-polarity:polarity;

        /* set bit*/
        shiftdata(&nav->fbits[0],&nav->fbits[1],sizeof(int),
            nav->flen+nav->addflen-1); /* shift to left */
        nav->fbits[nav->flen+nav->addflen-1]=nav->bit; /* add last */
        nav->swsync=ON;
    }
    nav->cnt++;

    return syncflag;
}
/* decode foward error correction ----------------------------------------------
* pre-decode foward error correction (before preamble detection)
* args   : navigation_t *nav    I/O navigation struct
* return : none
*-----------------------------------------------------------------------------*/
extern void predecodefec(navigation_t *nav)
{
    int i,j;
    unsigned char enc[NAVFLEN_SBAS+NAVADDFLEN_SBAS];
    unsigned char dec[94];
    int dec2[NAVFLEN_SBAS/2];

    /* GPS L1CA / GLONASS G1 */
    if (nav->ctype==CTYPE_L1CA ||
        nav->ctype==CTYPE_G1 ) {
        /* FEC is not used before preamble detection */
        memcpy(nav->fbitsdec,nav->fbits,sizeof(int)*(nav->flen+nav->addflen));
    }
    /* SBAS L1 */
    if( nav->ctype==CTYPE_L1SBAS ) {
        /* 1/2 convolutional code */
        init_viterbi27_port(nav->fec,0);
        for (i=0;i<NAVFLEN_SBAS+NAVADDFLEN_SBAS;i++)
            enc[i]=(nav->fbits[i]==1)? 0:255;
        update_viterbi27_blk_port(nav->fec,enc,(nav->flen+nav->addflen)/2);
        chainback_viterbi27_port(nav->fec,dec,nav->flen/2,0);
        for (i=0;i<94;i++) {
            for (j=0;j<8;j++) {
                dec2[8*i+j]=((dec[i]<<j)&0x80)>>7;
                nav->fbitsdec[8*i+j]=(dec2[8*i+j]==0)?1:-1;
                if (8*i+j==NAVFLEN_SBAS/2-1) {
                    break;
                }
            }
        }
    }
}
/* parity check ----------------------------------------------------------------
* parity check of navigation frame data
* args   : navigation_t *nav    I/O navigation struct
* return : int                  1:okay 0: wrong parity
*-----------------------------------------------------------------------------*/
extern int paritycheck(navigation_t *nav)
{
    int i,j,stat=0,crc,bits[MAXBITS];
    unsigned char bin[29]={0},pbin[3];

    /* copy */
    for (i=0;i<nav->flen+nav->addflen;i++) 
        bits[i]=nav->polarity*nav->fbitsdec[i];

    /* GPS L1CA parity check */
    if (nav->ctype==CTYPE_L1CA) {
        /* chacking all words */
        for (i=0;i<10;i++) {
            /* bit inversion */
            if (bits[i*30+1]==-1) {
                for (j=2;j<26;j++) 
                    bits[i*30+j]*=-1;
            }
            stat+=paritycheck_l1ca(&bits[i*30]);
        }
        /* all parities are correct */
        if (stat==10) {
            return 1;
        }
    }
    /* SBAS L1 parity check */
    if( nav->ctype==CTYPE_L1SBAS) {
        bits2byte(&bits[0],226,29,1,bin);
        bits2byte(&bits[226],24,3,0,pbin);

        /* compute CRC24 */
        crc=rtk_crc24q(bin,29);
        if (crc==getbitu(pbin,0,24)) {
            return 1;
        }
    }
    /* Galileo E1B / GLONASS G1 */
    if (nav->ctype==CTYPE_G1 ) {
        return 1;
    }

    return 0;
}
/* find preamble bits ----------------------------------------------------------
* search preamble bits from navigation data bits
* args   : navigation_t *nav    I/O navigation struct
* return : int                  1:found 0: not found
*-----------------------------------------------------------------------------*/
extern int findpreamble(navigation_t *nav)
{
    int i,corr=0;

    /* GPS L1CA */
    if (nav->ctype==CTYPE_L1CA ) {
        for (i=0;i<nav->prelen;i++)
            corr+=(nav->fbitsdec[nav->addflen+i]*nav->prebits[i]);
    }
    /* L1-SBAS */
    /* check 2 preambles */
    if( nav->ctype==CTYPE_L1SBAS ) {
        for (i=0;i<nav->prelen/2;i++) {
            corr+=(nav->fbitsdec[i    ]*nav->prebits[ 0+i]);
            corr+=(nav->fbitsdec[i+250]*nav->prebits[ 8+i]);
        }
    }
    /*  GLONASS G1 */
    /* time mark is last in word */
    if (nav->ctype==CTYPE_G1) {
        for (i=0;i<nav->prelen;i++)
            corr+=(nav->fbitsdec[nav->flen-nav->prelen+i]*nav->prebits[i]);
    }
    /* check preamble match */
    if (abs(corr)==nav->prelen) { /* preamble matched */
        nav->polarity=corr>0?1:-1; /* set bit polarity */
        /* parity check */
        if (paritycheck(nav)) {
            return 1;
        } else {
            if( nav->ctype==CTYPE_L1SBAS ) {
                if (nav->polarity==1) nav->flagpol=ON;
            }
        }
    }

    return 0;
}
/* decode navigation data ------------------------------------------------------
* decode navigation frame
* args   : navigation_t *nav    I/O navigation struct
* return : int                  0:decode sucsess 0: decode error
*-----------------------------------------------------------------------------*/
extern int decodenav(navigation_t *nav)
{
    switch (nav->ctype) {
        /* GPS L1CA (LNAV) */
        case CTYPE_L1CA:
            return decode_l1ca(nav);
        /* SBAS L1 */
        case CTYPE_L1SBAS:
            return decode_l1sbas(nav);
        /* GLONASS G1 */
        case CTYPE_G1:
            return decode_g1(nav);
        default:
            return -1;
    }
}

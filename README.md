Erlang Network GNSS-SDRLIB v4.0 (Community Edition)
===============================================================================
An Open Source GNSS Software Defined Radio Library Provided by Erlang Network

System Requirements
-------------------------------------------------------------------------------
* Erlang Network GNSS-SDRLIB works on both macOS and various Debian based Linux distributions.

Authors
-------------------------------------------------------------------------------
Taro Suzuki  (GNSS-SDRLIB)
E-Mail: <gnsssdrlib@gmail.com>
HP: <http://www.taroz.net>

Shu Wang  (Erlang Network)
Email: shu@erlangnetwork.com
Blog: https://blog.erlangnetwork.com
Wechat: run4life2

Features
-------------------------------------------------------------------------------
* GNSS signal processing functions written in C
    * PRN code generations
    * Signal acquisition
    * Signal tracking
    * Decoding navigation messages 
    * Pseudo-range & Doppler Measurement 
    * Carrier phase mesurements 
* Support hardware acceleration through SIMD instructions, such as SSE2 and AVX2
* Support hetergenous computing acceleration through GPGPUs
* Real-time positioning with RTKLIB (<http://www.rtklib.com/>)
* Observation data can be outputted in RINEX or RTCM format
* Support following signals (tracking and decoding navigation message) 
    * GPS L1CA
    * GLONASS G1
    * Galileo E1B
    * BeiDou B1I
    * QZSS L1CA/SAIF/LEX
    * SBAS L1
* Support following front-ends for real-time positioning
    * NSL Stereo <http://www.nsl.eu.com/primo.html>
    * SiGe GN3S sampler v2/v3 <https://www.sparkfun.com/products/10981>
    * Nuand BladeRF <http://nuand.com/>
    * RTL-SDR <http://sdr.osmocom.org/trac/wiki/rtl-sdr>
* Support RF binary file for post processing

Directory and Files
-------------------------------------------------------------------------------
    ./bin                   Executable APs for Windows  
        ./erlang-gnss       Real-time Erlang Network GNSS signal processing executable 
        ./gnss-sdrcli.ini   Configuration file for Erlang Network GNSS Applications
        ./gnss_L1CA_12.ini  Configuration file for Erlang Network GNSS Applications
        ./Makefile          Makefile
    ./config                Directory of front-end configuration files  
    ./src                   Library source codes  
        ./sdr.h             Library header file  
        ./acquisition.c     Functions related to signal acquisition  
        ./sdrcmn.c          Functions related to SIMD operation  
        ./sdrcode.c         Functions related to generation of ranging code  
        ./sdrinit.c         Functions related to initialization/end process  
        ./sdrlex.c          Functions related to QZSS LEX decoding  
        ./main.c            Main function  
        ./sdrthread.c
        ./sdrnav.c          Functions related to navigation data  
        ./sdrnav_gps.c      Functions related to decoding GPS nav. data  
        ./sdrnav_glo.c      Functions related to decoding GLONASS nav. data  
        ./sdrnav_gal.c      Functions related to decoding Galileo nav. data  
        ./sdrnav_bds.c      Functions related to decoding BeiDou nav. data  
        ./sdrnav_sbs.c      Functions related to decoding SBAS nav. data  
        ./sdrout.c          Functions related to RINEX/RTCM outputs  
        ./sdrplot.c         Functions related to plot graph  
        ./sdrrcv.c          Functions related to receiving RF data  
        ./sdrspec.c         Functions related to spectrum analysis  
        ./sdrsync.c         Functions related to generating observation data  
        ./sdrtrk.c          Functions related to signal tracking  
        ./rcv               Source codes related to front-end  
            ./rtl-sdr       Source codes from Osmocom Opensource Project
    ./lib                   Source codes related to used library  
        ./RTKLIB            Source codes of Tomoji Takasu's open source GNSS positionining engine 
        ./k9aq-fec          Source codes of Phil Karn, KA9Q's open source FEC library
        ./fftw3
    ./test                  Test data  
        ./data              Test IF data  
        ./output            Default RINEX output directory  


License
-------------------------------------------------------------------------------
Private Uses are permitted but commercial uses shall be licensed. See LICENSE file.  

Generally speaking, a dual-license is applied here.  Creative Common License is applied in the first priority.  If a certain clause of the creative common license is inapplicable due to related law and regulations, the corresponding clause from GPL 3.0 license will be applied.  

Erlang Network GNSSLib v4.0 (Community Edition)
===============================================================================
An Open Source GNSS Measurement Engine Software Defined Radio Library Provided by Erlang Network

System Requirements
-------------------------------------------------------------------------------
* Erlang Network GNSSLib works on both macOS and various Debian based Linux distributions.

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
* Real-time positioning with RTKLIB (<http://www.rtklib.com/>)
* Observation data can be outputted in RINEX or RTCM format
* Support following signals (tracking and decoding navigation message) 
    * GPS L1CA
    * GLONASS G1
    * Galileo E1B
    * BeiDou B1I
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
        ./gnss_L2CM.ini  Configuration file for Erlang Network GNSS Applications
        ./Makefile          Makefile
    ./config                Directory of front-end configuration files  
    ./src                   Library source codes  
        ./measurement_engine.h   GNSS measurement engine library header file  
        ./acquisition.c     Functions related to signal acquisition
        ./acquisition_initialization.c
        ./tracking.c        Functions related to signal tracking
        ./tracking_initialization.c
        ./sdrcmn.c          Functions related to SIMD operation  
        ./sdrcode.c         Functions related to generation of ranging code  
        ./sdrinit.c         Functions related to initialization/end process  
        ./main.c            Main function  
        ./statemachine.c
        ./navigation.c      Functions related to navigation data  
        ./navigation_gps.c  Functions related to decoding GPS nav. data  
        ./navigation_glo.c  Functions related to decoding GLONASS nav. data  
        ./navigation_gal.c  Functions related to decoding Galileo nav. data  
        ./navigation_bds.c  Functions related to decoding BeiDou nav. data  
        ./navigation_sbs.c  Functions related to decoding SBAS nav. data  
        ./sdrout.c          Functions related to RINEX/RTCM outputs  
        ./sdrplot.c         Functions related to plot graph  
        ./sdrrcv.c          Functions related to receiving RF data  
        ./sdrspec.c         Functions related to spectrum analysis  
        ./sdrsync.c         Functions related to generating observation data  
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

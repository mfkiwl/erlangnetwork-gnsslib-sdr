Erlang Network GNSSLib v3.0 (Community Edition)
===============================================================================
An Open Source GNSS Software Defined Radio Library Provided by Erlang Network

System Requirements
-------------------------------------------------------------------------------
* Erlang Network GNSSLib works on various Debian based Linux distributions.
* The software package dependancy includes fftw, rtl-sdr, libusb, ka9q-fec, RTKLIB.

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
        ./erlang-gnss       Real-time GNSS signal processing AP (CLI)  
        ./gnss-sdrcli.ini   Configuration file for CLI AP  
        ./Makefile          Makefile
    ./frontend              Directory of front-end configuration files  
    ./src                   Library source codes  
        ./sdr.h             Library header file  
        ./sdracq.c          Functions related to signal acquisition  
        ./sdrcmn.c          Functions related to common and shared operations  
        ./sdrcode.c         Functions related to generation of ranging code  
        ./sdrinit.c         Functions related to initialization/end process  
        ./sdrmain.c         Main function  
        ./sdrnav.c          Functions related to navigation data  
        ./sdrnav_gps.c      Functions related to decoding GPS nav. data  
        ./sdrnav_glo.c      Functions related to decoding GLONASS nav. data  
        ./sdrnav_sbs.c      Functions related to decoding SBAS nav. data  
        ./sdrout.c          Functions related to RINEX/RTCM outputs  
        ./sdrplot.c         Functions related to plot graph  
        ./sdrrcv.c          Functions related to receiving RF data  
        ./sdrspec.c         Functions related to spectrum analysis  
        ./sdrsync.c         Functions related to generating observation data  
        ./sdrtrk.c          Functions related to signal tracking  
        ./rcv               Source codes related to front-end  
    ./lib                   Source codes related to used library  
        ./RTKLIB            Source codes of Tomoji Takasu's open source GNSS positionining engine 
        ./k9aq-fec          Source codes of Phil Karn, KA9Q's open source FEC library
        ./fftw3
    ./test                  Test data  
        ./data              Test IF data  
        ./output            Default RINEX output directory
    ./output


License
-------------------------------------------------------------------------------
Copyright (C) 2020 Shu Wang shuwang1@outlook.com 

Copyright (C) 2014 Taro Suzuki gnsssdrlib@gmail.com

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

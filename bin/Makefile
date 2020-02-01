# \filename Makefile
# \brief Makefile for Erlang Network GNSS SDRLib
#
# \author Shu Wang, shuwang1@outlook.com


# Use or not use RTLSDR and BLADERF
# 1:Use 0:Not Use
USE_RTLSDR=1
USE_BLADERF=0

SRC=../src
RTKLIB=../lib/RTKLIB
FEC=../lib/ka9q-fec
STEREO=../src/rcv/stereo
RTL-SDR=../src/rcv/rtl-sdr
RTLSDR=../src/rcv/rtlsdr
BLADERF=../src/rcv/bladerf

INCLUDE=-I$(SRC) -I$(RTKLIB)/src -I$(FEC)

OPTIONS = -DFFTMTX -DDEBUG_ENABLED

OBS= main.o statemachine.o sdrcmn.o \
	 sdrcode.o sdrinit.o \
     navigation_gps.o navigation_glo.o navigation_sbs.o\
     sdrout.o sdrplot.o sdrrcv.o sdrspec.o \
     sdrsync.o rtkcmn.o rtcm.o rtcm2.o rtcm3.o rtcm3e.o rinex.o\
	 acquisition_initialization.o acquisition.o\
	 tracking_initialization.o tracking.o navigation.o

ifeq ($(USE_STEREO),1)
STEREO=../hardware/stereo
OPTIONS+= -DSTEREO -DSTEREOV26
INCLUDE+= -I$(STEREO)
#LIBS+=$(STEREO)/linux/libnslstereo.a
OBS+=stereo.o
endif

ifeq ($(USE_RTLSDR),1)
#RTLSDR=../src/rcv/rtlsdr
LIBS = -lrtlsdr
OPTIONS += -DRTLSDR -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON
INCLUDE += -I$(RTLSDR) -I../src/rcv/rtl-sdr/include
OBS += librtlsdr.o rtlsdr.o convenience.o tuner_e4k.o tuner_fc0012.o tuner_fc0013.o tuner_fc2580.o tuner_r82xx.o
endif

ifeq ($(USE_BLADERF),1)
OPTIONS+=-DBLADERF
LIBS+=-lbladeRF
INCLUDE += -I$(BLADERF)
OBS+=bladerf.o
endif

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CC = gcc
	CFLAGS = -O3 -march=native -mtune=native $(INCLUDE) $(OPTIONS)
	LDLIBS = -lm -lrt  -lfftw3f_threads -lfftw3_threads -lfftw3f -lfftw3 -lpthread $(LIBS) -lusb-1.0 -lfec 
	BIN = erlang-gnss
endif

ifeq ($(UNAME_S),Darwin)
	CC = gcc
	CFLAGS = -O3 -march=native -mtune=native $(INCLUDE) $(OPTIONS)
	LDLIBS = -lm -lfftw3f_threads -lfftw3_threads -lfftw3f -lfftw3 -lpthread $(LIBS) -lusb-1.0 -lfec 
	BIN = erlang-gnss
endif

$(BIN): $(OBS)
	 $(CC) -o $(BIN) $(OBS) $(CFLAGS) $(LDLIBS)

main.o: $(SRC)/main.c
	$(CC) -c $(CFLAGS) $(SRC)/main.c
statemachine.o : $(SRC)/statemachine.c
	$(CC) -c $(CFLAGS) $(SRC)/statemachine.c
sdrcmn.o : $(SRC)/sdrcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrcmn.c
acquisition_initialization.o : $(SRC)/acquisition_initialization.c
	$(CC) -c $(CFLAGS) $(SRC)/acquisition_initialization.c
acquisition.o : $(SRC)/acquisition.c
	$(CC) -c $(CFLAGS) $(SRC)/acquisition.c
tracking_initialization.o : $(SRC)/tracking_initialization.c
	$(CC) -c $(CFLAGS) $(SRC)/tracking_initialization.c
sdrcode.o: $(SRC)/sdrcode.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrcode.c
sdrinit.o: $(SRC)/sdrinit.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrinit.c
sdrout.o : $(SRC)/sdrout.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrout.c
navigation.o : $(SRC)/navigation.c
	$(CC) -c $(CFLAGS) $(SRC)/navigation.c
navigation_gps.o : $(SRC)/navigation_gps.c
	$(CC) -c $(CFLAGS) $(SRC)/navigation_gps.c
navigation_glo.o : $(SRC)/navigation_glo.c
	$(CC) -c $(CFLAGS) $(SRC)/navigation_glo.c
navigation_sbs.o : $(SRC)/navigation_sbs.c
	$(CC) -c $(CFLAGS) $(SRC)/navigation_sbs.c
sdrplot.o: $(SRC)/sdrplot.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrplot.c
sdrrcv.o : $(SRC)/sdrrcv.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrrcv.c
sdrspec.o: $(SRC)/sdrspec.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrspec.c
tracking.o : $(SRC)/tracking.c
	$(CC) -c $(CFLAGS) $(SRC)/tracking.c
sdrsync.o : $(SRC)/sdrsync.c
	$(CC) -c $(CFLAGS) $(SRC)/sdrsync.c
rtkcmn.o   : $(RTKLIB)/src/rtkcmn.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rtkcmn.c
rtcm.o     : $(RTKLIB)/src/rtcm.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rtcm.c
rtcm2.o    : $(RTKLIB)/src/rtcm2.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rtcm2.c
rtcm3.o    : $(RTKLIB)/src/rtcm3.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rtcm3.c
rtcm3e.o   : $(RTKLIB)/src/rtcm3e.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rtcm3e.c
rinex.o    : $(RTKLIB)/src/rinex.c
	$(CC) -c $(CFLAGS) $(RTKLIB)/src/rinex.c
stereo.o : $(STEREO)/stereo.c
	$(CC) -c $(CFLAGS) $(STEREO)/stereo.c

tuner_e4k.o: $(RTL-SDR)/src/tuner_e4k.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/tuner_e4k.c
tuner_fc0012.o: $(RTL-SDR)/src/tuner_fc0012.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/tuner_fc0012.c
tuner_fc0013.o: $(RTL-SDR)/src/tuner_fc0013.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/tuner_fc0013.c
tuner_fc2580.o: $(RTL-SDR)/src/tuner_fc2580.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/tuner_fc2580.c
tuner_r82xx.o: $(RTL-SDR)/src/tuner_r82xx.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/tuner_r82xx.c
librtlsdr.o : $(RTL-SDR)/src/librtlsdr.c
	$(CC) -c $(CFLAGS) $(RTL-SDR)/src/librtlsdr.c	
rtlsdr.o : $(RTLSDR)/rtlsdr.c
	$(CC) -c $(CFLAGS) $(RTLSDR)/rtlsdr.c
convenience.o : $(RTLSDR)/convenience.c
	$(CC) -c $(CFLAGS) $(RTLSDR)/convenience.c
bladerf.o : $(BLADERF)/bladerf.c
	$(CC) -c $(CFLAGS) $(BLADERF)/bladerf.c

main.o: $(SRC)/measurement_engine.h
statemachine.o: $(SRC)/measurement_engine.h
sdrcmn.o : $(SRC)/measurement_engine.h
acquisition_initialization.o : $(SRC)/measurement_engine.h
acquisition.o : $(SRC)/measurement_engine.h
tracking_initialization.o : $(SRC)/measurement_engine.h
tracking.o : $(SRC)/measurement_engine.h
sdrcode.o: $(SRC)/measurement_engine.h
sdrinit.o: $(SRC)/measurement_engine.h
sdrout.o : $(SRC)/measurement_engine.h
navigation.o : $(SRC)/measurement_engine.h
navigation_gps.o : $(SRC)/measurement_engine.h
navigation_glo.o : $(SRC)/measurement_engine.h
navigation_sbs.o : $(SRC)/measurement_engine.h
sdrplot.o: $(SRC)/measurement_engine.h
sdrrcv.o : $(SRC)/measurement_engine.h
sdrspec.o: $(SRC)/measurement_engine.h
sdrsync.o: $(SRC)/measurement_engine.h
rtkcmn.o : $(SRC)/measurement_engine.h
rtcm.o: $(SRC)/measurement_engine.h
rtcm2.o : $(SRC)/measurement_engine.h
rtcm3.o : $(SRC)/measurement_engine.h
rtcm3e.o: $(SRC)/measurement_engine.h
rinex.o : $(SRC)/measurement_engine.h
stereo.o : $(SRC)/measurement_engine.h
rtlsdr.o : $(SRC)/measurement_engine.h
convenience.o : $(SRC)/measurement_engine.h
bladerf.o : $(SRC)/measurement_engine.h

clean:
	rm -f *.o $(BIN)
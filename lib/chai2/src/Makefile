#
# Makefile for chai
#
###################################################################

ifneq ($(KERNELRELEASE),)
# kbuild part

ccflags-y := $(DRVDEBUG) -DLINUX -D_KERNEL_ -I$(src) -I$(src)/../include 

obj-m = unican.o
unican-y := sja1000.o statstr.o unicandrv.o unicandrv-lnx.o \
 boards.o boards-conf.o bmarathon.o sysdep.o

else


PROGS   = canmon  libchai chaitest
INCLUDES := -I$(CHAIBASEDIR)/src -I$(CHAIBASEDIR)/include
EXTRA_CFLAGS += -Wall -DLINUX $(DRVDEBUG) $(INCLUDES)

#EXTRA_CFLAGS += $(DRVDEBUG) -D_KERNEL_ -DMODULE -D__KERNEL__ -DLINUX  $(INCLUDES)
#MOD_CFLAGS += $(DRVDEBUG) -D_KERNEL_ -DMODULE -D__KERNEL__ -DLINUX  $(INCLUDES) -I$(KERNEL_INC)

all: $(PROGS)


libchai:
	$(CC) -fPIC $(EXTRA_CFLAGS) -o statstr.o -c statstr.c
	$(CC) -fPIC $(EXTRA_CFLAGS) -o chai.o -c chai-lnx.c
	$(CC) -shared -o ../lib/libchai.so statstr.o chai.o
	rm -f ../lib/libchai.so.$(CHAIVER)
	ln -s libchai.so ../lib/libchai.so.$(CHAIVER)

canmon: canmon.c libchai
	$(CC) $(EXTRA_CFLAGS) $(READLN_SUPPORT) -L../lib -o ../bin/canmon canmon.c $(READLN_LIBS) -lchai -lpthread

chaitest: chaitest.c
	$(CC) $(EXTRA_CFLAGS) -L../lib -o ../bin/chaitest chaitest.c -lchai


clean:
	rm -rf .tmp_versions
	rm -f  *.o *.ko core *~ .*.o.cmd .*.ko.cmd Module.symvers modules.order unican.mod.c

cleanall:
	make clean
	rm -f  $(PROGS) unican.mod.c *.o core *~ ../lib/libchai.so ../lib/libchai.so.$(CHAIVER) ../lib/unican.o ../lib/unican.ko ../bin/canmon ../bin/chaitest

canmon:		canmon.c
chaitest:	chaitest.c

endif
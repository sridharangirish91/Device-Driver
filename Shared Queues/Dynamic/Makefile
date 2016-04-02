KDIR:=/opt/clanton-full/1.4.2/sysroots/i586-poky-linux/usr/src/kernel
PWD:= $(shell pwd)

CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SROOT=/opt/clanton-full/1.4.2/sysroots/i586-poky-linux/

APP = gmem_tester
obj-m:= Squeue.o

all:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(KDIR) M=$(PWD) modules 
	i586-poky-linux-gcc -o $(APP) main_1.c --sysroot=$(SROOT) -lpthread

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(APP) 

#KDIR:= /lib/modules/$(shell uname -r)/build/
#
#ARCH = x86
#
#obj-m:= Squeue.o
#
#all:
#	make -C $(KDIR) M=$(PWD) modules
#
#clean:
#	rm -f *.ko
#	rm -f *.o
#	rm -f Module.symvers
#	rm -f modules.order
#	rm -f *.mod.c
#	rm -f *.mod.c
#	rm -f *.mod.o
#	rm -f \.*.cmd
#	rm -f Module.markers
#	rm -f $(APP) 

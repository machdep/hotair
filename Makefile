APP = 		hotair
ARCH =		mips

CC =		${CROSS_COMPILE}gcc
LD =		${CROSS_COMPILE}ld
OBJCOPY =	${CROSS_COMPILE}objcopy

LDSCRIPT =	${.CURDIR}/ldscript

export CFLAGS = -march=mips32r2 -EL -nostdlib -nostdinc -O	\
	-fno-pic -mno-abicalls -mmicromips -fno-builtin-printf	\
	-msoft-float -D__mips_o32

export AFLAGS = ${CFLAGS}

all:	
	@python3 -B mdepx/tools/emitter.py -j mdepx.conf
	@${OBJCOPY} -O srec obj/hotair.elf obj/hotair.srec

clean:
	@rm -rf obj/*

include mdepx/mk/user.mk

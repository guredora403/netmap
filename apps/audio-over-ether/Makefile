#settings
CC=clang
LD=ld.lld
VERSION = 1.0RC1
SRCDIR ?= ../..
VSOUND_HEADER_DIR = $(SRCDIR)/../vsound
BUILDDIR = $(SRCDIR)
CLEANFILES = $(PROGS) *.o
PROGS	=	aoe aoeserver aoeserver-gen

CLI_FLAGS = -DCLI
CLI_FLAGS += -DLOG

SVR_FLAGS = -DSVR
SVR_FLAGS += -DSERVER_STATS
SVR_FLAGS += -DLOG

WARN_FLAGS = -Werror -Wall -Wunused-function
WARN_FLAGS += -Wextra
WARN_FLAGS += -Wshadow

CFLAGS  = -DAOE_VERSION=\"$(VERSION)\"
CFLAGS += -O3 -pipe -g
CFLAGS += -mcpu=cortex-a72 -march=armv8-a+crc -mtune=cortex-a72
CFLAGS += -fno-stack-protector
CFLAGS += -fno-semantic-interposition
CFLAGS += -fno-math-errno
CFLAGS += -I $(SRCDIR)/sys -I $(SRCDIR)/apps/include -I $(SRCDIR)/libnetmap
CFLAGS += -I $(VSOUND_HEADER_DIR)
LDLIBS += -lm -lrt -lasound
LDFLAGS = -L./
LDLIBS += -lgcov

# checksum
#CFLAGS += -DCSUM

# coverage
#CFLAGS += --coverage -O0
#CFLAGS += -I/usr/lib/gcc/aarch64-unknown-linux-gnu/10.1.0/include/
#LDFLAGS += -L/usr/lib/gcc/aarch64-unknown-linux-gnu/10.1.0/

# asm
#CFLAGS += -g -save-temps=obj

all: clean aoe
	size -G aoe
	strip aoe
	ls -al aoe
	md5sum aoe

clean:
	-@rm -rf $(CLEANFILES)

# aoeserver (PGO generate)
gen: CFLAGS += -fprofile-generate -fprofile-correction
gen: CFLAGS += -ffunction-sections -fdata-sections -fomit-frame-pointer -flto
gen: LDFLAGS += -Wl,-flto -Wl,--gc-sections -Wl,--no-as-needed
gen: clean aoeserver

# aoeserver (PGO use)
use: CFLAGS += -fprofile-use -fprofile-correction
use: CFLAGS += -ffunction-sections -fdata-sections -fomit-frame-pointer -flto
use: LDFLAGS += -Wl,-flto -Wl,--gc-sections -Wl,--no-as-needed
use: clean aoeserver
	size -G aoeserver
	strip aoeserver
	ls -al aoeserver
	md5sum aoeserver

# aoe
aoe: CFLAGS += -ffunction-sections -fdata-sections -fomit-frame-pointer -flto
aoe: LDFLAGS += -flto
aoe: aoe.o nmctx.o  nmctx-pthreads.o  nmport.o  nmreq.o
aoe.o: aoe.c
	$(CC) $(CFLAGS) $(WARN_FLAGS) $(CLI_FLAGS) -c $^ -o $@

# aoeserver
aoeserver: CFLAGS += -ffunction-sections -fdata-sections -fomit-frame-pointer -flto
aoeserver: LDFLAGS += -flto
aoeserver: aoeserver.o nmctx.o  nmctx-pthreads.o  nmport.o  nmreq.o
aoeserver.o: aoe.c
	$(CC) $(CFLAGS) $(WARN_FLAGS) $(SVR_FLAGS) -c $^ -o $@

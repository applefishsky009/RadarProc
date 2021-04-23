ifneq ($(NX_CROSS_COMPILE),)
NX_CROSS_COMPILE:=$(NX_CROSS_COMPILE)
endif
AR=$(NX_CROSS_COMPILE)ar
ifeq ($(CC),cc)
CC:=$(NX_CROSS_COMPILE)gcc
endif
LD:=$(NX_CROSS_COMPILE)ld

RADAR_PROC_TARGET = lib/lib_radarproc_1224_release.a

RADAR_PROC_INC = -Iinclude/
CFLAGS = $(RADAR_PROC_INC)
CFLAGS += -Wall -s -lpthread -ldl -lm

RADAR_PROC_SRC = $(wildcard src/*.c)
RADAR_PROC_OBJS = $(RADAR_PROC_SRC:.c=.o)

.PHONY:all
all: $(RADAR_PROC_OBJS)
	$(AR) rc $(RADAR_PROC_TARGET) $(RADAR_PROC_OBJS)

clean:
	rm -rf $(RADAR_PROC_TARGET) $(RADAR_PROC_OBJS)

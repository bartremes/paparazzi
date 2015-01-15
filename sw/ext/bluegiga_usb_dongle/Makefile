
TARGET = bluetooth-proximity

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)
DEPS := $(SRCS:.c=.d)

CROSS_COMPILE   ?= arm-linux-gnueabi-
CC              := $(CROSS_COMPILE)gcc
CFLAGS          := -O3 -Wall -Wextra -Wno-unused-parameter -Wmissing-prototypes
LDFLAGS         := -Wl,--no-undefined -Wl,--as-needed

all: $(TARGET)

%.o: %.c
	$(CC) -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"

$(TARGET): $(OBJS)
	@echo 'Building target: $@'
	$(CC) -o"$(TARGET).elf" $(OBJS) $(LDFLAGS)
	@echo 'Finished building target: $@'

clean:
	-$(RM) $(OBJS) $(DEPS) $(TARGET).elf

#include $(PAPARAZZI_SRC)/conf/Makefile.linux

DRONE = $(PAPARAZZI_SRC)/sw/tools/parrot/ardrone2.py

# Program the device and start it.
upload program: $(OBJDIR)/$(TARGET).elf
	#$(Q)$(DRONE) --host=$(HOST) upload_file_and_run $(OBJDIR)/$(TARGET).elf $(SUB_DIR)
	#$(Q)$(DRONE) --host=$(HOST) status

.PHONY: all clean

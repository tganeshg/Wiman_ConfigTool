#For Openwrt - HLLink
#Serial AT Command Config tool @ Febraury 2026#

#BASE_PATH = ${HOME}/openwrt2
# Cross compiler
#STAGING_DIR = {BASE_PATH}/openwrt/staging_dir


#export PATH="$HOME/openwrt2/openwrt/staging_dir/toolchain-mipsel_24kc_gcc-14.3.0_musl/bin:$PATH"
#export STAGING_DIR="$HOME/openwrt2/openwrt/staging_dir"

#export PATH := ${STAGING_DIR}/toolchain-mipsel_24kc_gcc-14.3.0_musl/bin:$(PATH)
#export STAGING_DIR := ${STAGING_DIR}

CC 		= mipsel-openwrt-linux-gcc
CFLAGS	= -Wall -Wno-unused-variable -Wunused-but-set-variable -Wpointer-sign
INCS 	= -I./include -I${STAGING_DIR}/toolchain-mipsel_24kc_gcc-14.3.0_musl/include -I${STAGING_DIR}/target-mipsel_24kc_musl/usr/include
LFLAGS  = -L. -L${STAGING_DIR}/toolchain-mipsel_24kc_gcc-14.3.0_musl/lib -L${STAGING_DIR}/target-mipsel_24kc_musl/usr/lib -lm -lpthread -lrt -lubox -luci

# change these to set the proper directories where each files should be
SRCDIR   = source
INCDIR	 = include
OBJDIR   = objects
BINDIR   = bin

SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(INCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)
rm       = rm -Rf

TARGET=Sample

$(BINDIR)/$(TARGET): $(OBJECTS)
	@$(CC) -o  $@ $(LFLAGS) $(OBJECTS)
	#cp $(TARGET) ../Firmware/bin
	@echo "Linking complete!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
	@$(CC) $(INCS) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

.PHONEY: clean
clean:
	@$(rm) $(OBJDIR)/*.o
	@$(rm) $(BINDIR)/$(TARGET)
	@echo $(OBJECTS)
	@echo $(TARGET)
	@echo "Cleanup complete!"

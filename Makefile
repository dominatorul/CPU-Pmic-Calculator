#---------------------------------------------------------------------------------
# Project Metadata
#---------------------------------------------------------------------------------
TARGET     := pmic-calculator-cpu
BUILD      := build
SOURCES    := source
DATA       := data
INCLUDES   := include

APP_TITLE  := PMIC CPU Calculator
APP_AUTHOR := Dominatorul
APP_VERSION:= 1.0.0

#---------------------------------------------------------------------------------
# Environment Setup
#---------------------------------------------------------------------------------
ifeq ($(strip $(DEVKITPRO)),)
$(error "Please set DEVKITPRO in your environment. export DEVKITPRO=<path to>/devkitpro")
endif

TOPDIR ?= $(CURDIR)
include $(DEVKITPRO)/libnx/switch_rules

#---------------------------------------------------------------------------------
# Build Options
#---------------------------------------------------------------------------------
ARCH     := -march=armv8-a+crc+crypto -mtune=cortex-a57 -mtp=soft -fPIE

CFLAGS   := -g -Wall -O2 -ffunction-sections $(ARCH) $(DEFINES)
CFLAGS   += $(INCLUDE) -D__SWITCH__ -DMATRIX_SIZE=64

CXXFLAGS := $(CFLAGS) -fno-rtti -fno-exceptions -std=c++17
ASFLAGS  := -g $(ARCH)
LDFLAGS  := -specs=$(DEVKITPRO)/libnx/switch.specs -g $(ARCH) -Wl,-Map,$(notdir $*.map)

# Library order matters: specific → general
LIBS     := -lnx

LIBDIRS  := $(PORTLIBS) $(LIBNX)

#---------------------------------------------------------------------------------
# Build Rules
#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))

export OUTPUT   := $(CURDIR)/$(BUILD)/$(TARGET)
export TOPDIR   := $(CURDIR)
export VPATH    := $(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) \
                   $(foreach dir,$(DATA),$(CURDIR)/$(dir))
export DEPSDIR  := $(CURDIR)/$(BUILD)

CFILES    := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES  := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))
SFILES    := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.s)))
BINFILES  := $(foreach dir,$(DATA),$(notdir $(wildcard $(dir)/*.*)))

# Use CXX if C++ sources exist, else CC
ifeq ($(strip $(CPPFILES)),)
  export LD := $(CC)
else
  export LD := $(CXX)
endif

export OFILES_BIN := $(addsuffix .o,$(BINFILES))
export OFILES_SRC := $(CPPFILES:.cpp=.o) $(CFILES:.c=.o) $(SFILES:.s=.o)
export OFILES     := $(OFILES_BIN) $(OFILES_SRC)
export HFILES_BIN := $(addsuffix .h,$(subst .,_,$(BINFILES)))

export INCLUDE    := $(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
                     $(foreach dir,$(LIBDIRS),-I$(dir)/include) \
                     -I$(CURDIR)/$(BUILD)
export LIBPATHS   := $(foreach dir,$(LIBDIRS),-L$(dir)/lib)

# Icon detection
ifeq ($(strip $(ICON)),)
  icons := $(wildcard *.jpg)
  ifneq (,$(findstring $(TARGET).jpg,$(icons)))
    export APP_ICON := $(TOPDIR)/$(TARGET).jpg
  else ifneq (,$(findstring icon.jpg,$(icons)))
    export APP_ICON := $(TOPDIR)/icon.jpg
  endif
else
  export APP_ICON := $(TOPDIR)/$(ICON)
endif

ifeq ($(strip $(NO_ICON)),)
  export NROFLAGS += --icon=$(APP_ICON)
endif
ifeq ($(strip $(NO_NACP)),)
  export NROFLAGS += --nacp=$(OUTPUT).nacp
endif
ifneq ($(APP_TITLEID),)
  export NACPFLAGS += --titleid=$(APP_TITLEID)
endif

#---------------------------------------------------------------------------------
.PHONY: all clean
#---------------------------------------------------------------------------------
all: clean $(BUILD)

$(BUILD):
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile
	@echo "Copying $(TARGET).nro to root directory..."
	@cp $(BUILD)/$(TARGET).nro $(CURDIR)/$(TARGET).nro

clean:
	@echo "Cleaning..."
	@rm -fr $(BUILD) $(TARGET).nro

#---------------------------------------------------------------------------------
else
.PHONY: all
DEPENDS := $(OFILES:.o=.d)

all: $(OUTPUT).nro

$(OUTPUT).nro: $(OUTPUT).elf $(OUTPUT).nacp
$(OUTPUT).elf: $(OFILES)
$(OFILES_SRC): $(HFILES_BIN)

%.bin.o %_bin.h : %.bin
	@echo $(notdir $<)
	@$(bin2o)

-include $(DEPENDS)
endif
#---------------------------------------------------------------------------------

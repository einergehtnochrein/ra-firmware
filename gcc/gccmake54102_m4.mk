#.SILENT:

PDL_LIBRARY=lpc54xxx-pdl

ifeq ($(RTOS_BRAND),NONE)
RTOS=none
RTOS_INCLUDE= \
    -I../libs/rtos/$(RTOS)
RTOS_SOURCEDIR= \
    ../libs/rtos/$(RTOS)
RTOS_SRCS= \
    cmsis_os_none.c
RTOS_CFLAGS=
RTOS_LDFLAGS=
endif
ifeq ($(RTOS_BRAND),FREERTOS)
RTOS=FreeRTOS-7.1.0
RTOS_INCLUDE= \
    -I$(PDL)/libs/rtos/$(RTOS) \
    -I$(PDL)/libs/rtos/$(RTOS)/src/include \
    -I$(PDL)/libs/rtos/$(RTOS)/src/portable/GCC/ARM_CM3
RTOS_SOURCEDIR= \
    $(PDL)/libs/rtos/$(RTOS) \
    $(PDL)/libs/rtos/$(RTOS)/src \
    $(PDL)/libs/rtos/$(RTOS)/src/portable/MemMang \
    $(PDL)/libs/rtos/$(RTOS)/src/portable/GCC/ARM_CM3
RTOS_SRCS= \
    cmsis_os.c croutine.c list.c queue.c tasks.c timers.c heap_2.c port.c
RTOS_CFLAGS= -DFREERTOS
RTOS_LDFLAGS=
endif
ifeq ($(RTOS_BRAND),RTX)
RTOS=RTX-4.70
RTOS_INCLUDE= \
    -I$(PDL)/libs/rtos/$(RTOS)
RTOS_SOURCEDIR= .
RTOS_SRCS= \
    RTX_Conf_CM.c
RTOS_CFLAGS= -DRTX
RTOS_LDFLAGS= -Wl,-L$(PDL)/libs/rtos/$(RTOS)/GCC -Wl,-lRTX_CM3
endif


export CC=arm-none-eabi-gcc
export CPP=arm-none-eabi-g++
export AR=arm-none-eabi-ar
export OBJCOPY=arm-none-eabi-objcopy
export OBJDUMP=arm-none-eabi-objdump
ARCH=arm-none-eabi-ar

export WARNINGS= \
    -Wextra -Wshadow -Wpointer-arith -Wcast-align -Wsign-compare -Wswitch \
    -Wmissing-declarations -Wunused \
    -Wall \
    -Wno-packed-bitfield-compat

export PDL=$(shell pwd)/../../../../..
ifeq (x$(OBJDIR),x)
export OBJDIR=obj
endif
ifeq (x$(MAKEFILE),x)
MAKEFILE=Makefile
endif
COMMONFLAGS += \
    -DCORE_M4 -march=armv7e-m \
    -DLPCLIB_FAMILY=LPCLIB_FAMILY_LPC5410X \
    $(WARNINGS) $(INCLUDES) $(BASEINCLUDE) \
    -mthumb \
    -T$(LINKER_SCRIPT) \
    -g -O1 -fomit-frame-pointer -ffunction-sections -fshort-wchar
CFLAGS += \
    $(COMMONFLAGS) \
    -Wstrict-prototypes -Wmissing-prototypes -Wbad-function-cast
CPPFLAGS += \
    $(COMMONFLAGS)
LDFLAGS += \
    -nostartfiles -Wl,-Map=$(OBJDIR)/$(PROJECT).map,--cref,--gc-sections,-o$@,--no-wchar-size-warning,--print-memory-usage -lm
EXE_DEPENDENCIES += \
    $(MAKEFILE) $(C_OBJ) $(CPP_OBJ) $(OBJDIR)/$(PDL_LIBRARY).a $(LINKER_SCRIPT) $(OBJDIR)/$(PDL_LIBRARY).a
export BASEINCLUDE= \
    -I../src \
    -I../lpc54000/inc \
    -I../core/Device/NXP/LPC54xxx/Include \
    -I../core/CMSIS/Include \
    $(RTOS_INCLUDE)

STARTUP_CFLAGS += \
    $(CFLAGS)



vpath %.cpp ../src
vpath %.c   ../src
vpath %     ../lpc54000/src
vpath %     ../core/Device/NXP/LPC54xxx/Source
vpath %     ../core/Device/NXP/LPC54xxx/Source/GCC
vpath %     $(RTOS_SOURCEDIR)


STARTUP_SRCS = \
    startup_LPC5410x.c 

PDL_LIBRARY_SRCS = \
    system_LPC5410x.c \
    \
    $(RTOS_SRCS) \
    \
    lpc54xxx_clkpwr.c lpc54xxx_crc.c lpc54xxx_gpio.c lpc54xxx_iap.c \
    lpc54xxx_iocon.c lpc54xxx_mrt.c lpc54xxx_timer.c lpc54xxx_uart.c



# Object files corresponding to C files.
C_OBJ = $(patsubst %,$(OBJDIR)/%,$(C_SRCS:.c=.o))
CPP_OBJ = $(patsubst %,$(OBJDIR)/%,$(CPP_SRCS:.cpp=.o))
PDL_LIBRARY_C_OBJ = $(patsubst %,$(OBJDIR)/%,$(PDL_LIBRARY_SRCS:.c=.o))
STARTUP_OBJ = $(patsubst %,$(OBJDIR)/%,$(STARTUP_SRCS:.c=.o))


.PHONY: all
all :: $(OBJDIR)/$(PDL_LIBRARY).a $(OBJDIR)/$(PROJECT).hex

$(OBJDIR)/$(PDL_LIBRARY).a : $(MAKEFILE) $(PDL_LIBRARY_C_OBJ) $(STARTUP_OBJ)
	@echo "Creating library $@..."
	@$(AR) r $@ $(PDL_LIBRARY_C_OBJ) $(STARTUP_OBJ)

$(OBJDIR)/$(PROJECT).hex : $(MAKEFILE) $(OBJDIR)/$(PROJECT).axf
	@$(OBJCOPY) $(OBJDIR)/$(PROJECT).axf -O ihex $@
	@$(OBJCOPY) $(OBJDIR)/$(PROJECT).axf -O binary $(OBJDIR)/$(PROJECT).bin

$(OBJDIR)/$(PROJECT).axf : $(EXE_DEPENDENCIES)
	@echo "Linking into $@..."
	@$(CC) $(CFLAGS) $(C_OBJ) $(CPP_OBJ) $(OBJDIR)/$(PDL_LIBRARY).a $(LDFLAGS)
	@$(OBJDUMP) -d -S $@ >$(OBJDIR)/$(PROJECT).lst

$(C_OBJ) : $(OBJDIR)/%.o : %.c $(MAKEFILE)
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $< -o $@
	@printf "$(dir $@)" > $(OBJDIR)/$*.d
	@$(CC) $(CFLAGS) -MM $< >> $(OBJDIR)/$*.d

$(CPP_OBJ) : $(OBJDIR)/%.o : %.cpp $(MAKEFILE)
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CPP) -c $(CPPFLAGS) $< -o $@

$(PDL_LIBRARY_C_OBJ) : $(OBJDIR)/%.o : %.c $(MAKEFILE)
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $< -o $@
	@printf "$(dir $@)" > $(OBJDIR)/$*.d
	@$(CC) $(CFLAGS) -MM $< >> $(OBJDIR)/$*.d

$(STARTUP_OBJ) : $(OBJDIR)/%.o : %.c $(MAKEFILE)
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CC) -c $(STARTUP_CFLAGS) $< -o $@
	@printf "$(dir $@)" > $(OBJDIR)/$*.d
	@$(CC) $(CFLAGS) -MM $< >> $(OBJDIR)/$*.d

#
# Make sure dependency files are considered
#
-include $(C_OBJ:.o=.d)


#
#  Utility targets
#
.PHONY: clean
clean :
	@rm -rf $(OBJDIR)



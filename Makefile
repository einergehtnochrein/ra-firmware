# Select the RTOS brand (FREERTOS, RTX, NONE)
export RTOS_BRAND=NONE

# The only task is to call the Makefile in the source directory
all:
	@echo Ra1 LPC54102 M0+ && cd gcc && $(MAKE) -f Makefile_m0
	@echo Ra1 LPC54102 M4  && cd gcc && $(MAKE) -f Makefile_m4
	@echo Ra2 LPC54102 M0+ && cd gcc && $(MAKE) -f Makefile_m0_ra2
	@echo Ra2 LPC54102 M4  && cd gcc && $(MAKE) -f Makefile_m4_ra2

clean:
	@echo Ra1 LPC54102 M0+ && cd gcc && $(MAKE) -f Makefile_m0 clean
	@echo Ra1 LPC54102 M4  && cd gcc && $(MAKE) -f Makefile_m4 clean
	@echo Ra2 LPC54102 M0+ && cd gcc && $(MAKE) -f Makefile_m0_ra2 clean
	@echo Ra2 LPC54102 M4  && cd gcc && $(MAKE) -f Makefile_m4_ra2 clean


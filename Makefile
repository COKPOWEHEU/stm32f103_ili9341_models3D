progname := firmware
progdir := res
files = main lcd_ili9341 gl
srcdir = src
builddir = build

OPTIMIZE = s
DEBUG = dwarf-2

startup = ../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s
FWLIB = ../Libraries
PHDRIVER=STM32F10x_StdPeriph_Driver
CORESUPPORT=CMSIS/CM3/CoreSupport
DEVSUPPORT=CMSIS/CM3/DeviceSupport/ST/STM32F10x
INCLUDS = -I$(FWLIB)/$(PHDRIVER)/inc -I$(FWLIB)/$(CORESUPPORT) -I$(FWLIB)/$(DEVSUPPORT)
LINKERCMDFILE = ../ld/stm32f10xC8.ld
#DEVCLASS =  STM32F10X_LD     #/*!< STM32F10X_LD: STM32 Low density devices */
#DEVCLASS =  STM32F10X_LD_VL  #/*!< STM32F10X_LD_VL: STM32 Low density Value Line devices */  
DEVCLASS =  STM32F10X_MD     #/*!< STM32F10X_MD: STM32 Medium density devices */
#DEVCLASS =  STM32F10X_MD_VL  #/*!< STM32F10X_MD_VL: STM32 Medium density Value Line devices */  
#DEVCLASS =  STM32F10X_HD     #/*!< STM32F10X_HD: STM32 High density devices */
#DEVCLASS =  STM32F10X_HD_VL  #/*!< STM32F10X_HD_VL: STM32 High density value line devices */  
#DEVCLASS =  STM32F10X_XL     #/*!< STM32F10X_XL: STM32 XL-density devices */
#DEVCLASS =  STM32F10X_CL     #/*!< STM32F10X_CL: STM32 Connectivity line devices */

CFLAGS = -fno-builtin -Wall

CFLAGS += -MD -MP -MT $(builddir)/$(*F).o -MF $(builddir)/dep/$(@F).mk
CFLAGS += -mthumb -mcpu=cortex-m3 -g$(DEBUG) -O$(OPTIMIZE) -D$(DEVCLASS) $(INCLUDS) -DGCC_ARMCM3 -DVECT_TAB_FLASH
ASMFLAGS = -ahls -mapcs-32
LFLAGS  = -T$(LINKERCMDFILE) -nostartfiles -Wl,-Map -Wl,$(builddir)/$(progname).map -mthumb -mcpu=cortex-m3

CC      = arm-none-eabi-gcc
CPP      = arm-none-eabi-g++
#LD      = arm-none-eabi-ld -v
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD	= arm-none-eabi-objdump
SZ	= arm-none-eabi-size

frmname = $(progdir)/$(progname)
objs = $(addprefix $(builddir)/,$(addsuffix .o,$(files)))

all: $(frmname).bin $(frmname).hex $(frmname).lss size
release: all
	rm -rf $(builddir)
clean:
	rm -rf $(progdir)
	rm -rf $(builddir)
prog: $(frmname).bin
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "init" -c "reset halt" -c "flash write_image erase "$(frmname).bin" 0x08000000" -c "reset run" -c "exit"
$(frmname).bin: $(frmname).elf
	$(CP) -Obinary $(frmname).elf $(frmname).bin
$(frmname).hex: $(frmname).elf
	$(CP) -Oihex $(frmname).elf $(frmname).hex
$(frmname).lss: $(frmname).elf
	$(OD) -D -S $(frmname).elf > $(frmname).lss
size: $(frmname).elf
	$(SZ) $(frmname).elf
$(frmname).elf: $(objs) $(LINKERCMDFILE) $(builddir)/crt.o
	mkdir -p $(progdir)
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $(frmname).elf $(builddir)/crt.o -L../ld $(objs)
$(builddir)/crt.o:
	mkdir -p $(builddir)
	$(AS) -o $(builddir)/crt.o $(startup)
	
$(builddir)/%.o: $(srcdir)/%.c
	mkdir -p $(builddir)
	$(CC) $(CFLAGS) -c $< -o $@
	
-include $(shell mkdir -p $(builddir)/dep) $(wildcard $(builddir)/dep/*)

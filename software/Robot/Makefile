PROJECT         := Robot
DEVICES         := LPC1768
GCC4MBED_DIR    := ../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0
GCC4MBED_TYPE   := Develop
USER_LIBS       := ../libs/DmaSerial \
                   ../libs/GPDMA \
                   ../libs/AdafruitPrecision9DoF \
                   ../libs/ConfigFile \
                   ../libs/FlashFileSystem \
                   $(GCC4MBED_DIR)/external/mbed-os/features/unsupported/dsp

MBED_OS_ENABLE    := 0
MRI_ENABLE        := 0
MRI_BREAK_ON_INIT := 1
#MRI_UART          := MRI_UART_MBED_P28_P27 MRI_UART_BAUD=115200
MRI_UART          := MRI_UART_MBED_USB MRI_UART_BAUD=230400

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

main.cpp : files.h

FILES_H_DEPENDENTS := $(wildcard ../../files/*)

files.h : files.bin
	$(Q) ../../mriprog/bin2h $< $@ g_fileSystemData

files.bin : ../../files/ $(FILES_H_DEPENDENTS)
	$(Q) ../../macos/fsbld ../../files $@


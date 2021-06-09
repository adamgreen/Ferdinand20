PROJECT         := SawppyDrive
DEVICES         := LPC1768
GCC4MBED_DIR    := ../../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0
GCC4MBED_TYPE   := Develop

USER_LIBS       := !../../PdbSerial !../../SawppyDrive !../../LX16A ../../HalfDuplexSerial ../../PID ../../DebugSerial ../../GPDMA ../../Interlocked

MBED_OS_ENABLE    := 0
MRI_ENABLE        := 1
MRI_BREAK_ON_INIT := 1
MRI_UART          := MRI_UART_MBED_P28_P27 MRI_UART_BAUD=115200

include $(GCC4MBED_DIR)/build/gcc4mbed.mk


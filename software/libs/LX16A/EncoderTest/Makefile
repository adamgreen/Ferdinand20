PROJECT         := EncoderTest
DEVICES         := LPC1768
GCC4MBED_DIR    := ../../../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0
GCC4MBED_TYPE   := Develop

USER_LIBS       := !../../LX16A \
                   ../../PID \
                   ../../HalfDuplexSerial \
                   $(GCC4MBED_DIR)/external/mbed-os/features/unsupported/dsp


MBED_OS_ENABLE  := 0
MRI_ENABLE      := 0
MRI_UART        := MRI_UART_MBED_USB MRI_UART_BAUD=230400

include $(GCC4MBED_DIR)/build/gcc4mbed.mk


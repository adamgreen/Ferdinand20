PROJECT_NAME     := blemri
TARGETS          := nrf51422_xxac
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../nrfSDK
NRF51_TOOL_PATH := ../../nrfTools
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf51422_xxac.out: \
  LINKER_SCRIPT  := nrf51.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/block_dev/sdc/nrf_block_dev_sdc.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/pwm/app_pwm.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/sdcard/app_sdcard.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/uart/retarget.c \
  $(SDK_ROOT)/components/libraries/util/sdk_errors.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/ppi/nrf_drv_ppi.c \
  $(SDK_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/drivers_nrf/rng/nrf_drv_rng.c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/external/fatfs/port/diskio_blkdev.c \
  $(SDK_ROOT)/external/fatfs/src/ff.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.S \
  $(SDK_ROOT)/components/toolchain/system_nrf51.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_btn_ble.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/simple_timer/app_simple_timer.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/drivers_nrf/comp \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/softdevice/s130/headers \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/drivers_nrf/i2s \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/drivers_nrf/adc \
  $(SDK_ROOT)/components/softdevice/s130/headers/nrf51 \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/drivers_nrf/wdt \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/external/fatfs/port \
  $(SDK_ROOT)/external/fatfs/src \
  $(SDK_ROOT)/external/protothreads/pt-1.4 \
  $(SDK_ROOT)/external/protothreads \
  $(SDK_ROOT)/components/libraries/block_dev/sdc \
  $(SDK_ROOT)/components/libraries/block_dev \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/drivers_nrf/rtc \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/drivers_nrf/ppi \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/components/drivers_nrf/twis_slave \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/drivers_nrf/spi_slave \
  $(SDK_ROOT)/components/drivers_nrf/lpcomp \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/drivers_nrf/power \
  $(SDK_ROOT)/components/libraries/usbd/config \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/drivers_nrf/qdec \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/drivers_nrf/pdm \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/drivers_nrf/swi \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/drivers_nrf/saadc \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/drivers_nrf/usbd \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/libraries/simple_timer \
  $(PROJ_DIR) \

# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF51422
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -DDEBUG_NRF_USER
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF51422
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

.PHONY: $(TARGETS) default all clean help flash flash_softdevice sdk

# Default target - first one defined
default: nrf51422_xxac

# Print all targets that can be built
help:
	@echo The following rules are available:
	@echo   sdk - Needs to be run first to install Nordic SDK, required for build.
	@echo   clean - Deletes all output binaries so that next build is performed from scratch.
	@echo   all - Builds all supported targets - nrf51422_xxac.
	@echo   erase - Erases all of the FLASH on the nRF5 device.
	@echo   flash_softdevice - Flashes the Nordic BLE Soft Device. Will erase device first.
	@echo   flash - Just flashes your application firmware. flash_softdevice must be run at least once before.
	@echo 	nrf51422_xxac

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

-include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf51422_xxac.hex
	@echo Flashing: $<
	tools/nrfjprog/nrfjprog --program $< -f nrf51 --sectorerase
	tools/nrfjprog/nrfjprog --reset -f nrf51

# Flash softdevice
flash_softdevice: erase
	@echo Flashing: s130_nrf51_2.0.1_softdevice.hex
	$(NRF51_TOOL_PATH)/nrfjprog/nrfjprog --program $(SDK_ROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex -f nrf51 --sectorerase
	$(NRF51_TOOL_PATH)/nrfjprog/nrfjprog --reset -f nrf51

erase:
	$(NRF51_TOOL_PATH)/nrfjprog/nrfjprog --eraseall -f nrf51


SDK_VER := nRF5_SDK_12.3.0_d7731ad
SDK_ZIP := $(SDK_VER).zip
SDK_URL := https://developer.nordicsemi.com/nRF5_SDK/pieces/nRF5_SDK_v12.x.x/$(SDK_ZIP)
SDK_ZIP_PATH = $(dir $(SDK_ROOT))$(SDK_ZIP)

# sdk target is used to download SDK from the Nordic website.
sdk: $(SDK_ROOT)

# The SDK source code only exists after it has been unzipped from the SDK archive.
$(SDK_ROOT): $(SDK_ZIP_PATH)
	@echo Unzipping $<
	$(NO_ECHO)unzip -d $@ $<
	$(NO_ECHO)mv $(SDK_ROOT)/$(SDK_VER)/* $(SDK_ROOT)
	$(NO_ECHO)rm -d $(SDK_ROOT)/$(SDK_VER)
	$(NO_ECHO)patch -d $(SDK_ROOT) -p1 <$(dir $(SDK_ROOT))sdk.patch

# Download SDK archive from the Nordic website.
$(SDK_ZIP_PATH):
	@echo Downloading $@
	$(NO_ECHO)curl -L0 $(SDK_URL) >$@

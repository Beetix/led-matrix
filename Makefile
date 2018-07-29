PROGRAM=LED_Matrix
PROGRAM_DIR=$(dir $(firstword $(MAKEFILE_LIST)))
PROGRAM_SRC_DIR=$(PROGRAM_DIR)/src
PROGRAM_INC_DIR=$(PROGRAM_DIR)/src

EXTRA_COMPONENTS=lib/ugui
ugui_ROOT=$(PROGRAM_DIR)/lib/ugui
include ../esp-open-rtos/common.mk

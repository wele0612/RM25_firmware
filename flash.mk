# Author: Liwei Xue 2024.3.20
# Flash script for makefile project, using OPENOCD.

# Detect OS
ifeq ($(OS),Windows_NT)
    # Windows: use environment OPENOCD_HOME
else
    # Linux / Unix: default to /usr if not set
    OPENOCD_HOME ?= /usr
endif


OCDSCRIPT_PATH = $(subst \,/,$(OPENOCD_HOME))/share/openocd/scripts
#On windows "\" in file path will cause problems in OPENOCD

PROJECT = RM25_firmware
#TARGET = stm32h7x_noreset
TARGET = stm32h7x
INTERFACE = stlink-v2

OCD = openocd
ELF = build/$(PROJECT).elf

default: load

load: build_code
	$(OCD) -f $(OCDSCRIPT_PATH)/interface/$(INTERFACE).cfg \
	 -f  $(OCDSCRIPT_PATH)/target/$(TARGET).cfg \
	 -c "program $(ELF) verify reset exit"

build_code:
	make -C ./ -j

showpath:
	@echo $(OPENOCD_HOME)
	@echo $(OCDSCRIPT_PATH)

.PHONY: build_code load

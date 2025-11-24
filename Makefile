.PHONY: all clean

SEGGER_DIR ?= /Applications/SEGGER/SEGGER Embedded Studio 8.24
BUILD_CONFIG ?= Debug
SEGGER_PROJECT ?= aiot_play_fw.emProject

all:
	@echo "\e[1mBuilding all projects\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" $(SEGGER_PROJECT) -config $(BUILD_CONFIG) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

clean:
	"$(SEGGER_DIR)/bin/emBuild" $(SEGGER_PROJECT) -config $(BUILD_CONFIG) -clean

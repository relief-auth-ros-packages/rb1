SOURCES = speedway_embedded_example.cpp
LIB_DIR = /usr/bin
MPNJ_LIB_DIR = lib
HEADER_DIR = include

ETK_DEFAULT_INSTALL_DIR = /home/etk/impinj_etk
INSTALL_TOOL_HELP = \
"Failed to find the arm-none-linux-gnueabi-g++ compiler." \
"Please make sure that /home/etk/impinj_etk/arm-toolchain/bin is in your PATH."
INSTALL_ETK_HELP = "Please follow the ETK install instructions and make sure that" \
"/home/etk/impinj_etk is in your PATH, or that the ETK_INSTALL_DIR is defined."

# The cap_gen tool may be in the PATH, the install dir, or in the current dir.
CAP_GEN_EXE = cap_gen.sh
CAP_GEN_IN_PATH=$(shell which $(CAP_GEN_EXE))
CAP_GEN_CWD=$(shell ls ./$(CAP_GEN_EXE) 2>/dev/null)
CAP_GEN_DEFAULT=$(shell ls $(ETK_DEFAULT_INSTALL_DIR)/$(CAP_GEN_EXE) 2>/dev/null)
ifneq (,$(CAP_GEN_IN_PATH))
	CAP_GEN=$(CAP_GEN_IN_PATH)
else
	ifneq (,$(ETK_INSTALL_DIR))
		CAP_GEN=$(ETK_INSTALL_DIR)/$(CAP_GEN_EXE)
	else
		ifneq (,$(CAP_GEN_DEFAULT))
			CAP_GEN=$(CAP_GEN_DEFAULT)
		else
			ifneq (,$(CAP_GEN_CWD))
				CAP_GEN=$(CAP_GEN_CWD)
			endif
		endif
	endif
endif


all: x86 arm

help:
	@echo Example use:
	@echo  ‘make arm’ to build the sample for on-reader use
	@echo  ‘make x86’ to build the sample for the (x86) host
	@echo  ‘make cap’ to build a CAP upgrade file

bin/speedwayr_x86:
	mkdir -p ./bin
	g++ \
		-m32 -Wno-write-strings \
		-I$(HEADER_DIR) \
		$(SOURCES) \
		-L$(MPNJ_LIB_DIR) -lltkcpp_x86 -lltkcppimpinj_x86 -lxml2_x86 \
		-L$(LIB_DIR) -ldl -lssl -lcrypto \
		-o bin/speedwayr_x86

x86: bin/speedwayr_x86

bin/speedwayr_arm: check_env
	mkdir -p ./bin
	arm-none-linux-gnueabi-g++ \
		-Wno-write-strings \
		-I$(HEADER_DIR) \
		$(SOURCES) \
		-L$(MPNJ_LIB_DIR) \
		-static -lltkcpp_atmel -lltkcppimpinj_atmel -lxml2_atmel \
		-lssl_atmel -lcrypto_atmel -ldl_atmel \
		-o bin/speedwayr_arm
	arm-none-linux-gnueabi-strip bin/speedwayr_arm

arm: bin/speedwayr_arm

cap: arm check_env
	$(CAP_GEN) -d cap_description.in -o speedwayr_cap.upg

clean:
	rm -rf bin/*
	rm -rf speedwayr_cap.upg

.PHONY: check_env
check_env:
	@if ! which $(CAP_GEN_EXE) > /dev/null && \
		[ ! -f $(ETK_INSTALL_DIR)/$(CAP_GEN_EXE) > /dev/null ] && \
		[ ! -f ./$(CAP_GEN_EXE) > /dev/null ]; then \
		echo "Failed to find $(CAP_GEN_EXE)."; \
		echo $(INSTALL_ETK_HELP); \
		exit 1; \
	fi
	@if ! which arm-none-linux-gnueabi-g++ > /dev/null; then \
		echo $(INSTALL_TOOL_HELP); \
		exit 1; \
	fi


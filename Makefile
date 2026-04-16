APP_PARAM ?= ../../Makefile.param
ifneq ($(strip $(APP_PARAM)),)
include $(APP_PARAM)
endif

export LC_ALL=C
SHELL := /bin/bash

CURRENT_DIR := $(shell pwd)
BUILD_DIR := $(CURRENT_DIR)/out
OBJ_DIR := $(BUILD_DIR)/obj
BIN_DIR := $(BUILD_DIR)/bin

CC = ../../../../tools/linux/toolchain/arm-rockchip1240-linux-gnueabihf/bin/arm-rockchip1240-linux-gnueabihf-gcc
STRIP ?= $(if $(RK_APP_CROSS),$(RK_APP_CROSS)-strip,strip)

CFLAGS += -I$(CURRENT_DIR)/include
CFLAGS += -Wall -O2 $(RK_APP_OPTS)
LDFLAGS +=
LDLIBS += -lpthread

TARGETS := motion recive_irevent IR_recharge
BIN_TARGETS := $(addprefix $(BIN_DIR)/,$(TARGETS))

MOTION_SRCS := \
	$(CURRENT_DIR)/src/motion.c \
	$(CURRENT_DIR)/src/motion_cli.c

IR_SRCS := \
	$(CURRENT_DIR)/src/motion.c \
	$(CURRENT_DIR)/src/recive_irevent.c

IR_RECHARGE_SRCS := \
	$(CURRENT_DIR)/src/motion.c \
	$(CURRENT_DIR)/IR_recharge.c

MOTION_OBJS := $(MOTION_SRCS:$(CURRENT_DIR)/%.c=$(OBJ_DIR)/%.o)
IR_OBJS := $(IR_SRCS:$(CURRENT_DIR)/%.c=$(OBJ_DIR)/%.o)
IR_RECHARGE_OBJS := $(IR_RECHARGE_SRCS:$(CURRENT_DIR)/%.c=$(OBJ_DIR)/%.o)

.PHONY: all clean print debug motion recive_irevent IR_recharge

all: $(BIN_TARGETS)

motion: $(BIN_DIR)/motion

recive_irevent: $(BIN_DIR)/recive_irevent

IR_recharge: $(BIN_DIR)/IR_recharge

$(BIN_DIR)/motion: $(MOTION_OBJS)
	@mkdir -p $(dir $@)
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BIN_DIR)/recive_irevent: $(IR_OBJS)
	@mkdir -p $(dir $@)
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BIN_DIR)/IR_recharge: $(IR_RECHARGE_OBJS)
	@mkdir -p $(dir $@)
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: $(CURRENT_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

print:
	@echo "CC=$(CC)"
	@echo "CFLAGS=$(CFLAGS)"
	@echo "LDFLAGS=$(LDFLAGS)"
	@echo "LDLIBS=$(LDLIBS)"
	@echo "TARGETS=$(TARGETS)"
	@echo "BIN_DIR=$(BIN_DIR)"
	@echo "BIN_TARGETS=$(BIN_TARGETS)"

debug: print
	@echo "CURRENT_DIR=$(CURRENT_DIR)"
	@echo "BUILD_DIR=$(BUILD_DIR)"
	@echo "OBJ_DIR=$(OBJ_DIR)"
	@echo "BIN_DIR=$(BIN_DIR)"

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TARGETS)

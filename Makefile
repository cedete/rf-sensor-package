# ------------------------------------------------------------------------------
# ----------------------------- STM32 HAL Build --------------------------------
# ------------------------------------------------------------------------------

# Toolchain 
ARMGCC_ROOT_DIR  := /Applications/gcc-arm-none-eabi-10.3
ARMGCC_BIN_DIR   := $(ARMGCC_ROOT_DIR)/bin
OPENOCD_ROOT_DIR := /Applications/xpack-openocd-0.12.0-4
OPENOCD_BIN_DIR  := $(OPENOCD_ROOT_DIR)/bin

# Project Directories
BUILD_DIR  = build
BIN_DIR    = bin
SRC_DIR    = src
APP_DIR    = $(SRC_DIR)/app
DRIVER_DIR = $(SRC_DIR)/drivers
BOARD_DIR  = $(SRC_DIR)/board
TEST_DIR   = test
START_DIR  = startup

# Compiler 
native: CXX = g++
cross:  CXX = $(ARMGCC_BIN_DIR)/arm-none-eabi-g++

# Debugging 
DEBUG      = $(OPENOCD_BIN_DIR)/openocd
FLFLAGS = -f $(OPENOCD_ROOT_DIR)/openocd/scripts/interface/stlink.cfg \
          -f $(OPENOCD_ROOT_DIR)/openocd/scripts/target/stm32wlx.cfg

OBJDUMP = $(ARMGCC_BIN_DIR)/arm-none-eabi-objdump

# Testing Flag
TEST_FLAG ?= UNIT_TEST    

# Compilation and linking flags
native: CXXFLAGS = -std=c++17 -g3 -fno-exceptions -fno-rtti -D$(TEST_FLAG)
cross:  CXXFLAGS = -mcpu=cortex-m4 -std=gnu++17 -g3 -O0 \
                   -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fstack-usage \
                   -mfloat-abi=soft -mthumb
        WFLAGS   = -Wall -Wextra -Wshadow #-Werror
        LDFLAGS  = -mcpu=cortex-m4 -T"$(START_DIR)/stm32wl55xx.ld" --specs=nosys.specs -Wl,--gc-sections \
                   -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group \
                   -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -Wl,--gc-sections -lm
        OBJFLAGS = -S -C

# Files
BOARD_FILES  = $(wildcard $(BOARD_DIR)/*.cpp)
DRIVER_FILES = $(wildcard $(DRIVER_DIR)/*.cpp)
APP_FILES    = $(wildcard $(APP_DIR)/*.cpp)
START_FILE   = $(START_DIR)/startup_stm32wl55xx.cpp
MAIN_FILE    = $(SRC_DIR)/main.cpp
TEST_FILE    = $(TEST_DIR)/test.cpp 

NATIVE_SRCS = $(DRIVER_FILES) $(APP_FILES) $(BOARD_FILES) $(TEST_FILE)
CROSS_SRCS  = $(DRIVER_FILES) $(APP_FILES) $(BOARD_FILES) $(MAIN_FILE) $(START_FILE)

NATIVE_OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(NATIVE_SRCS))
CROSS_OBJS  = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CROSS_SRCS))

NATIVE_EXEC = $(BIN_DIR)/main.x
CROSS_EXEC  = $(BIN_DIR)/main.elf 

# ------------------------------------------------------------------------------

# Compilation/Linking

$(NATIVE_EXEC): $(NATIVE_OBJS)
	@mkdir -p $(dir $@)
	@echo "Linking $^"
	@$(CXX) $(CXXFLAGS) $(WFLAGS) -o $@ $^

$(CROSS_EXEC): $(CROSS_OBJS)
	@mkdir -p $(dir $@)
	@echo "Linking $^"
	@$(CXX) $(CXXFLAGS) $(WFLAGS) $(LDFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: %.cpp 
	@mkdir -p $(dir $@)
	@echo "Compiling $^"
	@$(CXX) $(CXXFLAGS) $(WFLAGS) -c $^ -o $@    

# ------------------------------------------------------------------------------

# Phony targets

.PHONY: native asm cross run flash clean

# Debugging
asm: 
	@$(OBJDUMP) $(OBJFLAGS) $(CROSS_OBJS)

native: $(NATIVE_EXEC)
	@echo "Building $^"
	@echo "Running $^"
	@$(NATIVE_EXEC)

cross: $(CROSS_EXEC)
	@echo "Building $^"

flash: 
	$(DEBUG) $(FLFLAGS) -c "program $(CROSS_EXEC) reset verify exit" 

clean:
	@echo "Removing build files and executables"
	@rm -rf $(BIN_DIR) $(BUILD_DIR) 

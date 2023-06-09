SRCS = $(shell ls *.c)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_RPI)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_RPI)
DEPS := $(DEPS_STD) $(DEPS_COMMON) $(DEPS_RPI)

include $(BUILD_COMMON)


all: $(BIN_PATH)/uart-test 
	@/bin/true

$(BIN_PATH)/uart-test: uart_test.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)


clean:
	@rm -rf *.o $(BIN_PATH)/uart-test

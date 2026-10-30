# GPL-3.0-only. Run in code/firmware/stm32; no flash/deploy side effects.
PYTHON ?= ../../../.venv/Scripts/python.exe
GCC ?= arm-none-eabi-gcc
.PHONY: all target gcc test clean
all: target
target:
	$(PYTHON) build.py
gcc:
	$(PYTHON) build.py --gcc $(GCC)
test:
	$(PYTHON) tools/host_test.py --cflags=-O2 --target-check
# Build files are ignored; removal is deliberately left to the operator.
clean:
	@echo Remove build/ manually when no other package is using it.

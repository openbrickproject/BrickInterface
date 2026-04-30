# Build and run host-side unit tests for BrickInterface firmware logic.
#
#   make           # build all test binaries
#   make run       # build + run, show summary
#   make clean     # remove build artifacts
#
# Tests compile firmware sources directly with the local Arduino.h stub on
# the include path, mocking out CH552-specific hardware as ordinary globals.

CXX        := g++
CXXFLAGS   := -std=c++17 -Wall -Wno-unused-function -Wno-unused-variable \
              -Wno-unused-parameter -Wno-non-virtual-dtor -Wno-missing-field-initializers \
              -I. -I..
LDFLAGS    :=

HARNESS    := test_harness.cpp

TESTS      := test_packet test_ir_pf test_ir_legacy test_ir_rcx test_dispatch

all: $(TESTS)

test_packet: test_packet.cpp $(HARNESS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_ir_pf: test_ir_pf.cpp $(HARNESS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_ir_legacy: test_ir_legacy.cpp $(HARNESS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_ir_rcx: test_ir_rcx.cpp $(HARNESS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_dispatch: test_dispatch.cpp $(HARNESS)
	$(CXX) $(CXXFLAGS) $^ -o $@

run: all
	@echo ""; \
	pass=0; fail=0; \
	for t in $(TESTS); do \
	    echo "==> $$t"; \
	    if ./$$t; then pass=$$((pass+1)); else fail=$$((fail+1)); fi; \
	    echo ""; \
	done; \
	echo "================================="; \
	echo "Suites: $$pass pass, $$fail fail"; \
	test $$fail -eq 0

clean:
	rm -f $(TESTS)

.PHONY: all run clean

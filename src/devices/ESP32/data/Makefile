# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra

# Directories
SRC_DIR = .
BUILD_DIR = build

# Source Files
COMMON_SRCS = physics_engine.cpp pid_controller.cpp
SITL_SRCS = sitl_main.cpp $(COMMON_SRCS)
HITL_SRCS = hitl_main.cpp $(COMMON_SRCS)

# Targets
SITL_TARGET = sitl_sim
HITL_TARGET = hitl_flight

# Rules
all: sitl hitl

sitl: $(SITL_SRCS)
	$(CXX) $(CXXFLAGS) -o $(SITL_TARGET) $(SITL_SRCS)
	@echo "Build successful! Run ./$(SITL_TARGET) to start simulation."

hitl: $(HITL_SRCS)
	$(CXX) $(CXXFLAGS) -o $(HITL_TARGET) $(HITL_SRCS)
	@echo "Build successful! (Check hardware dependencies for RPi)."

clean:
	rm -f $(SITL_TARGET) $(HITL_TARGET) sitl_flight_log.csv

log:
	python3 plot_log.py

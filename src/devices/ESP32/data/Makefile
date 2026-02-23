# Compiler and Flags
CXX = g++
# Added -O2 for math optimization on the Pi, and -I. to include local headers easily
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.

# Directories
SRC_DIR = .
BUILD_DIR = build

# --- SOURCE GROUPS ---
# 1. Pure Math and Logic (Used by everything)
MATH_SRCS = $(SRC_DIR)/physics_engine.cpp \
            $(SRC_DIR)/pid_controller.cpp \
            $(SRC_DIR)/KalmanFilter.cpp

# 2. Hardware Drivers (Only used on the Pi)
HW_SRCS = $(SRC_DIR)/ServoControllerI2C.cpp \
          $(SRC_DIR)/ICM20948.cpp \
          $(SRC_DIR)/BMP390.cpp

# --- TARGET CONFIGURATIONS ---

# SITL (Pure PC Simulation)
SITL_SRCS = $(SRC_DIR)/sitl_main.cpp $(MATH_SRCS)
SITL_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SITL_SRCS))
SITL_TARGET = sitl_sim

# SITL + Servo (Hybrid Desk Test)
SITL_SERVO_SRCS = $(SRC_DIR)/sitl_servo_test.cpp $(MATH_SRCS) $(SRC_DIR)/ServoControllerI2C.cpp
SITL_SERVO_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SITL_SERVO_SRCS))
SITL_SERVO_TARGET = sitl_servo

# HITL (Full Flight Computer)
HITL_SRCS = $(SRC_DIR)/hitl_main.cpp $(MATH_SRCS) $(HW_SRCS)
HITL_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(HITL_SRCS))
HITL_TARGET = hitl_flight

# --- BUILD RULES ---
.PHONY: all clean log directories sitl sitl_servo hitl

all: directories sitl sitl_servo hitl

directories:
	@mkdir -p $(BUILD_DIR)

# Generic rule to compile .cpp to .o inside the build directory
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | directories
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Pure Simulation Build
sitl: $(SITL_OBJS)
	$(CXX) $(CXXFLAGS) -o $(SITL_TARGET) $(SITL_OBJS)
	@echo "==> Pure SITL Build successful! Run ./$(SITL_TARGET)"

# Hybrid Simulation + Actuator Build
sitl_servo: $(SITL_SERVO_OBJS)
	$(CXX) $(CXXFLAGS) -o $(SITL_SERVO_TARGET) $(SITL_SERVO_OBJS)
	@echo "==> Hybrid SITL+Servo Build successful! Run ./$(SITL_SERVO_TARGET)"

# Full Flight Computer Build
hitl: $(HITL_OBJS)
	$(CXX) $(CXXFLAGS) -o $(HITL_TARGET) $(HITL_OBJS)
	@echo "==> Full Flight Computer Build successful! Run ./$(HITL_TARGET)"

# Utility Rules
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SITL_TARGET) $(SITL_SERVO_TARGET) $(HITL_TARGET) sitl_flight_log.csv
	@echo "==> Cleaned build artifacts."

log:
	python3 plot_log.py
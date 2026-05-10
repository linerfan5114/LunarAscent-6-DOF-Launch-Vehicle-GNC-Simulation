CXX = g++
CXXFLAGS = -std=c++17 -O3 -march=native -Wall -Wextra
INCLUDES = -Iinclude
LDFLAGS = -lm

TARGET = lunar_ascent
SRC_DIR = src
INCLUDE_DIR = include
SIM_DIR = sim

SRCS = $(SRC_DIR)/vehicle.cpp \
       $(SRC_DIR)/propulsion.cpp \
       $(SRC_DIR)/environment.cpp \
       $(SRC_DIR)/guidance.cpp \
       $(SRC_DIR)/navigation.cpp \
       $(SRC_DIR)/control.cpp \
       $(SRC_DIR)/main.cpp

OBJS = $(SRCS:.cpp=.o)

.PHONY: all clean run plot distclean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)
	@echo "Build complete: ./$(TARGET)"

$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCLUDE_DIR)/%.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(SRC_DIR)/main.o: $(SRC_DIR)/main.cpp $(wildcard $(INCLUDE_DIR)/*.hpp)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

plot:
	python3 $(SIM_DIR)/plot_trajectory.py trajectory.csv

clean:
	rm -f $(SRC_DIR)/*.o $(TARGET)

distclean: clean
	rm -f trajectory.csv trajectory_profile.png
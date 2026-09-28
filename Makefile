CXX := clang++

CPPFLAGS := -Iinclude -Itests
CXXFLAGS := -std=c++20 -Wall -Wextra -Wpedantic -g

# SFML via Homebrew. Resolving the prefix once keeps the build portable across
# Apple Silicon and Intel Homebrew installations.
SFML_PREFIX := $(shell brew --prefix sfml)
SFML_CPPFLAGS := -isystem $(SFML_PREFIX)/include
SFML_LIBS := -L$(SFML_PREFIX)/lib -lsfml-graphics -lsfml-window -lsfml-system

SRC_DIR := src
TEST_DIR := tests
BUILD_DIR := build

# Source discovery follows the module layout. Adding a .cpp file to one of
# these directories automatically includes it in the appropriate target.
CORE_SRCS := $(sort $(wildcard \
	$(SRC_DIR)/math/*.cpp \
	$(SRC_DIR)/geometry/*.cpp \
	$(SRC_DIR)/collision/*.cpp \
	$(SRC_DIR)/dynamics/*.cpp))
RENDER_SRCS := $(wildcard $(SRC_DIR)/rendering/*.cpp)
APP_SRCS := $(wildcard $(SRC_DIR)/app/*.cpp)

TEST_SRCS := $(filter-out $(TEST_DIR)/visual/%,$(wildcard $(TEST_DIR)/*/*.cpp))
VISUAL_SRCS := $(wildcard $(TEST_DIR)/visual/*.cpp)

CORE_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_SRCS))
RENDER_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RENDER_SRCS))
APP_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(APP_SRCS))
TEST_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SRCS))
VISUAL_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(VISUAL_SRCS))

ALL_OBJS := $(CORE_OBJS) $(RENDER_OBJS) $(APP_OBJS) $(TEST_OBJS) $(VISUAL_OBJS)

PROG_BIN := tinysim
TEST_BIN := run_tests
VISUAL_BIN := visual_check

.PHONY: all build test visual clean

all: build

build: $(PROG_BIN)

$(PROG_BIN): $(CORE_OBJS) $(RENDER_OBJS) $(APP_OBJS)
	$(CXX) $^ $(SFML_LIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(CORE_OBJS) $(RENDER_OBJS) $(TEST_OBJS)
	$(CXX) $^ $(SFML_LIBS) -o $@

visual: $(VISUAL_BIN)
	./$(VISUAL_BIN)

$(VISUAL_BIN): $(CORE_OBJS) $(RENDER_OBJS) $(VISUAL_OBJS)
	$(CXX) $^ $(SFML_LIBS) -o $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SFML_CPPFLAGS) -MMD -MP -c $< -o $@

-include $(ALL_OBJS:.o=.d)

clean:
	rm -rf $(BUILD_DIR) \
		$(PROG_BIN) $(TEST_BIN) $(VISUAL_BIN) \
		$(PROG_BIN).dSYM $(TEST_BIN).dSYM $(VISUAL_BIN).dSYM

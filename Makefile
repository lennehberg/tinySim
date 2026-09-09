CXX := clang++
CXXFLAGS := -std=c++20 -Wall -Wextra -Wpedantic -g -Iinclude -I.

# SFML via Homebrew. Asking brew for the prefix keeps this working on both
# Apple Silicon (/opt/homebrew) and Intel (/usr/local), and survives version
# bumps. ':=' so brew runs once per make, not once per rule.
SFML_PREFIX := $(shell brew --prefix sfml)
SFML_CFLAGS := -isystem $(SFML_PREFIX)/include
SFML_LIBS   := -L$(SFML_PREFIX)/lib -lsfml-graphics -lsfml-window -lsfml-system

SRC_DIR := src
TEST_DIR := tests

# Physics core: no SFML, linked into both binaries.
LIB_SRCS := $(SRC_DIR)/Vec2.cpp \
			$(SRC_DIR)/Transform.cpp \
			$(SRC_DIR)/Shape.cpp \
			$(SRC_DIR)/BoxShape.cpp \
			$(SRC_DIR)/CircleShape.cpp \
			$(SRC_DIR)/Simplex.cpp \
			$(SRC_DIR)/GJK.cpp \
			$(SRC_DIR)/RigidBody.cpp \
			$(SRC_DIR)/World.cpp

# Rendering: needs SFML, linked into tinysim only. Keeping this out of
# LIB_SRCS is what lets the tests stay headless and SFML-free.
GFX_SRCS := $(SRC_DIR)/Renderer.cpp

MAIN_SRCS := $(SRC_DIR)/main.cpp

# Eyeball harness: its own main(), so it must stay out of TEST_SRCS.
VISUAL_SRCS := $(TEST_DIR)/visual_check.cpp
TEST_SRCS := $(TEST_DIR)/test_main.cpp \
				$(TEST_DIR)/test_rigidbody.cpp \
				$(TEST_DIR)/test_vec2.cpp \
				$(TEST_DIR)/test_world.cpp \
				$(TEST_DIR)/test_renderer.cpp \
				$(TEST_DIR)/test_geometry.cpp \
				$(TEST_DIR)/test_collision.cpp

CORE_HEADERS := include/Vec2.h \
				include/Transform.h \
				include/RigidBody.h \
				include/World.h \
				include/Shape.h \
				include/BoxShape.h \
				include/CircleShape.h \
				include/Simplex.h \
				include/Collision.h \
				include/GJK.h
GFX_HEADERS  := include/Renderer.h

PROG_BIN := tinysim
TEST_BIN := run_tests
VISUAL_BIN := visual_check

.PHONY: all build test visual clean

all: build

build: $(PROG_BIN)

$(PROG_BIN): $(LIB_SRCS) $(GFX_SRCS) $(MAIN_SRCS) $(CORE_HEADERS) $(GFX_HEADERS)
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) $(LIB_SRCS) $(GFX_SRCS) $(MAIN_SRCS) $(SFML_LIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(LIB_SRCS) $(GFX_SRCS) $(TEST_SRCS) $(CORE_HEADERS) $(GFX_HEADERS) $(TEST_DIR)/test_utils.h
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) $(LIB_SRCS) $(GFX_SRCS) $(TEST_SRCS) $(SFML_LIBS) -o $@

visual: $(VISUAL_BIN)
	./$(VISUAL_BIN)

$(VISUAL_BIN): $(LIB_SRCS) $(GFX_SRCS) $(VISUAL_SRCS) $(CORE_HEADERS) $(GFX_HEADERS)
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) $(LIB_SRCS) $(GFX_SRCS) $(VISUAL_SRCS) $(SFML_LIBS) -o $@

clean:
	rm -rf $(PROG_BIN) $(TEST_BIN) $(VISUAL_BIN) $(PROG_BIN).dSYM $(TEST_BIN).dSYM $(VISUAL_BIN).dSYM

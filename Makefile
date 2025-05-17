# Default: Pure Data include path (can be overridden)
PD_INCLUDE ?= /usr/include/pd

# Default extension
EXT ?= pd_linux

# Platform detection
UNAME := $(shell uname -s)
ARCH := $(shell uname -m)

# OS-specific flags
ifeq ($(UNAME),Linux)
    LINKFLAGS = -shared
    ifeq ($(ARCH),x86_64)
        # Linux x64
        CXXFLAGS_BASE = -Wall -Wextra -march=native -fPIC -I$(PD_INCLUDE)
    else ifeq ($(ARCH),armv7l)
        # Organelle M / Raspberry Pi 3
        CXXFLAGS_BASE = -Wall -Wextra -fPIC -I$(PD_INCLUDE)
    else
        # Fallback for unknown Linux arch
        CXXFLAGS_BASE = -Wall -Wextra -fPIC -I$(PD_INCLUDE)
    endif
endif

ifeq ($(UNAME),Darwin)
    PD_INCLUDE = /Applications/Pd.app/Contents/Resources/src
    LINKFLAGS = -dynamiclib -undefined suppress -flat_namespace
    EXT = pd_darwin
    CXXFLAGS_BASE = -Wall -Wextra -fPIC -I$(PD_INCLUDE)
endif

ifeq ($(OS),Windows_NT)
    PD_INCLUDE = ./pd/include
    LINKFLAGS = -shared
    EXT = pd_windows
    LIBS = -L. -lpd
    CXXFLAGS_BASE = -Wall -Wextra -I$(PD_INCLUDE)
endif

# === Compiler ===
CXX = g++

# === Project: grav ===
G_NAME = grav
G_SRC = grav.cpp Gravity.cpp GravityMath.cpp
G_OBJ = $(G_SRC:%.cpp=$(BUILD_DIR)/%.o)
G_TARGET = $(BUILD_DIR)/$(G_NAME).$(EXT)

# === Project: gravf ===
ANALYSE_NAME = gravf
ANALYSE_SRC = gravf.cpp
ANALYSE_OBJ = $(ANALYSE_SRC:%.cpp=$(BUILD_DIR)/%.o)
ANALYSE_TARGET = $(BUILD_DIR)/$(ANALYSE_NAME).$(EXT)

# === Build directory ===
BUILD_DIR = build

all: release

debug: CXXFLAGS = $(CXXFLAGS_BASE) -O0 -g
debug: $(BUILD_DIR) $(G_TARGET) $(ANALYSE_TARGET)

release: CXXFLAGS = $(CXXFLAGS_BASE) -O2
release: $(BUILD_DIR) $(G_TARGET) $(ANALYSE_TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# === Compile rules ===
$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# === Linking ===
$(G_TARGET): $(G_OBJ)
	$(CXX) $(LINKFLAGS) -o $@ $^

$(ANALYSE_TARGET): $(ANALYSE_OBJ)
	$(CXX) $(LINKFLAGS) -o $@ $^

$(G_TARGET): $(G_OBJ)
	$(CXX) $(LINKFLAGS) -o $@ $^ $(LIBS)

$(ANALYSE_TARGET): $(ANALYSE_OBJ)
	$(CXX) $(LINKFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean debug release
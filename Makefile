# Motion Control Makefile
# Wraps CMake build system for convenience

BUILD_DIR := build
CMAKE := cmake
CTEST := ctest

.PHONY: all build test coverage clean rebuild

all: build

# Configure and build with CMake
build:
	@mkdir -p $(BUILD_DIR)
	$(CMAKE) -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"
	$(CMAKE) --build $(BUILD_DIR) --parallel

# Run unit tests
test: build
	cd $(BUILD_DIR) && $(CTEST) --output-on-failure

# Generate coverage report (requires lcov/gcov)
coverage: test
	@mkdir -p $(BUILD_DIR)/coverage
	lcov --capture --directory $(BUILD_DIR) --output-file $(BUILD_DIR)/coverage/coverage.info --ignore-errors mismatch,empty || true
	lcov --remove $(BUILD_DIR)/coverage/coverage.info '/usr/*' '*/test/*' '*/_deps/*' --output-file $(BUILD_DIR)/coverage/coverage.info --ignore-errors unused,empty || true
	genhtml $(BUILD_DIR)/coverage/coverage.info --output-directory $(BUILD_DIR)/coverage/html --ignore-errors empty || true
	@echo "Coverage report: $(BUILD_DIR)/coverage/html/index.html"

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Full rebuild
rebuild: clean build

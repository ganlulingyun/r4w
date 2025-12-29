# CMakeLists.txt for R4W C/C++ Examples
#
# This demonstrates how to use R4W from C/C++ projects with CMake.
#
# Usage:
#   mkdir build && cd build
#   cmake ..
#   make
#
# The Rust library must be built first:
#   cargo build --release -p r4w-ffi

cmake_minimum_required(VERSION 3.16)
project(r4w_examples C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to R4W library root
set(R4W_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

# Path to generated header
set(R4W_INCLUDE_DIR "${R4W_ROOT}/crates/r4w-ffi/include")

# Path to built library
# Try release first, then debug
if(EXISTS "${R4W_ROOT}/target/release")
    set(R4W_LIBRARY_DIR "${R4W_ROOT}/target/release")
elseif(EXISTS "${R4W_ROOT}/target/debug")
    set(R4W_LIBRARY_DIR "${R4W_ROOT}/target/debug")
else()
    message(FATAL_ERROR "R4W library not found. Run 'cargo build --release -p r4w-ffi' first.")
endif()

# Find the library
find_library(R4W_LIBRARY
    NAMES r4w
    PATHS "${R4W_LIBRARY_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)

message(STATUS "R4W include dir: ${R4W_INCLUDE_DIR}")
message(STATUS "R4W library: ${R4W_LIBRARY}")

# Create imported target for R4W
add_library(r4w::r4w SHARED IMPORTED)
set_target_properties(r4w::r4w PROPERTIES
    IMPORTED_LOCATION "${R4W_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${R4W_INCLUDE_DIR}"
)

# Platform-specific dependencies
if(UNIX AND NOT APPLE)
    set(PLATFORM_LIBS m pthread dl)
elseif(APPLE)
    set(PLATFORM_LIBS m pthread)
elseif(WIN32)
    set(PLATFORM_LIBS ws2_32 userenv bcrypt ntdll)
endif()

# C Example: FFT Demo
add_executable(fft_demo fft_demo.c)
target_link_libraries(fft_demo PRIVATE
    r4w::r4w
    ${PLATFORM_LIBS}
)

# Enable warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(fft_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Custom target to build Rust library
add_custom_target(build_r4w
    COMMAND cargo build --release -p r4w-ffi
    WORKING_DIRECTORY "${R4W_ROOT}"
    COMMENT "Building R4W Rust library..."
)

# Set RPATH for finding library at runtime
set_target_properties(fft_demo PROPERTIES
    INSTALL_RPATH "${R4W_LIBRARY_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Optional: CTest integration
enable_testing()

add_test(NAME fft_demo_run
    COMMAND fft_demo
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# Set environment for test
set_tests_properties(fft_demo_run PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${R4W_LIBRARY_DIR}"
)

# Print usage instructions
message(STATUS "")
message(STATUS "Build R4W C Examples:")
message(STATUS "  1. Build the Rust library: cargo build --release -p r4w-ffi")
message(STATUS "  2. mkdir build && cd build")
message(STATUS "  3. cmake ..")
message(STATUS "  4. make")
message(STATUS "  5. ./fft_demo")
message(STATUS "")

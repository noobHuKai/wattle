cmake_minimum_required(VERSION 3.16)
project(WattleCxxExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Set up Rust library path
set(WATTLE_CXX_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../target/release")
set(WATTLE_CXX_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

# Include directories
include_directories(${WATTLE_CXX_INCLUDE_DIR})

# Add library search path
link_directories(${WATTLE_CXX_LIB_DIR})

# Create basic demo executable
add_executable(basic_demo basic_demo.cpp)
target_link_libraries(basic_demo wattle_cxx)

# Create advanced demo executable
add_executable(advanced_demo advanced_demo.cpp)
target_link_libraries(advanced_demo wattle_cxx)

# Set runtime library path (for Linux/macOS)
if(UNIX)
    set_target_properties(basic_demo PROPERTIES
        INSTALL_RPATH "${WATTLE_CXX_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
    set_target_properties(advanced_demo PROPERTIES
        INSTALL_RPATH "${WATTLE_CXX_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
endif()

# Custom target to build Rust library first
add_custom_target(build_rust_lib
    COMMAND cargo build --release
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    COMMENT "Building Rust Wattle CXX library"
)

# Make executables depend on Rust library
add_dependencies(basic_demo build_rust_lib)
add_dependencies(advanced_demo build_rust_lib)

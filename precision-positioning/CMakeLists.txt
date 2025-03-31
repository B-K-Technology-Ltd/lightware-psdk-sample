cmake_minimum_required(VERSION 3.10)

# Project Definition
project(positioner
        DESCRIPTION "Precision Positioning"
        LANGUAGES CXX C
        VERSION 1.0)

# Detect OS and Set Compiler & Flags
if(APPLE)
    message(STATUS "Configuring for macOS")

    set(CMAKE_C_COMPILER "/usr/bin/clang")
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

    set(CMAKE_C_FLAGS "-lpthread")
    set(CMAKE_CXX_FLAGS "-std=c++11 -lpthread")
    set(CMAKE_EXE_LINKER_FLAGS "-lpthread")

    add_definitions(-DSYSTEM_ARCH_MACOS=1)

elseif(UNIX)
    message(STATUS "Configuring for Linux")

    set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")

    set(CMAKE_C_FLAGS "-pthread -std=gnu99")
    set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
    set(CMAKE_EXE_LINKER_FLAGS "-pthread")

    add_definitions(-DSYSTEM_ARCH_LINUX=1 -D_GNU_SOURCE)
endif()

# Set Toolchain-Specific Library Paths
if(APPLE)
    set(TOOLCHAIN_NAME "macos")
    set(LIBRARY_EXTENSION "dylib")
elseif(UNIX)
    set(TOOLCHAIN_NAME "aarch64-linux-gnu-gcc")
    set(LIBRARY_EXTENSION "a")
endif()

# Convenience Variables
set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/sources)
set(LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/libraries)

# Define Executable
add_executable(${PROJECT_NAME} ${SRC_DIR}/main.cpp)

# Include Directories
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${LIB_DIR}/psdk_lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/osal)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/hal)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/application)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/lightware)

# Source Files
file(GLOB_RECURSE DJI_OSAL_SRC ${SRC_DIR}/dji/osal/*.c*)
file(GLOB_RECURSE DJI_HAL_SRC ${SRC_DIR}/dji/hal/*.c*)
file(GLOB_RECURSE DJI_APP_SRC ${SRC_DIR}/dji/application/*.c*)
file(GLOB_RECURSE LIGHTWARE_SRC ${SRC_DIR}/lightware/*.c*)

target_sources(${PROJECT_NAME} PRIVATE
        ${DJI_OSAL_SRC}
        ${DJI_HAL_SRC}
        ${DJI_APP_SRC}
        ${LIGHTWARE_SRC}
)

# Link PayloadSDK
target_link_libraries(${PROJECT_NAME} ${LIB_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.${LIBRARY_EXTENSION})

# Dependency Paths (macOS uses Homebrew, Linux uses standard paths)
if(APPLE)
    set(DEP_PATHS "/usr/local" "/opt/homebrew")
else()
    set(DEP_PATHS "/usr" "/usr/local")
endif()

# Find and Link OpenCV
find_package(OpenCV REQUIRED PATHS ${DEP_PATHS}/opt/opencv)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}")
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()

# Find and Link FFMPEG
#find_package(FFMPEG REQUIRED PATHS ${DEP_PATHS}/opt/ffmpeg)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale libswresample)

if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG: ${FFMPEG_LIBRARIES}")
    target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
else()
    message(STATUS "FFMPEG not found, video processing might be disabled.")
endif()

# Find and Link OPUS
#find_package(OPUS REQUIRED PATHS ${DEP_PATHS}/opt/opus)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPUS REQUIRED opus)

if (OPUS_FOUND)
    message(STATUS "Found OPUS: ${OPUS_LIBRARY}")
    target_link_libraries(${PROJECT_NAME} ${OPUS_LIBRARY})
else()
    message(STATUS "OPUS not found, audio features might be disabled.")
endif()

# Find and Link LIBUSB
#find_package(LIBUSB REQUIRED PATHS ${DEP_PATHS}/opt/libusb)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB: ${LIBUSB_LIBRARY}")
    target_link_libraries(${PROJECT_NAME} usb-1.0)
else()
    message(STATUS "LIBUSB not found, USB features might be disabled.")
endif()

# Set Executable Output Directory
if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif()

# Link Math Library
target_link_libraries(${PROJECT_NAME} m)

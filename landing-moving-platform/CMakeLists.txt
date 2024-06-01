cmake_minimum_required(VERSION 3.10)

# Project Def
project(lander
        DESCRIPTION "Moving Platform Lander"
        LANGUAGES CXX C
        VERSION 1.0)

set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
add_definitions(-D_GNU_SOURCE)

set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)

add_definitions(-DPLATFORM_ARCH_aarch64=1)
add_definitions(-DSYSTEM_ARCH_LINUX=1)


# Convenience
set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/sources)
set(LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/libraries)

# End Product
add_executable(${PROJECT_NAME} ${SRC_DIR}/main.cpp)

# Includes
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${LIB_DIR}/psdk_lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/osal)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/hal)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/dji/application)
target_include_directories(${PROJECT_NAME} PRIVATE ${SRC_DIR}/lightware)

# Sources
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

# Libraries

# PayloadSDK
target_link_libraries(${PROJECT_NAME} ${LIB_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a)
message(STATUS "Linking PayloadSDK: ${LIB_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a")

# PayloadSDK Dependencies
set(CMAKE_MODULE_PATH ${SRC_DIR}/dji/3rdparty)

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

find_package(FFMPEG REQUIRED)
if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_output
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line ${ffmpeg_version_output})
    string(REGEX MATCH " .* " ffmpeg_version ${ffmpeg_version_line})
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version ${ffmpeg_version})

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()

    target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
    include_directories(${FFMPEG_INCLUDE_DIR})
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)

include_directories(${CMAKE_CURRENT_LIST_DIR}/libraries/psdk_lib/include)


find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
    target_link_libraries(${PROJECT_NAME} /usr/local/lib/libopus.a)
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
    target_link_libraries(${PROJECT_NAME} usb-1.0)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

target_link_libraries(${PROJECT_NAME} m)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()

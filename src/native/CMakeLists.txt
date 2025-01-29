cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
	message(FATAL_ERROR "In-source builds are not supported. Please choose a different binary directory.")
	return()
endif()

# Determine if engine is built as a subproject (using add_subdirectory)
# or if it is the master project.
if (NOT DEFINED ALIMER_MASTER_PROJECT)
    set(ALIMER_MASTER_PROJECT OFF)
    if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(ALIMER_MASTER_PROJECT ON)
        message(STATUS "CMake version: ${CMAKE_VERSION}")
    endif ()
endif ()

if (MSVC AND ALIMER_MASTER_PROJECT)
    if (NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
        set (CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()

# Use solution folders to organize projects
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Configure CMake global variables
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_MESSAGE LAZY)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_DEBUG_POSTFIX "")

set(CMAKE_OSX_DEPLOYMENT_TARGET "11" CACHE STRING "Minimum OS X deployment version")
set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE STRING "Build architectures for OS X")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Build version
set(ALIMER_VERSION_MAJOR 1)
set(ALIMER_VERSION_MINOR 0)
set(ALIMER_VERSION_PATCH 0)
set(ALIMER_VERSION ${ALIMER_VERSION_MAJOR}.${ALIMER_VERSION_MINOR}.${ALIMER_VERSION_PATCH})

# Project declaration
project(alimer_physics LANGUAGES CXX C)

# Set the default CMAKE_BUILD_TYPE to Release.
# This should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if (ALIMER_MASTER_PROJECT AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The default build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif ()

# CPM
include(cmake/CPM.cmake)

# Setting Platform
if(EMSCRIPTEN)
    set(ALIMER_PLATFORM_NAME "Web")
elseif(ANDROID)
    set(ALIMER_PLATFORM_NAME "Android")
elseif(IOS)
    set(ALIMER_PLATFORM_NAME "iOS")
elseif(APPLE)
    set(ALIMER_PLATFORM_NAME "macOS")
elseif(WINDOWS_STORE)
    set(ALIMER_PLATFORM_NAME "UWP")
elseif(WIN32)
    set(ALIMER_PLATFORM_NAME "Win32")
elseif(UNIX)
    set(ALIMER_PLATFORM_NAME "Unix")
    set(ALIMER_LINUX_WSI_SELECTION "XCB" CACHE STRING "Select WSI target (XCB, XLIB, WAYLAND, D2D)")
else()
    message(FATAL_ERROR "Unrecognized platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Setting Architecture
if (DEFINED VCPKG_TARGET_ARCHITECTURE)
    set(ALIMER_ARCH ${VCPKG_TARGET_ARCHITECTURE})
elseif(CMAKE_GENERATOR_PLATFORM MATCHES "^[Ww][Ii][Nn]32$")
    set(ALIMER_ARCH x86)
elseif(CMAKE_GENERATOR_PLATFORM MATCHES "^[Xx]64$")
    set(ALIMER_ARCH x64)
elseif(CMAKE_GENERATOR_PLATFORM MATCHES "^[Aa][Rr][Mm]$")
    set(ALIMER_ARCH arm)
elseif(CMAKE_GENERATOR_PLATFORM MATCHES "^[Aa][Rr][Mm]64$")
    set(ALIMER_ARCH arm64)
endif()

# Options
if (IOS OR EMSCRIPTEN)
    set(ALIMER_SHARED_LIBRARY OFF CACHE BOOL "Disable shared library on (IOS, WEB)" FORCE)
elseif(ANDROID)
    set(ALIMER_SHARED_LIBRARY ON CACHE BOOL "Always enable shared library on (ANDROID)" FORCE)    
else()
    option(ALIMER_SHARED_LIBRARY "Build as shared library" ON)
endif ()

if (ALIMER_SHARED_LIBRARY)
    set(LIBRARY_TYPE SHARED)
    message(STATUS "  Library         SHARED")
else ()
    set(LIBRARY_TYPE STATIC)
    message(STATUS "  Library         STATIC")
endif ()

CPMAddPackage(
        NAME JoltPhysics
        GITHUB_REPOSITORY jrouwe/JoltPhysics
        GIT_TAG 15561f849140c06693b34fd16dd87b503ab155d0
        SOURCE_SUBDIR Build
        OPTIONS
            "OBJECT_LAYER_BITS 16"
            "DOUBLE_PRECISION OFF"
            "GENERATE_DEBUG_SYMBOLS OFF"
            "CROSS_PLATFORM_DETERMINISTIC OFF"
            "ENABLE_INSTALL OFF"
            "ENABLE_ALL_WARNINGS OFF"
            "CPP_EXCEPTIONS_ENABLED OFF"
            "CPP_RTTI_ENABLED OFF"
            "USE_STATIC_MSVC_RUNTIME_LIBRARY OFF"
            "ENABLE_OBJECT_STREAM OFF"
    )
    set_property(TARGET Jolt PROPERTY FOLDER "ThirdParty")

# Define target name
set (TARGET_NAME alimer_physics)

set(SOURCE_FILES
    include/alimer_physics.h
    src/alimer_internal.h
    src/alimer_internal.cpp
    src/alimer_physics.cpp
)

add_library(${TARGET_NAME} ${LIBRARY_TYPE} ${SOURCE_FILES})

target_compile_definitions (${TARGET_NAME} PRIVATE ALIMER_IMPLEMENTATION=1)
if (ALIMER_SHARED_LIBRARY)
    target_compile_definitions (${TARGET_NAME} PRIVATE ALIMER_SHARED_LIBRARY=1)
endif ()

target_include_directories(${TARGET_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${TARGET_NAME} PRIVATE Jolt)

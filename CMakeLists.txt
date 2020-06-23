project(MOTIONDETECTION)

cmake_minimum_required(VERSION 3.5)
# -> OpenCV
find_package(OpenCV REQUIRED)




# Based on : https://github.com/arnavb/cpp14-project-template/blob/master/CMakeLists.txt
#      and : https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

# Version (should be updated on release)
set(MOTIONDETECTION_VERSION_MAJOR 1)
set(MOTIONDETECTION_VERSION_MINOR 0)
set(MOTIONDETECTION_VERSION_PATCH 0)
set(MOTIONDETECTION_VERSION "${MOTIONDETECTION_VERSION_MAJOR}.${MOTIONDETECTION_VERSION_MINOR}.${MOTIONDETECTION_VERSION_PATCH}")

# Set project name
set(PROJ_NAME "\"MOTIONDETECTION\"") # PROJECT_NAME is a predefined variable
set(PROJECT_DESCRIPTION "\"Using CV algorithm to detect moving objects in scene\"")

# Set C++ standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
# Using set instead of options as option requires to change the cache
option(BUILD_TESTS "Build test binary testrunner" ON)

option(GEN_DOCS "Generate documentation" OFF)

# Build type
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as no build type was specified")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type (Debug/Release)" FORCE)
endif (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

# Compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
else(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "No compatible compiler selected for this project")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

# Define output dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Define targets to build
# -> Libs
add_library(motionDetectionLib SHARED
        src/motionDetection.cpp
        src/imageUtils.cpp
        test/testClasses.cpp
        src/configParser.cpp
        src/signalHandler.cpp
        src/customDataTypes.cpp
        ext/NMS/nms.cpp)
# (Rewrite library output name to match linux naming conventions)
set_target_properties(motionDetectionLib PROPERTIES SOVERSION "${MOTIONDETECTION_VERSION}")
set_target_properties(motionDetectionLib PROPERTIES LIBRARY_OUTPUT_NAME 
    motiondetection)


# Linker Language for targets
set_target_properties(motionDetectionLib PROPERTIES LINKER_LANGUAGE CXX)



# Includes
# -> Base of the source tree
# -> OpenCV include
target_include_directories(motionDetectionLib
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        )
# -> General include
target_include_directories(motionDetectionLib
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        )
# -> inih
target_include_directories(motionDetectionLib
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/inih-headeronly/
)



# -> OpenCV linking
target_link_libraries(motionDetectionLib ${OpenCV_LIBS})

# Unit tests
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif (BUILD_TESTS)

# Documentation
if (GEN_DOCS)
    add_subdirectory(doc)
endif (GEN_DOCS)
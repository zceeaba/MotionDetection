# Based on : https://github.com/arnavb/cpp14-project-template/blob/master/test/CMakeLists.txt

# Dependencies
# -> Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Sources
set(TEST_SOURCES
        motionDetectionTest.cpp
        )

# Libraries
add_library(motionDetectionTest INTERFACE)


# Includes
target_include_directories(motionDetectionTest
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
        )

# Executables to build
add_executable(motionDetectionTestRunner _testRunner.cpp ${TEST_SOURCES})
target_link_libraries(motionDetectionTestRunner motionDetectionLib motionDetectionTest Threads::Threads)

add_test(all_tests motionDetectionTestRunner)
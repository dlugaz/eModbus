cmake_minimum_required(VERSION 3.14)
project(eModbusProject)
set(CMAKE_CXX_STANDARD 20)

add_library(eModbus
        ./source/ModbusMasterBase.cpp
        )

target_include_directories(eModbus PUBLIC include)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

# 3. Create the test executable
add_executable(eModbus_tests
        tests/test_FrameView.cpp
        tests/test_Frame.cpp
)

# 4. Link library and GTest
target_link_libraries(eModbus_tests
        PRIVATE
        eModbus
        GTest::gtest_main
)

# 5. Enable test discovery for CLion
include(GoogleTest)
gtest_discover_tests(eModbus_tests)
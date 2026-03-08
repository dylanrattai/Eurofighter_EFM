cmake_minimum_required(VERSION 3.14)
project(your_project)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

add_executable(FCS_tests
    tests/FCS_tests.cpp
    FCS.cpp
    State.cpp
    Input.cpp
    Engine.cpp
    Airframe.cpp
    Actuators.cpp
)

target_include_directories(FCS_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

target_link_libraries(FCS_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(FCS_tests)

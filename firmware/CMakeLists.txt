cmake_minimum_required(VERSION 3.20)
project(foc2_portable_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_library(foc2_core
    src/feedback_mode.cpp
    src/safety_state.cpp
)
target_include_directories(foc2_core PUBLIC include)
target_compile_options(foc2_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(foc2_core_tests
    tests/test_core.cpp
)
target_link_libraries(foc2_core_tests PRIVATE foc2_core)
target_compile_options(foc2_core_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME foc2_core_tests COMMAND foc2_core_tests)

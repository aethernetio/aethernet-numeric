# Copyright 2025 Aethernet Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(test-ae-numeric LANGUAGES CXX)

include(../cmake/CPM.cmake)

# add Unity with support for double precision
add_compile_definitions(UNITY_INCLUDE_DOUBLE=1)
if (CMAKE_COLOR_DIAGNOSTICS)
  add_compile_definitions(UNITY_OUTPUT_COLOR=1)
endif()
CPMAddPackage("https://github.com/ThrowTheSwitch/Unity.git#master")

option(AE_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer for tests" OFF)
option(AE_ENABLE_COVERAGE
  "Build a separate gcov target for integer_math (AE_COVERAGE_BUILD)"
  OFF)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
  main.cpp
  test-tiered-int.cpp
  test-tiered-int-view.cpp
  test-fixed-point.cpp
  test-exponential.cpp
  test-exponential-arithmetic.cpp
  test-exponential-math-policy.cpp
  test-exponential-floating-runtime.cpp
  test-text-io.cpp
  test-ostream-io.cpp
  test-wire-io.cpp
  test-packed-ring.cpp
  test-composed-types.cpp
  test-composed-exponential.cpp
  test-composed-exponential-tiered.cpp
  test-fixed-math.cpp
  test-integer-math.cpp
  test-tiered-int-aether-compat.cpp
  test-type-size.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE ae-numeric unity)

# enable warnings and werror
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror>
  $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /MP /constexpr:steps16777216 /wd4530 /wd4702 /wd4127>
)


if(AE_ENABLE_UBSAN)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Prefer trap-on-error so MinGW builds do not require libubsan.
    target_compile_options(${PROJECT_NAME} PRIVATE
      -fsanitize=undefined -fsanitize-undefined-trap-on-error)
    target_link_options(${PROJECT_NAME} PRIVATE
      -fsanitize=undefined -fsanitize-undefined-trap-on-error)
  else()
    message(WARNING "AE_ENABLE_UBSAN ignored: unsupported compiler")
  endif()
endif()

if(AE_ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Full suite gets gcov arcs (constexpr helpers stay constexpr here).
    target_compile_options(${PROJECT_NAME} PRIVATE --coverage -O0 -g)
    target_link_options(${PROJECT_NAME} PRIVATE --coverage)

    # Isolated target: AE_COVERAGE_BUILD strips constexpr only here so gcov can
    # instrument integer_math.h without breaking constexpr fixed_math callers.
    add_executable(test-integer-math-coverage
      test-integer-math-coverage-main.cpp
      test-integer-math.cpp)
    target_link_libraries(test-integer-math-coverage PRIVATE ae-numeric unity)
    target_compile_definitions(test-integer-math-coverage PRIVATE AE_COVERAGE_BUILD=1)
    target_compile_options(test-integer-math-coverage PRIVATE
      --coverage -O0 -g
      $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror>
      $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror>)
    target_link_options(test-integer-math-coverage PRIVATE --coverage)
    add_test(NAME test-integer-math-coverage
      COMMAND $<TARGET_FILE:test-integer-math-coverage>)
  else()
    message(WARNING "AE_ENABLE_COVERAGE ignored: unsupported compiler")
  endif()
endif()

add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

# Compile-fail tests: each target must fail to compile. They are excluded from
# the default build; ctest builds them on demand and treats a build failure as
# success (WILL_FAIL).
foreach(fail_case
    construct_out_of_range
    add_out_of_range
    exp_construct_out_of_range
    exp_float_runtime
    wide_mul_intermediate
    tiered_add_overflow
    tiered_add_underflow)
  add_executable(fail-${fail_case} EXCLUDE_FROM_ALL
    compile-fail/${fail_case}.cpp)
  target_link_libraries(fail-${fail_case} PRIVATE ae-numeric)
  add_test(NAME fail-${fail_case}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
      --target fail-${fail_case} --config $<CONFIG>)
  set_tests_properties(fail-${fail_case} PROPERTIES WILL_FAIL TRUE)
endforeach()

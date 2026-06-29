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

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(test-numeric LANGUAGES CXX)

include(../cmake/CPM.cmake)

# add Unity with support for double precision
add_compile_definitions(UNITY_INCLUDE_DOUBLE=1)
if (CMAKE_COLOR_DIAGNOSTICS)
  add_compile_definitions(UNITY_OUTPUT_COLOR=1)
endif()
CPMAddPackage("https://github.com/ThrowTheSwitch/Unity.git#master")

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
  test-type-size.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE numeric unity)

# enable warnings and werror
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror>
  $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /MP /constexpr:steps16777216 /wd4530 /wd4702 /wd4127>
)

add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

# Compile-fail tests: each target must fail to compile. They are excluded from
# the default build; ctest builds them on demand and treats a build failure as
# success (WILL_FAIL).
foreach(fail_case
    construct_out_of_range
    add_out_of_range
    exp_construct_out_of_range
    exp_float_runtime)
  add_executable(fail-${fail_case} EXCLUDE_FROM_ALL
    compile-fail/${fail_case}.cpp)
  target_link_libraries(fail-${fail_case} PRIVATE numeric)
  add_test(NAME fail-${fail_case}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
      --target fail-${fail_case} --config $<CONFIG>)
  set_tests_properties(fail-${fail_case} PROPERTIES WILL_FAIL TRUE)
endforeach()

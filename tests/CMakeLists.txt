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
  test-cyclic-counter.cpp
  test-composed-types.cpp
  test-composed-exponential.cpp
  test-composed-exponential-tiered.cpp
  test-segmented-number-core.cpp
  test-segmented-number-formats.cpp
  test-segmented-number-wire.cpp
  test-segmented-number-formula-lookup.cpp
  test-segmented-number-floating-runtime.cpp
  test-segmented-number-size.cpp
  test-fixed-math.cpp
  test-integer-math.cpp
  test-tiered-int-aether-compat.cpp
  test-type-size.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE ae-numeric unity)

# enable warnings and werror
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror>
  $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror -fconstexpr-ops-limit=268435456 -fconstexpr-loop-limit=1048576>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /MP /constexpr:steps100000000 /wd4530 /wd4702 /wd4127>
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
    tiered_add_underflow
    segmented_overlap
    segmented_gap
    segmented_duplicate_boundary
    segmented_exponential_non_positive
    segmented_too_many_one_byte_codes
    segmented_impossible_error
    segmented_runtime_rep_too_small
    segmented_invalid_wire_bytes
    segmented_duplicate_runtime_value
    segmented_impossible_continuous_step)
  add_executable(fail-${fail_case} EXCLUDE_FROM_ALL
    compile-fail/${fail_case}.cpp)
  target_link_libraries(fail-${fail_case} PRIVATE ae-numeric)
  add_test(NAME fail-${fail_case}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
      --target fail-${fail_case} --config $<CONFIG>)
  set_tests_properties(fail-${fail_case} PROPERTIES WILL_FAIL TRUE)
endforeach()

function(ae_numeric_add_footprint tgt)
  add_executable(${tgt} EXCLUDE_FROM_ALL footprint/segmented_footprint.cpp)
  target_link_libraries(${tgt} PRIVATE ae-numeric)
  target_compile_definitions(${tgt} PRIVATE ${ARGN})
  target_compile_options(${tgt} PRIVATE
    $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror>
    $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror -fconstexpr-ops-limit=268435456 -fconstexpr-loop-limit=1048576 -ffunction-sections -fdata-sections>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /MP /constexpr:steps100000000 /Gy /wd4530 /wd4702 /wd4127>
  )
  target_link_options(${tgt} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wl,--gc-sections>
    $<$<CXX_COMPILER_ID:Clang>:-Wl,--gc-sections>
    $<$<CXX_COMPILER_ID:MSVC>:/OPT:REF>
  )
endfunction()

ae_numeric_add_footprint(footprint-rssi AE_SEG_HAS_RSSI=1)
ae_numeric_add_footprint(footprint-temperature AE_SEG_HAS_TEMP=1)
ae_numeric_add_footprint(footprint-humidity AE_SEG_HAS_HUM=1)
ae_numeric_add_footprint(footprint-co2 AE_SEG_HAS_CO2=1)
ae_numeric_add_footprint(footprint-rx-window AE_SEG_HAS_RX=1)
ae_numeric_add_footprint(footprint-battery AE_SEG_HAS_BAT=1)
ae_numeric_add_footprint(footprint-connect-duration AE_SEG_HAS_CONN=1)
ae_numeric_add_footprint(footprint-all-formula AE_SEG_FOOTPRINT_ALL=1)
ae_numeric_add_footprint(footprint-rssi-lookup AE_SEG_HAS_RSSI_LOOKUP=1)
ae_numeric_add_footprint(footprint-temperature-lookup AE_SEG_HAS_TEMP_LOOKUP=1)
ae_numeric_add_footprint(footprint-all-lookup AE_SEG_FOOTPRINT_ALL_LOOKUP=1)

add_custom_target(segmented-footprint DEPENDS
  footprint-rssi
  footprint-temperature
  footprint-humidity
  footprint-co2
  footprint-rx-window
  footprint-battery
  footprint-connect-duration
  footprint-all-formula
  footprint-rssi-lookup
  footprint-temperature-lookup
  footprint-all-lookup
)

# Minimal object-only CyclicCounter footprint probes (no CRT link).
function(ae_numeric_add_cyclic_fp_obj tgt src)
  add_library(${tgt} OBJECT EXCLUDE_FROM_ALL ${src})
  target_link_libraries(${tgt} PRIVATE ae-numeric)
  target_include_directories(${tgt} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/footprint)
  target_compile_options(${tgt} PRIVATE
    $<$<CXX_COMPILER_ID:Clang>: -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti>
    $<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /Gy /GR- /EHs-c- /wd4530>
  )
endfunction()

ae_numeric_add_cyclic_fp_obj(fp-obj-cyclic-u8-u16 footprint/minimal_cyclic_u8_u16.cpp)
ae_numeric_add_cyclic_fp_obj(fp-obj-cyclic-u8-u32 footprint/minimal_cyclic_u8_u32.cpp)
ae_numeric_add_cyclic_fp_obj(fp-obj-cyclic-u16-u32 footprint/minimal_cyclic_u16_u32.cpp)

add_custom_target(cyclic-counter-footprint-obj DEPENDS
  fp-obj-cyclic-u8-u16
  fp-obj-cyclic-u8-u32
  fp-obj-cyclic-u16-u32
)

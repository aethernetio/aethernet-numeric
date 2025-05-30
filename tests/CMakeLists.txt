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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(test-numeric LANGUAGES CXX)

# enable doubles in unity tests
add_compile_definitions("UNITY_INCLUDE_DOUBLE")
if(NOT TARGET unity)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../third_party/Unity" Unity)
endif()

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
  main.cpp
  test-tiered-int.cpp
  test-fixed-point.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE numeric)
target_link_libraries(${PROJECT_NAME} PRIVATE unity)

add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

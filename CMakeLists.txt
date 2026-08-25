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

project(ae-numeric CXX)

# define is the project is the current root project
# root project builds tests and defines installation targets by default
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
  set(IS_ROOT_PROJECT ON)
else()
  set(IS_ROOT_PROJECT OFF)
endif()

option(AE_BUILD_TESTS "Build tests" ${IS_ROOT_PROJECT} )
option(AE_NUMERIC_INSTALL "Install ae-numeric library" ${IS_ROOT_PROJECT} )

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/CPM.cmake)
CPMAddPackage(URI "https://github.com/aethernetio/gcem.git#master" EXCLUDE_FROM_ALL FALSE)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_link_libraries(${PROJECT_NAME} INTERFACE gcem)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (AE_NUMERIC_INSTALL)
  include(GNUInstallDirs)

  # define the installation target, install lib headers and export cmake config
  install(TARGETS ${PROJECT_NAME}
          EXPORT ${PROJECT_NAME}Targets
  )

  install(DIRECTORY ae-numeric DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  install(EXPORT ${PROJECT_NAME}Targets
          FILE ${PROJECT_NAME}Targets.cmake
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
endif()

if ( AE_BUILD_TESTS )
  enable_testing()
  add_subdirectory("tests")
endif()

# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.15)

project(telemetry-recorder VERSION 0.0.1 LANGUAGES CXX)


find_package(libhal REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)
find_package(libhal-icm REQUIRED CONFIG)
find_package(libhal-neo REQUIRED CONFIG)
find_package(libhal-mpl REQUIRED CONFIG)


add_library(telemetry-recorder src/telemetry-recorder.cpp)

target_include_directories(telemetry-recorder PUBLIC include
)

target_compile_features(telemetry-recorder PRIVATE cxx_std_20)
target_compile_options(telemetry-recorder PRIVATE -Wall -Wextra)
target_link_libraries(telemetry-recorder PRIVATE
  libhal::libhal
  libhal::util
  libhal::icm
  libhal::neo
  libhal::mpl
)

if(NOT BUILD_TESTING STREQUAL OFF)
  add_subdirectory(tests)
endif()

install(TARGETS telemetry-recorder)

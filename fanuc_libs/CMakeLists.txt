# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_libs VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Build the sockpp library
include(FetchContent)
FetchContent_Declare(
    sockpp
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/sockpp
    PATCH_COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/apply_patch.bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/sockpp.patch
)
set(SOCKPP_BUILD_SHARED OFF CACHE BOOL "Build sockpp as a shared library")
set(SOCKPP_BUILD_STATIC ON CACHE BOOL "Build sockpp as a static library")
FetchContent_MakeAvailable(sockpp)
set_target_properties(sockpp-static PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Create readerwriterqueue target
FetchContent_Declare(
    readerwriterqueue
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/readerwriterqueue
    PATCH_COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/apply_patch.bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/readerwriterqueue.patch
)
FetchContent_MakeAvailable(readerwriterqueue)

FetchContent_Declare(
  yaml-cpp
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/yaml-cpp
)
FetchContent_MakeAvailable(yaml-cpp)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "Build yaml-cpp shared library")
set(YAML_CPP_INSTALL OFF CACHE BOOL "Enable generation of yaml-cpp install targets")

# Build the reflectcpp library
FetchContent_Declare(
    reflect-cpp
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/reflect-cpp
    PATCH_COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/apply_patch.bash ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/reflectcpp.patch
)
set(REFLECTCPP_BUILD_SHARED OFF CACHE BOOL "Build reflectcpp as a shared library")
set(REFLECTCPP_YAML ON CACHE BOOL "Enable YAML support")
FetchContent_MakeAvailable(reflect-cpp)
set_target_properties(reflectcpp PROPERTIES CXX_STANDARD 20)
set_target_properties(reflectcpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest." FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "Builds the googlemock subproject." FORCE)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
  enable_testing()
endif ()

add_subdirectory(stream_motion)
add_subdirectory(rmi)
add_subdirectory(fanuc_client)
add_subdirectory(gpio_config)

if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif ()

# Create cmake config and version files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfigVersion.cmake
    VERSION
    ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    ARCH_INDEPENDENT
)
configure_package_config_file(fanuc_libsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfig.cmake
    INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs/
)

# --- Install ---
include(GNUInstallDirs)
install(TARGETS stream_motion rmi fanuc_client gpio_config
    EXPORT fanuc_libsExports
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT fanuc_libsExports
    FILE
    fanuc_libsTargets.cmake
    NAMESPACE
    fanuc_libs::
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs
)
install(DIRECTORY
    rmi/include/
    stream_motion/include/
    fanuc_client/include/
    gpio_config/include/
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfigVersion.cmake
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs
)

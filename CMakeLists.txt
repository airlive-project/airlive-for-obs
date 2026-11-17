# airlive-obs — the Windows build. (macOS builds with scripts/macos/build-macos.sh, which bundles
# its own FFmpeg and signs/notarizes; this file is not used there.)
#
# What it needs, all fetched by .github/workflows/windows.yml (the steps there are the recipe):
#   OBS_SOURCE_DIR   an obs-studio 30.2.3 source tree, for the libobs headers. Build against the
#                    OLDEST OBS we support, never the newest: OBS refuses a module that reports a
#                    newer libobs than its own (see scripts/macos/build-macos.sh).
#   OBS_IMPORT_LIB   obs.lib, made from OBS 30.2.3's obs.dll (the plugin links obs.dll at runtime).
#   FFmpeg           avcodec + swscale from vcpkg, STATIC (triplet x64-windows-static-md), so the
#                    plugin carries its own decoder and never depends on which FFmpeg DLLs a given
#                    OBS version ships.

cmake_minimum_required(VERSION 3.20)
project(airlive-obs VERSION 1.4.0 LANGUAGES C CXX)

if(NOT WIN32)
  message(FATAL_ERROR "This CMake build is for Windows. On macOS run scripts/macos/build-macos.sh.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OBS_SOURCE_DIR "" CACHE PATH "obs-studio 30.2.3 source tree (libobs headers)")
set(OBS_IMPORT_LIB "" CACHE FILEPATH "obs.lib made from OBS 30.2.3's obs.dll")
if(NOT EXISTS "${OBS_SOURCE_DIR}/libobs/obs-module.h")
  message(FATAL_ERROR "OBS_SOURCE_DIR must point at an obs-studio 30.2.3 source tree")
endif()
if(NOT EXISTS "${OBS_IMPORT_LIB}")
  message(FATAL_ERROR "OBS_IMPORT_LIB must point at obs.lib")
endif()

# vcpkg's ffmpeg port ships FindFFMPEG.cmake in its share folder: FFMPEG_INCLUDE_DIRS and
# FFMPEG_LIBRARIES, the static libraries together with the system libraries they need.
set(AIRLIVE_VCPKG_PREFIX "" CACHE PATH "vcpkg's installed/<triplet> folder holding FFmpeg")
file(TO_CMAKE_PATH "${AIRLIVE_VCPKG_PREFIX}" AIRLIVE_VCPKG_PREFIX)
list(APPEND CMAKE_MODULE_PATH "${AIRLIVE_VCPKG_PREFIX}/share/ffmpeg")
list(APPEND CMAKE_PREFIX_PATH "${AIRLIVE_VCPKG_PREFIX}")
find_package(FFMPEG REQUIRED)

file(GLOB AIRLIVE_SHARED_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB AIRLIVE_WINDOWS_SOURCES CONFIGURE_DEPENDS src/platform/windows/*.cpp)
add_library(${PROJECT_NAME} MODULE ${AIRLIVE_SHARED_SOURCES} ${AIRLIVE_WINDOWS_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
  src
  "${OBS_SOURCE_DIR}/libobs"
  cmake/windows            # obsconfig.h, which the libobs headers expect next to them
  ${FFMPEG_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE
  NOMINMAX                 # windows.h must not turn std::min / std::max into macros
  WIN32_LEAN_AND_MEAN
  _CRT_SECURE_NO_WARNINGS)
target_compile_options(${PROJECT_NAME} PRIVATE /utf-8 /permissive- /W3)

target_link_libraries(${PROJECT_NAME} PRIVATE
  "${OBS_IMPORT_LIB}"
  ${FFMPEG_LIBRARIES}
  ws2_32                   # sockets
  dnsapi                   # mDNS advert (DnsServiceRegister)
  bcrypt                   # HMAC + random for the receiver password
  advapi32)                # Credential Manager

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

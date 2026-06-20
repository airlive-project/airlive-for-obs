# airlive-obs — CMake build.
#
# Pragmatic, self-contained build that finds libobs and FFmpeg. It also works
# when dropped into obsproject/obs-plugintemplate (which provides its own
# helpers); in that case prefer the template's CMake and just add src/*.cpp.

# Plugin binary is "airlive-obs" — kept distinct from any older Airlive
# screen-mirroring plugin file already on the machine.
cmake_minimum_required(VERSION 3.16)
project(airlive-obs VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- libobs ---------------------------------------------------------------
# Provided by an OBS dev install (libobsConfig.cmake) or the plugin template.
find_package(libobs REQUIRED)

# --- FFmpeg (libavcodec / libavutil / libswscale) -------------------------
# macOS/Linux ship pkg-config with their dev toolchains. Windows/MSVC does not,
# so find_package(PkgConfig REQUIRED) hard-fails the configure step there. On
# Windows, build inside obsproject/obs-plugintemplate (which vendors FFmpeg) or
# provide FFmpeg via vcpkg (find_package(FFMPEG) below picks up its targets).
if(NOT WIN32)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavcodec
    libavutil
    libswscale)
  set(AIRLIVE_FFMPEG_LIB PkgConfig::FFMPEG)
else()
  # vcpkg's ffmpeg port exposes a FFMPEG find module (FFMPEG_INCLUDE_DIRS /
  # FFMPEG_LIBRARIES). Requires -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake.
  find_package(FFMPEG REQUIRED)
  add_library(airlive_ffmpeg INTERFACE)
  target_include_directories(airlive_ffmpeg INTERFACE ${FFMPEG_INCLUDE_DIRS})
  target_link_libraries(airlive_ffmpeg INTERFACE ${FFMPEG_LIBRARIES})
  set(AIRLIVE_FFMPEG_LIB airlive_ffmpeg)
endif()

add_library(${PROJECT_NAME} MODULE
  src/packet-parser.cpp
  src/h264-decoder.cpp
  src/bonjour-service.cpp
  src/airlive-connection.cpp
  src/airlive-source.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  OBS::libobs
  ${AIRLIVE_FFMPEG_LIB})

# --- Bonjour / mDNS per platform ------------------------------------------
if(APPLE)
  # dns_sd is part of the system frameworks; no extra link needed.
elseif(WIN32)
  # Bonjour SDK for Windows: provides dns_sd.h + dnssd.lib.
  target_link_libraries(${PROJECT_NAME} PRIVATE dnssd ws2_32)
else() # Linux
  # avahi-compat-libdns_sd exposes the same dns_sd.h API over Avahi.
  pkg_check_modules(DNSSD REQUIRED IMPORTED_TARGET avahi-compat-libdns_sd)
  target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::DNSSD)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

# Install rules differ per packaging target; the plugin template handles this.
# For a manual install, copy the built module + data/ into your OBS plugins dir.

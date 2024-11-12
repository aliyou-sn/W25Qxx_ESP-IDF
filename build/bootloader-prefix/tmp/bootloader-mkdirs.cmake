# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/apple/Desktop/v5.1-rc2/esp-idf/components/bootloader/subproject"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/tmp"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/src"
  "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/apple/Desktop/audio_playback_system/file_upload/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

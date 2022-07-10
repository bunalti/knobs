# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/kerem/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/kerem/knobs/build/bootloader"
  "C:/Users/kerem/knobs/build/bootloader-prefix"
  "C:/Users/kerem/knobs/build/bootloader-prefix/tmp"
  "C:/Users/kerem/knobs/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/kerem/knobs/build/bootloader-prefix/src"
  "C:/Users/kerem/knobs/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/kerem/knobs/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()

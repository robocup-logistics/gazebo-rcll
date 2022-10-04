# ***************************************************************************
# Created:   Mon  3 May 14:08:58 CEST 2021
#
# Copyright  2021  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
# ****************************************************************************/
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Library General Public License for more
# details.
#
# Read the full text in the LICENSE.md file.
#

find_package(gazebo REQUIRED)

if(gazebo_VERSION_MAJOR LESS_EQUAL 11)
  list(APPEND GAZEBO_CFLAGS -DTBB_SUPPRESS_DEPRECATED_MESSAGES
       -DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

if(gazebo_VERSION_MAJOR LESS_EQUAL 9)
  # Gazebo on Ubuntu Focal produces this warning, disable it.
  list(APPEND GAZEBO_CFLAGS -Wno-deprecated-declarations)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION
                                            GREATER_EQUAL 11)
  list(APPEND GAZEBO_CFLAGS -Wno-range-loop-construct -Wno-deprecated-declarations)
endif()

include_directories(libs)
add_subdirectory(libs)
add_subdirectory(plugins)

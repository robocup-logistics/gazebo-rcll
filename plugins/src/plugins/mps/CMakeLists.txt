# ***************************************************************************
# Created:   Mon  3 May 15:57:37 CEST 2021
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

add_library(
  mps SHARED
  mps.cpp
  mps_loader.cpp
  base_station.cpp
  ring_station.cpp
  cap_station.cpp
  delivery_station.cpp
  storage_station.cpp)

target_link_libraries(
  mps
  PUBLIC core
         configurable
         gazsim_msgs
         gazebo
         opcuacore
         opcuaserver
         opcuaprotocol)
target_include_directories(mps PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_compile_options(mps PUBLIC ${GAZEBO_CFLAGS})

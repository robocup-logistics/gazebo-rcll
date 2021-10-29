# ***************************************************************************
# Created:   Mon  3 May 15:26:16 CEST 2021
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

find_package(Boost REQUIRED COMPONENTS system)

add_library(conveyor_vision SHARED conveyor_vision.cpp)
target_link_libraries(conveyor_vision PUBLIC gazebo mps llsf_msgs configurable core
                                             Boost::system)
target_include_directories(conveyor_vision PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_compile_options(conveyor_vision PUBLIC ${GAZEBO_CFLAGS})

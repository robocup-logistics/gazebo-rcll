# ***************************************************************************
# Created:   Mon  3 May 15:32:13 CEST 2021
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

find_package(OGRE REQUIRED COMPONENTS Paging)

add_library(depthcam SHARED depthcam.cpp)
target_link_libraries(
  depthcam PUBLIC core configurable gazebo ${OGRE_LIBRARIES}
                  ${OGRE_Paging_LIBRARIES})
target_include_directories(
  depthcam PUBLIC ${OGRE_INCLUDE_DIRS} ${OGRE_Paging_INCLUDE_DIRS}
                  ${GAZEBO_INCLUDE_DIRS})
target_compile_options(depthcam PUBLIC ${GAZEBO_CFLAGS})

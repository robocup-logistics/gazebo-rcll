# ***************************************************************************
# Created:   Mon  3 May 15:48:24 CEST 2021
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

add_library(light_control SHARED light_control.cpp)
target_link_libraries(light_control PUBLIC configurable llsf_msgs gazsim_msgs
                                           gazebo)
target_include_directories(light_control PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_compile_options(light_control PUBLIC ${GAZEBO_CFLAGS})

# ***************************************************************************
# Created:   Mon  3 May 14:08:27 CEST 2021
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

cmake_minimum_required(VERSION 3.14)

project(gazebo-rcll)
include(GNUInstallDirs)

set(USERDIR
    ".fawkes"
    CACHE PATH
          "The directory for user config files, relative to its home directory")
set(CONFDIR
    "${CMAKE_SOURCE_DIR}/cfg"
    CACHE PATH "The directory where config files are stored")

add_compile_options(-g -Wall -Werror)
add_compile_options(-DUSERDIR="${USERDIR}" -DCONFDIR="${CONFDIR}")

include_directories(plugins/src/libs)
add_subdirectory(plugins/src)

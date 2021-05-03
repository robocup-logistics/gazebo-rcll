# ***************************************************************************
# Created:   Mon  3 May 14:33:33 CEST 2021
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

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(
  PROTO_SRCS
  PROTO_HDRS
  Float.proto
  NewPuck.proto
  SimTime.proto
  WorkpieceCommand.proto
  LightSignalDetection.proto)
add_library(gazsim_msgs SHARED ${PROTO_SRCS})
target_include_directories(gazsim_msgs
                           INTERFACE ${CMAKE_BINARY_DIR}/plugins/src/libs)

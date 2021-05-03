# ***************************************************************************
# Created:   Mon  3 May 14:44:57 CEST 2021
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
  AttentionMessage.proto
  BeaconSignal.proto
  ConveyorVisionResult.proto
  ExplorationInfo.proto
  GameInfo.proto
  GameState.proto
  LightSignals.proto
  LockMasterAnnounce.proto
  LockMessage.proto
  MachineCommands.proto
  MachineDescription.proto
  MachineInfo.proto
  MachineInstructions.proto
  MachineReport.proto
  OrderInfo.proto
  Pose2D.proto
  Pose3D.proto
  PreparedForProduction.proto
  ProductColor.proto
  PuckDetectionResult.proto
  RingInfo.proto
  RobotCommands.proto
  RobotInfo.proto
  SimTimeSync.proto
  Sync.proto
  Team.proto
  Time.proto
  VersionInfo.proto
  Zone.proto)

add_library(llsf_msgs SHARED ${PROTO_SRCS})
target_include_directories(llsf_msgs
                           INTERFACE ${CMAKE_BINARY_DIR}/plugins/src/libs)

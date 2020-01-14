/***************************************************************************
 *  delivery_station.cpp - controls a delivery station mps
 *
 *  Generated: Wed Apr 22 14:32:39 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "delivery_station.h"

using namespace gazebo;

DeliveryStation::DeliveryStation(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
    : Mps(_parent, _sdf), prepared_(false), puck_(NULL) {}

void DeliveryStation::on_puck_msg(ConstPosePtr &msg) {
  if (puck_in_input(msg) && !is_puck_hold(msg->name())) {
    puck_ = world_->GZWRAP_MODEL_BY_NAME(msg->name());
    printf("%s got puck %s\n", name_.c_str(), puck_->GetName().c_str());
    if (prepared_) {
      // We received the puck and have been prepared, thus deliver.
      deliver();
    }
  }
}

void DeliveryStation::new_machine_info(ConstMachine &machine) {
  if (machine.state() == "IDLE") {
    prepared_ = false;
    set_state(State::IDLE);
  } else if (machine.state() == "PREPARED") {
    prepared_ = true;
    if (puck_) {
      // We have a puck and the machine is prepared, thus we can deliver.
      deliver();
    }
  }
}

/** Send delivery information to the refbox and move the puck.
 * If we have a puck in the input and we received a prepare message, move the
 * puck to the selected gate and send a DELIVER command to the refbox, then
 * reset the puck and the prepared status.
 * Otherwise, do nothing.
 */
void DeliveryStation::deliver() {
  set_state(State::AVAILABLE);
  if (!prepared_ || !puck_) {
    // Machine is not prepared yet or there is no workpiece yet.
    return;
  }
  // TODO use the right gate
  puck_->SetWorldPose(get_puck_world_pose(0.3, -0.2));
  printf("%s: Sending delivery information for puck %s\n", name_.c_str(),
         puck_->GetName().c_str());
  gazsim_msgs::WorkpieceCommand cmd_msg;
  cmd_msg.set_command(gazsim_msgs::Command::DELIVER);
  cmd_msg.set_puck_name(puck_->GetName());
  if (name_[0] == 'C') {
    cmd_msg.set_team_color(gazsim_msgs::Team::CYAN);
  } else if (name_[0] == 'M') {
    cmd_msg.set_team_color(gazsim_msgs::Team::MAGENTA);
  }
  puck_cmd_pub_->Publish(cmd_msg);
  prepared_ = false;
  puck_ = NULL;
}

void DeliveryStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg) {

  // printf("MPS:GOT INSTRUCT MESSAGE\n");

  if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_DS) {
    return;
  }

  std::string machine_name = "NOT-SET";
  machine_name = msg->machine();

  std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());
}

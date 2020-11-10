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

DeliveryStation::DeliveryStation(physics::ModelPtr _parent, sdf::ElementPtr  _sdf) :
  Mps(_parent,_sdf),
  selected_gate_(0),
  puck_(NULL)
{
}

void DeliveryStation::on_puck_msg(ConstPosePtr &msg)
{

  if(puck_in_input(msg) &&
     !is_puck_hold(msg->name()))
  {
    puck_ = world_->GZWRAP_MODEL_BY_NAME(msg->name());
    printf("%s got puck %s\n", name_.c_str(), puck_->GetName().c_str());
    if (selected_gate_ > 0) {
      // We received the puck and know the gate, thus we can deliver.
      deliver();
    }
 }
}

void DeliveryStation::new_machine_info(ConstMachine &machine)
{
  if (machine.state() == "IDLE") {
    set_state(State::IDLE);
    puck_ = NULL;
    selected_gate_ = 0;
  }

  if (machine.state() == "BROKEN") {
      puck_ = NULL;
      selected_gate_ = 0;
  }

  if (machine.state() == "PREPARED"){
      if (machine.has_instruction_ds()){
          // selected_gate_ = machine.instruction_ds().gate();
          selected_gate_ = 1;
          printf("%s got the new gate %i\n", name_.c_str(), selected_gate_);
          // We already have a puck and now have the gate info, thus we can deliver.
          if (puck_)   deliver();
      }
  }

}

/** Send delivery information to the refbox and move the puck.
 * If we have a puck in the input and we received the gate information, move the
 * puck to the selected gate and send a DELIVER command to the refbox, then
 * reset the puck and the selected gate.
 * Otherwise, do nothing.
 */
void DeliveryStation::deliver()
{
  if (!selected_gate_ || !puck_) {
    // Gate is 0 (no prepare msg received yet) or no puck in the machine.
    return;
  }
  set_state(State::AVAILABLE);
  switch(selected_gate_)
  {
    case 1:
      puck_->SetWorldPose(get_puck_world_pose(0.3,-0.2));
      break;
    case 2:
      puck_->SetWorldPose(get_puck_world_pose(0.3,-0.1));
      break;
    case 3:
      puck_->SetWorldPose(get_puck_world_pose(0.3,-0.0));
      break;
    default:
      printf("bad gateway for puck\n");
      puck_->SetWorldPose(get_puck_world_pose(-0.5,0.5));
      return;
  }
  printf("%s: Sending delivery information for puck %s on gate %i\n",
      name_.c_str(), puck_->GetName().c_str(), selected_gate_);
  gazsim_msgs::WorkpieceCommand cmd_msg;
  cmd_msg.set_command(gazsim_msgs::Command::DELIVER);
  cmd_msg.set_puck_name(puck_->GetName());
  if(name_[0] == 'C')
  {
    cmd_msg.set_team_color(gazsim_msgs::Team::CYAN);
  }
  else if(name_[0] == 'M')
  {
    cmd_msg.set_team_color(gazsim_msgs::Team::MAGENTA);
  }
  puck_cmd_pub_->Publish(cmd_msg);
  selected_gate_ = 0;
  puck_ = NULL;
}

void DeliveryStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg){

    //printf("MPS:GOT INSTRUCT MESSAGE\n");

    if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_DS){
        return;
    }


    std::string machine_name = "NOT-SET";
    machine_name = msg->machine();

    std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());
}


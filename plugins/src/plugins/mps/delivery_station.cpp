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
  Mps(_parent,_sdf)
{
  selected_gate_ = 0;
}

void DeliveryStation::on_puck_msg(ConstPosePtr &msg)
{
  if(puck_in_input(msg) &&
     !is_puck_hold(msg->name()))
  {
    physics::ModelPtr puck = world_->GZWRAP_MODEL_BY_NAME(msg->name());
    printf("%s got puck %s for gate %i\n",this->name_.c_str(), puck->GetName().c_str(), selected_gate_);
    bool successfull_deliver = true;
    switch(selected_gate_)
    {
      case 1:
        puck->SetWorldPose(get_puck_world_pose(0.3,-0.2));
        break;
      case 2:
        puck->SetWorldPose(get_puck_world_pose(0.3,-0.1));
        break;
      case 3:
        puck->SetWorldPose(get_puck_world_pose(0.3,-0.0));
        break;
      default:
        printf("bad gateway for puck\n");
        puck->SetWorldPose(get_puck_world_pose(-0.5,0.5));
        successfull_deliver = false;
        break;
    }
    set_state(State::AVAILABLE);
    if(successfull_deliver)
    {
      gazsim_msgs::WorkpieceCommand cmd_msg;
      cmd_msg.set_command(gazsim_msgs::Command::DELIVER);
      cmd_msg.set_puck_name(msg->name());
      if(name_[0] == 'C')
      {
        cmd_msg.set_team_color(gazsim_msgs::Team::CYAN);
      }
      else if(name_[0] == 'M')
      {
        cmd_msg.set_team_color(gazsim_msgs::Team::MAGENTA);
      }
      puck_cmd_pub_->Publish(cmd_msg);
    }
  }
}

void DeliveryStation::new_machine_info(ConstMachine &machine)
{
  selected_gate_ = machine.instruction_ds().gate();
  printf("%s got the new gate %i\n", this->name_.c_str(), selected_gate_);
}



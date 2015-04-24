/***************************************************************************
 *  base_station.cpp - controls a basesation mps
 *
 *  Generated: Wed Apr 22 13:25:39 2015
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

#include "base_station.h"

using namespace gazebo;

BaseStation::BaseStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) :
  Mps(_parent,_sdf)
{
  have_puck_ = "";
  
  factoryPub = node_->Advertise<msgs::Factory>("~/factory");
}

void BaseStation::on_puck_msg(ConstPosePtr &msg)
{
  if(msg->name() == have_puck_ &&
     !puck_in_input(msg) &&
     !puck_in_output(msg))
  {
    have_puck_ = "";
    set_state(State::RETRIEVED);
  }
}

void BaseStation::new_machine_info(ConstMachine &machine)
{
  if(machine.state() == "PROCESSING" || machine.state() == "READY-AT-OUTPUT")
  {
    if(!machine.has_instruction_bs())
    {
      printf("machine %s without instructions",name_.c_str());
      return;
    }
    math::Pose spawn_pose;
    if(machine.instruction_bs().side() == llsf_msgs::MachineSide::INPUT)
    {
      spawn_pose = math::Pose(input_x(),input_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at input\n");
    }
    else if(machine.instruction_bs().side() == llsf_msgs::MachineSide::OUTPUT)
    {
      spawn_pose = math::Pose(output_x(), output_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at output\n");
    }
    else
    {
      spawn_pose = math::Pose(0,0,0,0,0,0);
    }
    msgs::Factory new_puck_msg;
    new_puck_msg.set_sdf_filename("model://workpiece_base");
    msgs::Set(new_puck_msg.mutable_pose(),spawn_pose);
    //new_puck_msg.set_edit_name(std::string("puckn"));
    factoryPub->Publish(new_puck_msg);
    have_puck_ = "workpiece_base";
    set_state(State::PROCESSED);
  }
}

void BaseStation::on_new_puck(ConstNewPuckPtr &msg)
{
  Mps::on_new_puck(msg);
  physics::ModelPtr new_puck = world_->GetModel(msg->puck_name());
  if(puck_in_input(new_puck->GetWorldPose()) || puck_in_output(new_puck->GetWorldPose()))
  {
    have_puck_ = new_puck->GetName();
  }
  
}

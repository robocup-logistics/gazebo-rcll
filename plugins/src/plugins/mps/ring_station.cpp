/***************************************************************************
 *  ring_station.h - controls a ring station mps
 *
 *  Generated: Wed Apr 22 13:48:39 2015
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

#include "ring_station.h"

using namespace gazebo;

RingStation::RingStation(physics::ModelPtr _parent, sdf::ElementPtr  _sdf) :
  Mps(_parent,_sdf)
{
}

void RingStation::on_puck_msg(ConstPosePtr &msg)
{
  //check if the puck is in the input area
  if(puck_in_input(msg))
  {
    set_state(State::AVAILABLE);
    puck_in_processing_name_ = msg->name();
    printf("%s got %s\n", name_.c_str(), puck_in_processing_name_.c_str());
  }
}

void RingStation::new_machine_info(ConstMachine &machine)
{
  if(machine.state() == "PREPARED")
  {
    switch(machine.instruction_rs().ring_color())
    {
      case llsf_msgs::RingColor::RING_BLUE:
        color_to_put_ = gazsim_msgs::Color::BLUE;
        break;
      case llsf_msgs::RingColor::RING_GREEN:
        color_to_put_ = gazsim_msgs::Color::GREEN;
        break;
      case llsf_msgs::RingColor::RING_ORANGE:
        color_to_put_ = gazsim_msgs::Color::ORANGE;
        break;
      case llsf_msgs::RingColor::RING_YELLOW:
        color_to_put_ = gazsim_msgs::Color::YELLOW;
        break;
    }
    printf("%s is prepared to put %s on a workpiece\n", name_.c_str(), gazsim_msgs::Color_Name(color_to_put_).c_str());
  }
  else if(machine.state() == "PROCESSED")
  {
    printf("%s is putting a %s ring onto %s\n", name_.c_str(), gazsim_msgs::Color_Name(color_to_put_).c_str(), puck_in_processing_name_.c_str());
    //teleport puck to output
    model_->GetWorld()->GetEntity(puck_in_processing_name_)->SetWorldPose(math::Pose(output_x(), output_y(), BELT_HEIGHT, 0, 0, 0));
    //spawn a ring ontop of the puck
    //write to the puck plugin
    if(!puck_cmd_pub_->HasConnections())
    {
      printf("cannot connect to puck %s on topic %s\n",puck_in_processing_name_.c_str(),TOPIC_PUCK_COMMAND);
    }
    else
    {
      //TODO: dont'spawn a fixed color, get color from better source
      gazsim_msgs::WorkpieceCommand cmd;
      cmd.set_command(gazsim_msgs::Command::ADD_RING);
      cmd.set_color(color_to_put_);
      cmd.set_puck_name(puck_in_processing_name_);
      puck_cmd_pub_->Publish(cmd);
    }
  }
}

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
  add_base_publisher_ = node_->Advertise<llsf_msgs::MachineAddBase>(TOPIC_MACHINE_ADD_BASE);
  number_bases_ = 0;
}

void RingStation::on_puck_msg(ConstPosePtr &msg)
{
  if(pose_hit(gzwrap::Pose3d(msg->position().x(), msg->position().y(), msg->position().z(), 0,0,0), add_base_pose(), 0.1) &&
     !is_puck_hold(msg->name()))
  {
    add_base();
    world_->GZWRAP_ENTITY_BY_NAME(msg->name())->SetWorldPose(get_puck_world_pose(-0.2,-0.5));
  }
  //check if the puck is in the input area
 if (current_state_ == "PREPARED") {
	 if(puck_in_input(msg) && !is_puck_hold(msg->name()))
	 {
		puck_in_processing_name_ = msg->name();
		printf("%s got %s\n", name_.c_str(),
			   puck_in_processing_name_.c_str());
		set_state(State::AVAILABLE);
	 }
 }
  if(current_state_ == "READY-AT-OUTPUT" &&
     msg->name() == puck_in_processing_name_ &&
     !puck_in_output(msg))
  {
    set_state(State::RETRIEVED);
    puck_in_processing_name_ = "";
  }
}

void RingStation::publish_indicator(bool active, int number)
{
  gazebo::msgs::Visual msg;
  msg.set_parent_name(name_+"::body");
  msg.set_name(name_+"::body::base_" + std::to_string(number));
#if GAZEBO_MAJOR_VERSION > 5
  gazebo::msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(-0.35 + (number*0.11),0,BELT_HEIGHT+0.3,0,0,0));
#else
  gazebo::msgs::Set(msg.mutable_pose(), gazebo::math::Pose(-0.35 + (number*0.11),0,BELT_HEIGHT+0.3,0,0,0));
#endif
  if(active)
  {
    msgs::Set(msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(1,0,0));
  }
  else
  {
    msgs::Set(msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(0.3,0,0));
  }
  visPub_->Publish(msg);
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
  else if(machine.state() == "PROCESSED" )
  {
    printf("%s: Putting a %s ring onto %s\n", name_.c_str(), gazsim_msgs::Color_Name(color_to_put_).c_str(), puck_in_processing_name_.c_str());
    if ( puck_in_processing_name_ != "")
    {
        //teleport puck to output
        printf("%s: Teleporting %s to output\n", name_.c_str(),puck_in_processing_name_.c_str());
        model_->GetWorld()->GZWRAP_ENTITY_BY_NAME(puck_in_processing_name_)->SetWorldPose(
            gzwrap::Pose3d(output_x(), output_y(), BELT_HEIGHT, 0, 0, 0) );
       //spawn a ring ontop of the puck
       //write to the puck plugin
       if(!puck_cmd_pub_->HasConnections())
          printf("cannot connect to puck %s on topic %s\n",puck_in_processing_name_.c_str(),topic_puck_command_.c_str());
       else
       {
           //TODO: dont'spawn a fixed color, get color from better source
          /// TODO: PUT THIS IN STATE PROCESSING?
          printf("%s is in %s state: Creating Product! \n",name_.c_str(),machine.state().c_str());
          gazsim_msgs::WorkpieceCommand cmd;
          cmd.set_command(gazsim_msgs::Command::ADD_RING);
          cmd.add_color(color_to_put_);
          cmd.set_puck_name(puck_in_processing_name_);
         puck_cmd_pub_->Publish(cmd);
       }
       set_state(State::DELIVERED);
    }
    else
    printf("%s: Puck not found at input\n", name_.c_str());
  }
  else if (machine.state() == "BROKEN")
  {
    puck_in_processing_name_ = "";
  }

  // show number of bases
  number_bases_ = machine.loaded_with();
  for(u_int32_t i=0; i < (u_int32_t) MAX_NUM_BASES; i++)
  {
    publish_indicator(i < machine.loaded_with(), i);
  }
}

void RingStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg){

    //printf("MPS:GOT INSTRUCT MESSAGE\n");

    if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_RS){
        return;
    }


    std::string machine_name = "NOT-SET";
    machine_name = msg->machine();

    std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());
}

void RingStation::add_base()
{
  printf("Adding Base to %s\n", name_.c_str());
  llsf_msgs::MachineAddBase add_base_msg;
  add_base_msg.set_machine_name(name_);
  add_base_publisher_->Publish(add_base_msg);
  publish_indicator(true, number_bases_++);
}

gzwrap::Pose3d RingStation::add_base_pose()
{
  return get_puck_world_pose(-0.25,0);
}

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

#include "storage_station.h"

using namespace gazebo;

StorageStation::StorageStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) :
  Mps(_parent,_sdf)
{



    shelf_pos_x = 10.0;
    shelf_pos_z = 0.0;

    if (name_.find("M-") != std::string::npos){

        shelf_pos_y = 0.0;
        printf("Strage_Station: Generating Puck Storage Magenta\n");
    }else{
        shelf_pos_y = 10.0;
        printf("Strage_Station: Generating Puck Storage Cyan\n");
    }

    shelf_x_offset = 0.1;
    shelf_y_offset = 0.1;
    shelf_z_offset = 0.4;


    for (int z=0; z<6;z++){
        for (int y=0; y<4; y++)
            for (int x=0; x<2; x++){

            spawn_puck(get_slot_position(x,y,z),gazsim_msgs::Color::RED);
            }
    }





  have_puck_ = "";
}

void StorageStation::on_puck_msg(ConstPosePtr &msg)
{
    /*
  if(msg->name() == have_puck_ &&
     !puck_in_input(msg) &&
     !puck_in_output(msg))
  {
    have_puck_ = "";
    set_state(State::RETRIEVED);
  }
  */
}

void StorageStation::new_machine_info(ConstMachine &machine)
{
  /*
  if(machine.state() == "PROCESSED")
  {
    if(!machine.has_instruction_bs())
    {
      printf("machine %s without instructions",name_.c_str());
      return;
    }
    gzwrap::Pose3d spawn_pose;
    if(machine.instruction_bs().side() == llsf_msgs::MachineSide::INPUT)
    {
      spawn_pose = gzwrap::Pose3d(input_x(),input_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at input\n");
    }
    else if(machine.instruction_bs().side() == llsf_msgs::MachineSide::OUTPUT)
    {
      spawn_pose = gzwrap::Pose3d(output_x(), output_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at output\n");
    }
    else
      spawn_pose = gzwrap::Pose3d::Zero;

    gazsim_msgs::Color spawn_clr;
    switch(machine.instruction_bs().color()){
      case llsf_msgs::BaseColor::BASE_BLACK:
        spawn_clr = gazsim_msgs::Color::BLACK;
        break;
      case llsf_msgs::BaseColor::BASE_SILVER:
        spawn_clr = gazsim_msgs::Color::SILVER;
        break;
      case llsf_msgs::BaseColor::BASE_RED:
      default:
        spawn_clr = gazsim_msgs::Color::RED;
        break;
    }

    spawn_puck(spawn_pose, spawn_clr);
    have_puck_ = "workpiece_base";
    set_state(State::PROCESSED);
    set_state(State::DELIVERED);
  }
  */
}

void StorageStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg){

    printf("MPS:GOT INSTRUCT MESSAGE\n");

    if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_SS){
        return;
    }


    std::string machine_name = "NOT-SET";
    machine_name = msg->machine();

    std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());
}

void StorageStation::on_new_puck(ConstNewPuckPtr &msg)
{

  Mps::on_new_puck(msg);
  physics::ModelPtr new_puck = world_->GZWRAP_MODEL_BY_NAME(msg->puck_name());
  if(puck_in_input(new_puck->GZWRAP_WORLD_POSE()) || puck_in_output(new_puck->GZWRAP_WORLD_POSE()))
  {
    have_puck_ = new_puck->GetName();
  }
  
}

gzwrap::Pose3d StorageStation::get_slot_position(uint32_t slot_x,uint32_t slot_y,uint32_t slot_z){

    double x,y,z;
    slot_z+=1;
    slot_y+=1;
    slot_x+=1;
    x = shelf_pos_x + (slot_x * shelf_x_offset) +(slot_z *shelf_z_offset) ; //0.1/0.2 , 0.2/0.4
    y = shelf_pos_y + (slot_y * shelf_y_offset);

    z = 0.0;

    return gzwrap::Pose3d(gazebo::math::Pose(x,y,z,0,0,0));
}

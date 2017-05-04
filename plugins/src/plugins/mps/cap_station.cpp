/***************************************************************************
 *  cap_station.h - controls a cap station mps
 *
 *  Generated: Wed Apr 22 12:48:29 2015
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

#include <utils/misc/gazebo_api_wrappers.h>

#include "cap_station.h"

using namespace gazebo;

CapStation::CapStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) :
  Mps(_parent,_sdf)
{
  spawn_puck(shelf_left_pose(), gazsim_msgs::Color::RED);
  spawn_puck(shelf_middle_pose(), gazsim_msgs::Color::RED);
  spawn_puck(shelf_right_pose(), gazsim_msgs::Color::RED);
  workpiece_result_subscriber_ = node_->Subscribe(TOPIC_PUCK_COMMAND_RESULT ,&CapStation::on_puck_result,this);
  stored_cap_color_ = gazsim_msgs::Color::NONE;
  puck_spawned_time_ = created_time_;
}

void CapStation::OnUpdate(const common::UpdateInfo &info)
{
  Mps::OnUpdate(info);
  if(model_->GetWorld()->GZWRAP_SIM_TIME().Double() - puck_spawned_time_ < SPAWN_PUCK_TIME)
  {
    return;
  }

  if(puck_in_shelf_left_ && !pose_hit(puck_in_shelf_left_->GZWRAP_WORLD_POSE(),shelf_left_pose(),0.1))
    puck_in_shelf_left_ = nullptr;
  if(puck_in_shelf_middle_ && !pose_hit(puck_in_shelf_middle_->GZWRAP_WORLD_POSE(),shelf_middle_pose(),0.1))
    puck_in_shelf_middle_ = nullptr;
  if(puck_in_shelf_right_ && !pose_hit(puck_in_shelf_right_->GZWRAP_WORLD_POSE(),shelf_right_pose(),0.1))
    puck_in_shelf_right_ = nullptr;
  if(!puck_in_shelf_right_ && !puck_in_shelf_middle_ && !puck_in_shelf_left_){
    //shelf is empty -> refill
    spawn_puck(shelf_left_pose(), gazsim_msgs::Color::RED);
    spawn_puck(shelf_middle_pose(), gazsim_msgs::Color::RED);
    spawn_puck(shelf_right_pose(), gazsim_msgs::Color::RED);
    puck_spawned_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
  }
}

void CapStation::work_puck(std::string puck_name)
{
    printf("CAPSTATION: on_work_puck: %s\n",puck_name.c_str());

  set_state(State::AVAILABLE);
  gazsim_msgs::WorkpieceCommand cmd_msg = gazsim_msgs::WorkpieceCommand();
  cmd_msg.set_puck_name(puck_name);
  switch(task_)
  {
    case llsf_msgs::CSOp::RETRIEVE_CAP:
      printf("%s retrives cap from %s\n ", name_.c_str(), puck_name.c_str());
      cmd_msg.set_command(gazsim_msgs::Command::REMOVE_CAP);
      puck_cmd_pub_->Publish(cmd_msg);
      break;
    case llsf_msgs::CSOp::MOUNT_CAP:
      if(stored_cap_color_ != gazsim_msgs::Color::NONE)
      {
	printf("%s mounts cap on %s with color %s\n", name_.c_str(), puck_name.c_str(), gazsim_msgs::Color_Name(stored_cap_color_).c_str());
	cmd_msg.set_command(gazsim_msgs::Command::ADD_CAP);
    cmd_msg.add_color(stored_cap_color_);
      
	gazebo::msgs::Visual vis_msg;
	vis_msg.set_parent_name(name_+"::body");
	vis_msg.set_name(name_+"::body::have_cap");
	gazebo::msgs::Set(vis_msg.mutable_material()->mutable_diffuse(),gazebo::common::Color(0.3,0,0));
	visPub_->Publish(vis_msg);
	puck_cmd_pub_->Publish(cmd_msg);
	stored_cap_color_ = gazsim_msgs::Color::NONE;
      }
      else
      {
	printf("%s can't mount cap on %s without a cap loaded first\n", name_.c_str(), puck_name.c_str());
      }
      break;
  }
  //set_state(State::PROCESSED);
  world_->GZWRAP_MODEL_BY_NAME(puck_name)->SetWorldPose(output());
  puck_in_processing_name_ = puck_name;
}

void CapStation::on_puck_msg(ConstPosePtr &msg)
{
  if(current_state_ == "READY-AT-OUTPUT")
  {
    if(puck_in_processing_name_ != "" && 
       !puck_in_output(world_->GZWRAP_MODEL_BY_NAME(puck_in_processing_name_)->GZWRAP_WORLD_POSE()))
    {
      set_state(State::RETRIEVED);
      puck_in_processing_name_ = "";
    }
  }
  else if(current_state_ == "PREPARED")
  {
    if(puck_in_input(msg)&&
       !is_puck_hold(msg->name()))
    {
      work_puck(msg->name());
    }
  }
}

void CapStation::on_new_puck(ConstNewPuckPtr &msg)
{
    Mps::on_new_puck(msg);

  //get model
  physics::Model_V models = world_->GZWRAP_MODELS();
  for(physics::ModelPtr model: models)
  {
    if(model->GetName() == msg->puck_name())
    {
      gazsim_msgs::WorkpieceCommand cmd;
      cmd.set_command(gazsim_msgs::Command::ADD_CAP);
      if(name_.find("CS1") != std::string::npos)
      {
        cmd.add_color(gazsim_msgs::Color::GREY);
      }
      else if(name_.find("CS2") != std::string::npos)
      {
        cmd.add_color(gazsim_msgs::Color::BLACK);
      }
      cmd.set_puck_name(msg->puck_name());
      if(pose_in_shelf_left(model->GZWRAP_WORLD_POSE()))
      {
        puck_in_shelf_left_ = model;
        puck_cmd_pub_->Publish(cmd);
      }
      else if(pose_in_shelf_middle(model->GZWRAP_WORLD_POSE()))
      {
        puck_in_shelf_middle_ = model;
        puck_cmd_pub_->Publish(cmd);
      }
      else if(pose_in_shelf_right(model->GZWRAP_WORLD_POSE()))
      {
        puck_in_shelf_right_ = model;
        puck_cmd_pub_->Publish(cmd);
      }
    }
  }
}

void CapStation::new_machine_info(ConstMachine &machine)
{
  if(machine.state() == "PREPARED")
  {
    task_ = machine.instruction_cs().operation();
    printf("%s got a new task: %s\n",name_.c_str(),llsf_msgs::CSOp_Name(task_).c_str());
  }
  else if(machine.state() == "PROCESSED" && puck_in_output(world_->GZWRAP_MODEL_BY_NAME(puck_in_processing_name_)->GZWRAP_WORLD_POSE()))
  {
    set_state(State::DELIVERED);
  }
  else if(machine.state() == "IDLE" &&
          current_state_ == "DOWN")
  {
    for(gazebo::physics::ModelPtr model: world_->GZWRAP_MODELS())
    {
      if(pose_hit(model->GZWRAP_WORLD_POSE(),input()))
      {
        work_puck(model->GetName());
      }
    }
  }
}

void CapStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg){

    //printf("MPS:GOT INSTRUCT MESSAGE\n");

    if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_CS){
        return;
    }


    std::string machine_name = "NOT-SET";
    machine_name = msg->machine();

    std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());
}


void CapStation::on_puck_result(ConstWorkpieceResultPtr &result)
{
    printf("CAPSTATION: on_puck_result: %s\n",result->puck_name().c_str());

  if(result->puck_name() == puck_in_processing_name_)
  {
    printf("%s got cap from %s with color %s\n",name_.c_str(), result->puck_name().c_str(), gazsim_msgs::Color_Name(result->color()).c_str());
    if(result->color() != gazsim_msgs::Color::NONE)
    {
      stored_cap_color_ = result->color();
      gazebo::msgs::Visual vis_msg;
      vis_msg.set_parent_name(name_+"::body");
      vis_msg.set_name(name_+"::body::have_cap");
      gazebo::msgs::Set(vis_msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(1,0,0));
      visPub_->Publish(vis_msg);
    }
  }
}

gzwrap::Pose3d CapStation::shelf_left_pose()
{
  return get_puck_world_pose(-0.1, 0, BELT_HEIGHT + 0.005);
}

gzwrap::Pose3d CapStation::shelf_middle_pose()
{
  return get_puck_world_pose(-0.2, 0, BELT_HEIGHT + 0.005);
}

gzwrap::Pose3d CapStation::shelf_right_pose()
{
  return get_puck_world_pose(-0.3,0, BELT_HEIGHT + 0.005);
}


bool CapStation::pose_in_shelf_left(const gzwrap::Pose3d &puck_pose)
{
  return pose_hit(puck_pose, shelf_left_pose(),0.05);
}

bool CapStation::pose_in_shelf_middle(const gzwrap::Pose3d &puck_pose)
{
  return pose_hit(puck_pose, shelf_middle_pose(),0.05);
}

bool CapStation::pose_in_shelf_right(const gzwrap::Pose3d &puck_pose)
{
  return pose_hit(puck_pose, shelf_right_pose(),0.05);
}

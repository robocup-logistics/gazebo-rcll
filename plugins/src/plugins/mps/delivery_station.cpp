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
}

void DeliveryStation::on_puck_msg(ConstPosePtr &msg)
{
  if(puck_in_input(msg))
  {
    physics::ModelPtr puck = world_->GetModel(msg->name());
    switch(selected_gate_)
    {
      case 1:
        puck->SetWorldPose(slide_1_pose());
        break;
      case 2:
        puck->SetWorldPose(slide_2_pose());
        break;
      case 3:
        puck->SetWorldPose(slide_3_pose());
        break;
    }
  }
}

void DeliveryStation::new_machine_info(ConstMachine &machine)
{
  selected_gate_ = machine.instruction_ds().gate();
}

math::Pose DeliveryStation::slide_1_pose()
{
  double mps_x = this->model_->GetWorldPose().pos.x;
  double mps_y = this->model_->GetWorldPose().pos.y;
  double mps_ori = this->model_->GetWorldPose().rot.GetAsEuler().z;
  double x = mps_x
      + (BELT_OFFSET_SIDE-0.30)  * cos(mps_ori)
      - ((BELT_LENGTH-0.05) / 2 - PUCK_SIZE) * sin(mps_ori);
  double y = mps_y
             + (BELT_OFFSET_SIDE-0.30)  * sin(mps_ori)
             + ((BELT_LENGTH-0.5) / 2 - PUCK_SIZE) * cos(mps_ori);
  return math::Pose(x,y,BELT_HEIGHT,0,00,0);
}

math::Pose DeliveryStation::slide_2_pose()
{  
  double mps_x = this->model_->GetWorldPose().pos.x;
  double mps_y = this->model_->GetWorldPose().pos.y;
  double mps_ori = this->model_->GetWorldPose().rot.GetAsEuler().z;
  double x = mps_x
             + (BELT_OFFSET_SIDE-0.30)  * cos(mps_ori)
             - ((BELT_LENGTH-0.05) / 2 - PUCK_SIZE) * sin(mps_ori);
  double y = mps_y
             + (BELT_OFFSET_SIDE-0.30)  * sin(mps_ori)
             + ((BELT_LENGTH-0.5) / 2 - PUCK_SIZE) * cos(mps_ori);
  return math::Pose(x,y,BELT_HEIGHT,0,00,0);
  //return math::Pose(0,0,0,0,0,0);
}

math::Pose DeliveryStation::slide_3_pose()
{
  double mps_x = this->model_->GetWorldPose().pos.x;
  double mps_y = this->model_->GetWorldPose().pos.y;
  double mps_ori = this->model_->GetWorldPose().rot.GetAsEuler().z;
  double x = mps_x
      + (BELT_OFFSET_SIDE-0.30)  * cos(mps_ori)
      - ((BELT_LENGTH-0.05) / 2 - PUCK_SIZE) * sin(mps_ori);
  double y = mps_y
             + (BELT_OFFSET_SIDE-0.30)  * sin(mps_ori)
             + ((BELT_LENGTH-0.5) / 2 - PUCK_SIZE) * cos(mps_ori);
  return math::Pose(x,y,BELT_HEIGHT,0,00,0);
  //return math::Pose(0,0,0,0,0,0);
}



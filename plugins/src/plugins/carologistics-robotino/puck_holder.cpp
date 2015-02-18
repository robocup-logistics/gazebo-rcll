/***************************************************************************
 *  puck_holder.cpp - holds a puck in the robotinos gripper while turning
 *
 *  Created: Mon Sep 30 17:02:08 2013
 *  Copyright  2013  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "puck_holder.h"
#include "../llsf/data_table.h"
#include "config.h"

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 */
PuckHolder::PuckHolder(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
PuckHolder::~PuckHolder()
{
}

void PuckHolder::init()
{
  printf("Initialize PuckHolder \n");
  table_ = LlsfDataTable::get_table();
  puck_attached_ = false;
}

void PuckHolder::create_publishers()
{
  //no publishers
}

void PuckHolder::create_subscribers()
{
  //no subscribers
}

void PuckHolder::update()
{
  //check if the robotino is turning
  math::Vector3 lin_vel = model->GetRelativeLinearVel();
  math::Vector3 ang_vel = model->GetRelativeAngularVel();
  if(lin_vel.GetLength() < 0.01 && ang_vel.GetLength() > 0.1)
  {
    //locate the center of the gripper
    math::Pose rob_pos = model->GetWorldPose();
    double center_x = rob_pos.pos.x + cos(rob_pos.rot.GetYaw()) * DISTANCE_GRIPPER_CENTER_ROBOTINO;
    double center_y = rob_pos.pos.y + sin(rob_pos.rot.GetYaw()) * DISTANCE_GRIPPER_CENTER_ROBOTINO;
    if(!puck_attached_)
    {
      //is a puck inside the gripper?
      Puck closest;
      float min_dist = 100;
      for(int p = 0; p < NUMBER_PUCKS; p++)
      {
	Puck puck = table_->get_puck(p);
	double distance = sqrt((puck.x - center_x) * (puck.x - center_x) + (puck.y - center_y) * (puck.y - center_y));
	if(distance < min_dist)
	{
	  min_dist = distance;
	  closest = puck;
	}
      }
      if(min_dist < 0.04)
      {
	puck_model_ = model->GetWorld()->GetEntity(closest.name_link.c_str())->GetParentModel();
	puck_attached_ = true;
      }
    }
    else
    {
      puck_model_->SetWorldPose(math::Pose(center_x, center_y, 0.02, 0, 0, 0));
    }
  }
  else
  {
    puck_attached_ = false;
  }
}

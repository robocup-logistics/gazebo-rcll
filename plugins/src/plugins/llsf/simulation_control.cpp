/***************************************************************************
 *  simulation_control.cpp - shuts down the simulation on request
 *
 *  Created: Mon Oct 07 11:39:40 2013
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
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>

#include "simulation_control.h"

using namespace gazebo;

/** Constructor
 * @param world World to control
 * @param gazebo_node Transport node to publish and subscribe messages on
 */
SimulationControl::SimulationControl(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  gazebo_node_ = gazebo_node;
  world_ = world;

  //create subscriber
  this->simulation_control_sub_ = gazebo_node_->Subscribe(std::string("~/LLSF/Control/"), &SimulationControl::on_string_msg, this);
}

SimulationControl::~SimulationControl()
{
}

void SimulationControl::on_string_msg(ConstHeaderPtr &msg)
{
  // if(msg->str_id().c_str().compare("end") == 0)
  // {
  //   //shutdown simulation
  //   printf("Executing stop request\n");
  //   world_->Stop();
  // }
}


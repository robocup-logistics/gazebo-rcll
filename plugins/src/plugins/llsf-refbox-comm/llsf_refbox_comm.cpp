/***************************************************************************
 *  llsf_refbox_comm.cpp - World plugin for the refbox connection in the llsf
 *
 *  Created: Fri Mar 06 16:10:09 2015
 *  Copyright  2015  Frederik Zwilling
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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string.h>

#include "llsf_refbox_comm.h"

using namespace gazebo;

LlsfRefboxCommPlugin::LlsfRefboxCommPlugin() : WorldPlugin() 
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init("LLSF");
}

LlsfRefboxCommPlugin::~LlsfRefboxCommPlugin() 
{
}

/** Initialization while loading the plugin
 * @param _world World where the plugi was loaded
 * @param _sdf Pointer to the sdf model definition
 */
void LlsfRefboxCommPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  
  //connect update function
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LlsfRefboxCommPlugin::Update, this));
  printf("Timesync-Plugin loaded!\n");
}

void LlsfRefboxCommPlugin::Update()
{
}

/***************************************************************************
 *  llsf_world.cpp - The main plugin for the llsf field
 *
 *  Created: Sun Aug 18 14:55:33 2013
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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "data_table.h"
#include "llsf_world.h"
#include <string.h>

using namespace gazebo;

LlsfWorldPlugin::LlsfWorldPlugin() : WorldPlugin() 
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init("LLSF");
  puck_update_frequency_ = 3.0;
  time_sync_frequency_ = 4.0;
}

LlsfWorldPlugin::~LlsfWorldPlugin() 
{
  delete light_control_;
  delete puck_localization_;
  delete rfid_sensors_;
  delete time_sync_;
  delete field_referee_;
  delete simulation_control_;
}

/** Initialization while loading the plugin
 * @param _world World where the plugi was loaded
 * @param _sdf Pointer to the sdf model definition
 */
void LlsfWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  
  //init simulation data
  LlsfDataTable::init(_world, node_);
  table_ = LlsfDataTable::get_table();

  //has to be created after the table
  light_control_ = new LightControl(world_);
  puck_localization_ = new PuckLocalization(world_);
  rfid_sensors_ = new RfidSensors();
  time_sync_ = new TimeSync(world_, node_);
  field_referee_ = new FieldReferee(world_);
  simulation_control_ = new SimulationControl(world_, node_);

  //connect update function
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LlsfWorldPlugin::Update, this));
  printf("LLSF-World-Plugin loaded!\n");
}

void LlsfWorldPlugin::Update()
{
  double time = world_->GetSimTime().Double();
  light_control_->update();
  if((time - last_puck_update_) > (1.0 / puck_update_frequency_))
  {
    last_puck_update_ = time;
    puck_localization_->update();
    rfid_sensors_->update();
    field_referee_->update();
  }
  if((time - last_time_sync_) > (1.0 / time_sync_frequency_))
  {
    last_time_sync_ = time;
    time_sync_->send_time_sync();
  }
}

/***************************************************************************
 *  time_sync.cpp - The main plugin for synchronizing the time with a robot control software
 *
 *  Created: Mon Mar 02 15:54:47 2015
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

#include "time_sync.h"

using namespace gazebo;

TimesyncPlugin::TimesyncPlugin() : WorldPlugin() 
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init("LLSF");
  time_sync_frequency_ = 4.0;

  //create publisher
  this->time_sync_pub_ = node_->Advertise<gazsim_msgs::SimTime>("~/gazsim/time-sync/");

  //init variables
  last_real_time_ = 0.0;
  last_sim_time_ = 0.0;
}

TimesyncPlugin::~TimesyncPlugin() 
{
}

/** Initialization while loading the plugin
 * @param _world World where the plugi was loaded
 * @param _sdf Pointer to the sdf model definition
 */
void TimesyncPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  
  //connect update function
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TimesyncPlugin::Update, this));
  last_time_sync_ = world_->GetSimTime().Double();
  printf("Timesync-Plugin loaded!\n");
}

void TimesyncPlugin::Update()
{
  double time = world_->GetSimTime().Double();
  if((time - last_time_sync_) > (1.0 / time_sync_frequency_))
  {
    last_time_sync_ = time;
    send_time_sync();
  }
}

void TimesyncPlugin::send_time_sync()
{
  double sim_time = world_->GetSimTime().Double();
  double real_time = world_->GetRealTime().Double();

  gazsim_msgs::SimTime msg;

  msg.set_sim_time_sec(sim_time); //automatically rounded to integer
  msg.set_sim_time_nsec((sim_time - msg.sim_time_sec()) * 1000000000.f);

  //Calculate real time factor (did not find it in gazebo api)
  double real_time_factor = (sim_time - last_sim_time_) / (real_time - last_real_time_);
  msg.set_real_time_factor(real_time_factor);

  msg.set_paused(!world_->GetRunning());
  time_sync_pub_->Publish(msg);

  last_sim_time_ = sim_time;
  last_real_time_ = real_time;
}

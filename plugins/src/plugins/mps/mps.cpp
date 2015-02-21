/***************************************************************************
 *  mps.cpp - Plugin to control a simulated MPS
 *
 *  Created: Fri Feb 20 17:15:54 2015
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

#include <math.h>

#include "mps.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Mps)

///Constructor
Mps::Mps()
{
}
///Destructor
Mps::~Mps()
{
  printf("Destructing Mps Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Mps::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Mps Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Mps::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->node_->Init(model_->GetWorld()->GetName());

  //subscribe for puck locations
  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    this->puck_subs_[i] = this->node_->Subscribe(std::string("~/puck_") + std::to_string(i) + "/gazsim/gps/" , &Mps::on_puck_msg, this);
  }
}

/** Called by the world update start event
 */
void Mps::OnUpdate(const common::UpdateInfo & /*_info*/)
{
}

/** on Gazebo reset
 */
void Mps::Reset()
{
}


/** Functions for recieving puck locations Messages
 * @param msg message
 */ 
void Mps::on_puck_msg(ConstPosePtr &msg)
{
  //printf("Got Msg from %s!!!", msg->name().c_str());
}

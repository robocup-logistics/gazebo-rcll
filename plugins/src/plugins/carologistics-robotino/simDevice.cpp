/***************************************************************************
 *  simDevice.cpp - general superclass for simulated devices
 *
 *  Created: Mon Jul 29 17:33:31 2013
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

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 */
SimDevice::SimDevice(physics::ModelPtr model, transport::NodePtr node)
{
  //Store the pointer to the model
  this->model = model;

  //communication Node
  this->node = node;

  last_sent_time_ = 0.0;
}

SimDevice::~SimDevice()
{
}

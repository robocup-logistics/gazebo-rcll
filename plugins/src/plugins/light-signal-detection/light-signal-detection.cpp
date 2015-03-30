/***************************************************************************
 *  light-signal-detection.cpp - provides ground truth light signal detection of the nearest achine in front of the robotino
 *  Created: Mon Mar 30 16:28:38 2015
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

#include "light-signal-detection.h"
#include <llsf_msgs/LightSignals.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(LightSignalDetection)

///Constructor
LightSignalDetection::LightSignalDetection()
{
}
///Destructor
LightSignalDetection::~LightSignalDetection()
{
  printf("Destructing LightSignalDetection Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void LightSignalDetection::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading LightSignalDetection Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LightSignalDetection::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(model_->GetWorld()->GetName()+"/"+name_);

  //init last sent time
  last_sent_time_ = model_->GetWorld()->GetSimTime().Double();

  //create publisher
  this->light_signal_pub_ = this->node_->Advertise<llsf_msgs::MachineSignal>("~/gazsim/light-signal/");
  
  robot_pose_ = model_->GetWorldPose();
}


/** Called by the world update start event
 */
void LightSignalDetection::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Safe robot position to know if a light signal is in front of the robot
  robot_pose_ = model_->GetWorldPose();
}

/** on Gazebo reset
 */
void LightSignalDetection::Reset()
{
}

/** Sending position to Fawkes
 * 
 */
void LightSignalDetection::send_light_detection()
{

}

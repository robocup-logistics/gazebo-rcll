/***************************************************************************
 *  gripper.cpp - provides gripper simulation
 *
 *  Created: Mon Mar 23 2015
 *  Copyright  2015 Stefan Profanter
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

#include "gripper.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Gripper)

///Constructor
Gripper::Gripper()
{
}
///Destructor
Gripper::~Gripper()
{
  printf("Destructing Gripper Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Gripper::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
    // Store the pointer to the model
    this->model_ = _parent;

    //get the model-name
    this->name_ = model_->GetName();
    printf("Loading Gripper Plugin of model %s\n", name_.c_str());

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Gripper::OnUpdate, this, _1));

    //Create the communication Node for communication with fawkes
    this->node_ = transport::NodePtr(new transport::Node());
    //the namespace is set to the model name!
    this->node_->Init(model_->GetWorld()->GetName()+"/"+name_);

    //create subscriber
    this->set_gripper_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/SetGripper/"), &Gripper::on_set_gripper_msg, this);
}


/** Called by the world update start event
 */
void Gripper::OnUpdate(const common::UpdateInfo & /*_info*/)
{

}

/** on Gazebo reset
 */
void Gripper::Reset()
{
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void Gripper::on_set_gripper_msg(ConstIntPtr &msg)
{
	if (msg->data() == 0)
		this->close();
	else
		this->open();
}


void Gripper::close() {
	std::cout << "Closing gripper!" << std::endl;
	//TODO add link to puck model

	physics::ModelPtr nearestPuck = getNearestPuck();

	// TODO link models and store nearest puck reference
}

void Gripper::open() {
	std::cout << "Opening gripper!" << std::endl;
	//TODO remove link from puck model (nearest puck reference stored in close)
}

physics::ModelPtr Gripper::getNearestPuck() {

	physics::ModelPtr nearest;

	//TODO filter returned list by name. Each puck starts with "Puck", e.g. "Puck0", "Puck1", ... and then find the nearest puck
	for(std::vector<physics::ModelPtr>::iterator it = model_->GetWorld()->GetModels().begin(); it != model_->GetWorld()->GetModels().end(); ++it) {
		physics::ModelPtr modelPtr = *it;
		std::cout << "Model = " << modelPtr->GetName() << std::endl;
	}

	return nearest;
}
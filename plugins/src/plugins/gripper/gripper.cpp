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
#include <cfloat>

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

    robotino_ = model_->GetParentModel();
    robotino_link_ = robotino_->GetChildLink("robotino3::body");

    grabJoint = model_->GetWorld()->GetPhysicsEngine()->CreateJoint( "revolute", model_);
    grabJoint->SetName("gripper_grab_puck");
    grabJoint->SetModel( model_);
    // grabJoint->SetPose(gazebo::math::Vector3(0.0,0.0,0.0));
}


/** Called by the world update start event
 */
void Gripper::OnUpdate(const common::UpdateInfo & /*_info*/)
{}

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

inline bool ends_with(std::string const & value, std::string const & ending)
{
	if (ending.size() > value.size()) return false;
		return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

gazebo::physics::LinkPtr Gripper::getLinkEndingWith(physics::ModelPtr model, std::string ending) {
	std::vector<gazebo::physics::LinkPtr> links = model->GetLinks();
	for (unsigned int i=0; i<links.size(); i++) {
		if (ends_with(links[i]->GetName(), ending))
			return links[i];
	}
	return gazebo::physics::LinkPtr();
}

gazebo::physics::JointPtr Gripper::getJointEndingWith(physics::ModelPtr model, std::string ending) {
	std::vector<gazebo::physics::JointPtr> joints = model->GetJoints();
	for (unsigned int i=0; i<joints.size(); i++) {
		if (ends_with(joints[i]->GetName(), ending))
			return joints[i];
	}
	return gazebo::physics::JointPtr();
}

void Gripper::close() {
	if (grippedPuck)
		return;

	std::cout << "Closing gripper!" << std::endl;
	//TODO add link to puck model
	grippedPuck = getNearestPuck();

        //teleport puck into gripper center
	setPuckPose();

	// link both models through a joint
	gazebo::physics::LinkPtr gripperLink = getLinkEndingWith(model_,"link");

	if (!gripperLink){
		std::cerr << "Link 'gripper_grab' not found in gripper model" << std::endl;
		return;
	}
        else{
          printf("Found Gripper Link\n");
        }

	gazebo::physics::LinkPtr puckLink = getLinkEndingWith(grippedPuck,"cylinder");
	if (!puckLink){
		std::cerr << "Link 'cylinder' not found in workpiece model" << std::endl;
		return;
	}
        else{
          printf("Found Puck Link\n");
        }

	grabJoint->Load(gripperLink, puckLink, math::Pose(-0.285, 0, 0, 0, 0, 0));
	grabJoint->Attach(gripperLink, puckLink);
        printf("Attached\n");

	grabJoint->SetAxis(0,  gazebo::math::Vector3(0.0f,0.0f,1.0f) );
	grabJoint->SetHighStop( 0, gazebo::math::Angle( 0.0f ) );
	grabJoint->SetLowStop( 0, gazebo::math::Angle( 0.0f ) );
        printf("Sachen gesetzt\n");
}

void Gripper::open() {
	if (!grippedPuck)
		return;

	// apply forces
	/*gazebo::physics::JointPtr leftFingerJoint = getJointEndingWith(model_,"left_finger_move");
	leftFingerJoint->SetForce(0,-10);
	gazebo::physics::JointPtr rightFingerJoint = getJointEndingWith(model_,"right_finger_move");
	rightFingerJoint->SetForce(0,10);

	grabJoint->Detach();*/

	std::cout << "Opening gripper!" << std::endl;
	grippedPuck.reset();
	//TODO remove link from puck model (nearest puck reference stored in close)
}

void Gripper::setPuckPose(){
	if (!grippedPuck)
		return;
	math::Pose gripperPose = model_->GetWorldPose();
	math::Pose newPose = gripperPose;

	newPose.pos.y += 0.28;
	newPose.pos.z += 0.93;
	grippedPuck->SetWorldPose(newPose);
}

physics::ModelPtr Gripper::getNearestPuck() {

	physics::ModelPtr nearest;
	double gripperX = model_->GetWorldPose().pos.x;
	double gripperY = model_->GetWorldPose().pos.y;
	double gripperZ = model_->GetWorldPose().pos.z;
	double distance = DBL_MAX;
	unsigned int modelCount = model_->GetWorld()->GetModelCount();
	physics::ModelPtr tmp;
	//filter returned list by name. Each puck starts with "Puck", e.g. "Puck0", "Puck1", ... and then find the nearest puck
	for(unsigned int i = 0 ; i < modelCount; i++){
		tmp = model_->GetWorld()->GetModel(i);
		if (fnmatch("puck*",tmp->GetName().c_str(),FNM_CASEFOLD) == 0){
			double puckX = tmp->GetWorldPose().pos.x;
			double puckY = tmp->GetWorldPose().pos.y;
			double puckZ = tmp->GetWorldPose().pos.z;
			double tmpDistance = std::sqrt(std::pow(gripperX-puckX,2)+std::pow(gripperY-puckY,2)+std::pow(gripperZ-puckZ,2));
			if(tmpDistance < distance){
				distance = tmpDistance;
				nearest = tmp;
			}
		}
	}
	std::cout << "Nearest puck: " << nearest->GetName() << std::endl;
	return nearest;
}

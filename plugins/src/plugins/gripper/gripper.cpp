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

#include "gripper.h"

#include <utils/misc/gazebo_api_wrappers.h>

#include <cfloat>
#include <math.h>
#include <queue>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Gripper)

///Constructor
Gripper::Gripper()
{
	last_action_rcvd_ = NOTHING;
}
///Destructor
Gripper::~Gripper()
{
	printf("Destructing Gripper Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void
Gripper::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	// Store the pointer to the model
	this->model_ = _parent;

	//get the model-name
	this->name_ = model_->GetName();
	printf("Loading Gripper Plugin of model %s\n", name_.c_str());

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->update_connection_ =
	  event::Events::ConnectWorldUpdateBegin(boost::bind(&Gripper::OnUpdate, this, _1));

	//Create the communication Node for communication with fawkes
	this->node_ = transport::NodePtr(new transport::Node());
	//the namespace is set to the model name!
	this->node_->Init(model_->GetWorld()->GZWRAP_NAME() + "/" + name_);

	//create subscriber
	this->set_gripper_sub_ =
	  this->node_->Subscribe(std::string(TOPIC_SET_GRIPPER), &Gripper::on_set_gripper_msg, this);

	has_puck_pub_ = this->node_->Advertise<msgs::Int>(TOPIC_HOLDS_PUCK);
	joint_pub_    = this->node_->Advertise<msgs::Joint>(TOPIC_JOINT);

	robotino_      = model_->GetParentModel();
	robotino_link_ = robotino_->GetChildLink("robotino3::body");

	grabJoint = model_->GetWorld()->GZWRAP_PHYSICS()->CreateJoint("revolute", model_);
	grabJoint->SetName("gripper_grab_puck");
	grabJoint->SetModel(model_);
	// grabJoint->SetPose(gazebo::math::Vector3(0.0,0.0,0.0));

	action_duration_ = 3.0;
}

/** Called by the world update start event
 */
void
Gripper::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	if (message_queue_.empty()) {
		return;
	}

	//    double time = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	//    if(time - last_action_time_ < action_duration_){
	//        std::cout << "WAIT FOR GRIPPER_ACTION" << std::endl;
	//        return;
	//    } else{
	//      last_action_time_ = time;
	//      std::cout << "START GRIPPER ACTION" << std::endl;
	//    }

	int action = message_queue_.front();
	message_queue_.pop();

	switch (action) {
	case CLOSE: this->close(); break;
	case OPEN: this->open(); break;
	default: last_action_rcvd_ = NOTHING; break;
	}
}

/** on Gazebo reset
 */
void
Gripper::Reset()
{
	open();
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void
Gripper::on_set_gripper_msg(ConstIntPtr &msg)
{
	switch (msg->data()) {
	case 0: message_queue_.push(CLOSE); break;
	case 1: message_queue_.push(OPEN); break;
	case 2: message_queue_.push(MOVE); break;
	default: break;
	}
}

inline bool
ends_with(std::string const &value, std::string const &ending)
{
	if (ending.size() > value.size())
		return false;
	return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

gazebo::physics::LinkPtr
Gripper::getLinkEndingWith(physics::ModelPtr model, std::string ending)
{
	std::vector<gazebo::physics::LinkPtr> links = model->GetLinks();
	for (unsigned int i = 0; i < links.size(); i++) {
		if (ends_with(links[i]->GetName(), ending))
			return links[i];
	}
	return gazebo::physics::LinkPtr();
}

gazebo::physics::JointPtr
Gripper::getJointEndingWith(physics::ModelPtr model, std::string ending)
{
	std::vector<gazebo::physics::JointPtr> joints = model->GetJoints();
	for (unsigned int i = 0; i < joints.size(); i++) {
		if (ends_with(joints[i]->GetName(), ending))
			return joints[i];
	}
	return gazebo::physics::JointPtr();
}

void
Gripper::close()
{
	std::cout << "Closing gripper!" << std::endl;

	if (grippedPuck) {
		sendHasPuck(true);
		return;
	}

	grippedPuck = getNearestPuck();
	if (!grippedPuck) {
		printf("No Puck found in gripper.\n");
		return;
	}

	//teleport puck into gripper center
	setPuckPose();

	// link both models through a joint
	gazebo::physics::LinkPtr gripperLink = getLinkEndingWith(model_, "link");

	if (!gripperLink) {
		std::cerr << "Link 'gripper_grab' not found in gripper model" << std::endl;
		return;
	}

	gazebo::physics::LinkPtr puckLink = getLinkEndingWith(grippedPuck, "cylinder");
	if (!puckLink) {
		std::cerr << "Link 'cylinder' not found in workpiece model" << std::endl;
		return;
	}

	grabJoint->Load(gripperLink, puckLink, gzwrap::Pose3d(-0.285, 0, 0, 0, 0, 0));
	grabJoint->Attach(gripperLink, puckLink);

	grabJoint->SetAxis(0, gzwrap::Vector3d::UnitZ);
#if GAZEBO_MAJOR_VERSION >= 8
	grabJoint->SetUpperLimit(0, 0);
	grabJoint->SetLowerLimit(0, 0);
#else
	grabJoint->SetHighStop(0, gazebo::math::Angle(0.0f));
	grabJoint->SetLowStop(0, gazebo::math::Angle(0.0f));
#endif

	sendHasPuck(true);
}

void
Gripper::open()
{
	if (!grippedPuck)
		return;

	grabJoint->Detach();

	std::cout << "Opening gripper!" << std::endl;
	grippedPuck.reset();

	sendHasPuck(false);
}

void
Gripper::setPuckPose()
{
	if (!grippedPuck)
		return;
	gzwrap::Pose3d newPose = getGripperLink()->GZWRAP_WORLD_POSE();

	// printf("gripper pos: (%f,%f,%f)", newPose.pos.x, newPose.pos.y, newPose.rot.GetYaw());
	// newPose.pos.x += 0.28 * cos(newPose.rot.GetYaw());
	// newPose.pos.y += 0.28 * sin(newPose.rot.GetYaw());
	// newPose.pos.z += 0.93;
	grippedPuck->SetWorldPose(newPose);
}

physics::ModelPtr
Gripper::getNearestPuck()
{
	physics::ModelPtr nearest;
	gzwrap::Pose3d    gripperPose = getGripperLink()->GZWRAP_WORLD_POSE();
	double            distance    = DBL_MAX;
	unsigned int      modelCount  = model_->GetWorld()->GZWRAP_MODEL_COUNT();
	physics::ModelPtr tmp;
	//filter returned list by name. Each puck starts with "Puck", e.g. "Puck0", "Puck1", ... and then find the nearest puck
	for (unsigned int i = 0; i < modelCount; i++) {
		tmp = model_->GetWorld()->GZWRAP_MODEL_BY_INDEX(i);
		if (fnmatch("puck*", tmp->GetName().c_str(), FNM_CASEFOLD) == 0) {
			double tmpDistance = gripperPose.GZWRAP_POS.Distance(tmp->GZWRAP_WORLD_POSE().GZWRAP_POS);
			if (tmpDistance < distance) {
				distance = tmpDistance;
				nearest  = tmp;
			}
		}
	}
	if (nearest == nullptr) {
		grippedPuck.reset();
		return nullptr;
	}
	std::cout << "Nearest puck: " << nearest->GetName() << std::endl;
	std::cout << "Distance: " << distance << "GRAB AREA: " << RADIUS_GRAB_AREA << std::endl;
	if (distance < RADIUS_GRAB_AREA) {
		return nearest;
	} else {
		grippedPuck.reset();
		return grippedPuck;
	}
}

void
Gripper::sendHasPuck(bool has_puck)
{
	//send msg to robot framework
	msgs::Int msg;
	if (has_puck) {
		msg.set_data(1);
	} else {
		msg.set_data(0);
	}
	has_puck_pub_->Publish(msg);
	//send info to mps
	msgs::Joint joint_msg;
	joint_msg.set_name(name_);
	joint_msg.set_id(grabJoint->GetId());
	joint_msg.set_parent_id(grabJoint->GetId());
	if (has_puck) {
		joint_msg.set_child_id(grippedPuck->GetId());
		joint_msg.set_child(grippedPuck->GetName());
	} else {
		joint_msg.set_child_id(0);
		joint_msg.set_child("");
	}
	joint_pub_->Publish(joint_msg);
}

/**
 * Get the link of the gripper, which could be directly included in the model or in a submodel
 * (e.g. when having a model robotino-number-1 which includes a team specific robotino model that contains the gripper)
 */
gazebo::physics::LinkPtr
Gripper::getGripperLink()
{
	// XXX: Switch to 'constexpr linkLen = length("gripper::link")' in c++11
	static const short int linkLen = strlen("gripper::link");

	// Search for gripper in model
	physics::LinkPtr res = model_->GetLink("gripper::link");
	if (res)
		return res;

	// Search for gripper link in included submodels
	std::vector<physics::LinkPtr> links = model_->GetLinks();
	for (std::vector<physics::LinkPtr>::iterator it = links.begin(); it != links.end(); it++) {
		printf("Checking %s\n", (*it)->GetName().c_str());
		if ((*it)->GetName().rfind("gripper::link", (*it)->GetName().length() - linkLen)
		    != std::string::npos)
			return (*it);
	}
	printf("Could not find gripper link of model %s\n", name_.c_str());

	return res;
}

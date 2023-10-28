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

	//create publisher
	has_puck_pub_       = this->node_->Advertise<msgs::Int>(TOPIC_HOLDS_PUCK);
	joint_pub_          = this->node_->Advertise<msgs::Joint>(TOPIC_JOINT);
	gripper_pose_pub_   = this->node_->Advertise<msgs::Pose>(TOPIC_GRIPPER_POSE);
	final_pub_          = this->node_->Advertise<msgs::Int>(TOPIC_FINAL);
	gripper_closed_pub_ = this->node_->Advertise<msgs::Int>(TOPIC_GRIPPER_CLOSED);

	gripperHandLink_ = getLinkEndingWith(model_, "gripper-hand");

	leftFingerJoint_  = getJointEndingWith(model_, "left-finger-hinge");
	rightFingerJoint_ = getJointEndingWith(model_, "right-finger-hinge");

	if (leftFingerJoint_) {
		//finger_yaw - 90°rotation = 1.04719 - pi/2 = -0.5236
		leftFingerJoint_->SetAxis(0, {cos(-0.5236), sin(-0.5236), 0});
	}
	if (rightFingerJoint_) {
		//finger_yaw - 90°rotation = -1.04719 - pi/2 = -2.6180
		rightFingerJoint_->SetAxis(0, {cos(-2.6180), sin(-2.6180), 0});
	}

	gripperJointX_ = getJointEndingWith(model_, "gripper_joint_x");
	gripperJointY_ = getJointEndingWith(model_, "gripper_joint_y");
	gripperJointZ_ = getJointEndingWith(model_, "gripper_joint_z");

	if (gripperJointY_) {
		//reverse y-axis
		gripperJointY_->SetAxis(0, {0, -1, 0});
	}

	grabJoint_ = model_->GetWorld()->GZWRAP_PHYSICS()->CreateJoint("fixed", model_);
	grabJoint_->SetName("gripper_grab_puck");
	grabJoint_->SetModel(model_);

	sendGripperClosed(true);
	finger_moving_         = 0;
	gripper_moving_        = false;
	left_last_yaw_         = 10;
	left_second_last_yaw_  = 10;
	right_last_yaw_        = 10;
	right_second_last_yaw_ = 10;
}

/** Called by the world update start event
 */
void
Gripper::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	//if gripper is opening
	if (finger_moving_ == 1) {
		if (leftFingerJoint_->Position(0) >= leftFingerJoint_->UpperLimit(0) - FINGER_TOLERANCE
		    && rightFingerJoint_->Position(0) >= rightFingerJoint_->UpperLimit(0) - FINGER_TOLERANCE) {
			std::cout << "Gripper open" << std::endl;
			sendGripperClosed(false);
			setFingerMoving(0);
		}

		if (leftFingerJoint_->Position(0) < leftFingerJoint_->UpperLimit(0) - FINGER_TOLERANCE) {
			leftFingerJoint_->SetVelocity(0, FINGER_OPEN_VELOCITY);
		}

		if (rightFingerJoint_->Position(0) < rightFingerJoint_->UpperLimit(0) - FINGER_TOLERANCE) {
			rightFingerJoint_->SetVelocity(0, FINGER_OPEN_VELOCITY);
		}
		//if gripper is closing
	} else if (finger_moving_ == 2) {
		//gripper is closed if fingers are at LowerLimit or cannot close
		//further (e.g. because the puck is blocking one)
		if ((leftFingerJoint_->Position(0) >= left_second_last_yaw_
		     || leftFingerJoint_->Position(0) <= rightFingerJoint_->LowerLimit(0) + FINGER_TOLERANCE)
		    && (rightFingerJoint_->Position(0) >= right_second_last_yaw_
		        || rightFingerJoint_->Position(0)
		             <= rightFingerJoint_->LowerLimit(0) + FINGER_TOLERANCE)) {
			std::cout << "Gripper closed" << std::endl;
			sendGripperClosed(true);
			setFingerMoving(0);
			left_last_yaw_         = 10;
			left_second_last_yaw_  = 10;
			right_last_yaw_        = 10;
			right_second_last_yaw_ = 10;
			grabWP();
		} else {
			left_last_yaw_         = leftFingerJoint_->Position(0);
			left_second_last_yaw_  = left_last_yaw_;
			right_last_yaw_        = rightFingerJoint_->Position(0);
			right_second_last_yaw_ = right_last_yaw_;
		}

		if (leftFingerJoint_->Position(0) > leftFingerJoint_->LowerLimit(0) + FINGER_TOLERANCE) {
			leftFingerJoint_->SetVelocity(0, -FINGER_CLOSE_VELOCITY);
		}

		if (rightFingerJoint_->Position(0) > rightFingerJoint_->LowerLimit(0) + FINGER_TOLERANCE) {
			rightFingerJoint_->SetVelocity(0, -FINGER_CLOSE_VELOCITY);
		}
	}

	if (gripper_moving_) {
		if (gripperJointX_->Position(0) <= gripper_target_x_ + GRIPPER_TOLERANCE
		    && gripperJointX_->Position(0) >= gripper_target_x_ - GRIPPER_TOLERANCE
		    && gripperJointY_->Position(0) <= gripper_target_y_ + GRIPPER_TOLERANCE
		    && gripperJointY_->Position(0) >= gripper_target_y_ - GRIPPER_TOLERANCE
		    && gripperJointZ_->Position(0) <= gripper_target_z_ + GRIPPER_TOLERANCE
		    && gripperJointZ_->Position(0) >= gripper_target_z_ - GRIPPER_TOLERANCE) {
			std::cout << "Gripper reached position (" + std::to_string(gripper_target_x_) + ","
			               + std::to_string(gripper_target_y_) + "," + std::to_string(gripper_target_z_)
			               + ")"
			          << std::endl;
			setGripperMoving(false);
		}
		if (gripperJointX_->Position(0) < gripper_target_x_ - GRIPPER_TOLERANCE) {
			gripperJointX_->SetVelocity(0, GRIPPER_VELOCITY);
		} else if (gripperJointX_->Position(0) > gripper_target_x_ + GRIPPER_TOLERANCE) {
			gripperJointX_->SetVelocity(0, -GRIPPER_VELOCITY);
		}

		if (gripperJointY_->Position(0) < gripper_target_y_ - GRIPPER_TOLERANCE) {
			gripperJointY_->SetVelocity(0, GRIPPER_VELOCITY);
		} else if (gripperJointY_->Position(0) > gripper_target_y_ + GRIPPER_TOLERANCE) {
			gripperJointY_->SetVelocity(0, -GRIPPER_VELOCITY);
		}

		if (gripperJointZ_->Position(0) < gripper_target_z_ - GRIPPER_TOLERANCE) {
			gripperJointZ_->SetVelocity(0, GRIPPER_VELOCITY);
		} else if (gripperJointZ_->Position(0) > gripper_target_z_ + GRIPPER_TOLERANCE) {
			gripperJointZ_->SetVelocity(0, -GRIPPER_VELOCITY);
		}

		//send position
		msgs::Pose posMsg;
		posMsg.set_name(this->name_);
		posMsg.mutable_position()->set_x(gripperJointX_->Position(0));
		posMsg.mutable_position()->set_y(gripperJointY_->Position(0));
		posMsg.mutable_position()->set_z(gripperJointZ_->Position(0));
		posMsg.mutable_orientation()->set_x(0);
		posMsg.mutable_orientation()->set_y(0);
		posMsg.mutable_orientation()->set_z(0);
		posMsg.mutable_orientation()->set_w(0);
		gripper_pose_pub_->Publish(posMsg);
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
Gripper::on_set_gripper_msg(ConstGripperCommandPtr &msg)
{
	switch (msg->command()) {
	case 0: close(); break;
	case 1: open(); break;
	case 2: move_gripper(msg->x(), msg->y(), msg->z()); break;
	default: std::cerr << "Unexpected Gripper Command!" << std::endl; break;
	}
}

void
Gripper::close()
{
	std::cout << "Closing gripper" << std::endl;

	if (grippedPuck_) {
		sendHasPuck(true);
		return;
	}

	if (!(leftFingerJoint_ && rightFingerJoint_)) {
		std::cout << "Could not find all joints to close the gripper!" << std::endl;
	} else {
		setFingerMoving(2);
	}
}

void
Gripper::open()
{
	std::cout << "Opening gripper" << std::endl;

	// start moving the fingers
	if (!(leftFingerJoint_ && rightFingerJoint_)) {
		std::cout << "Could not find all joints to open the gripper!" << std::endl;
	} else {
		setFingerMoving(1);
	}

	// remove puck
	if (!grippedPuck_) {
		return;
	}

	gazebo::physics::LinkPtr puckLink = getLinkEndingWith(grippedPuck_, "cylinder");
	if (!puckLink) {
		std::cerr << "Link 'cylinder' not found in workpiece model!" << std::endl;
		return;
	}

	puckLink->SetCollideMode("all");
	grabJoint_->Detach();
	grippedPuck_.reset();
	sendHasPuck(false);
}

void
Gripper::move_gripper(float x, float y, float z)
{
	float target_x = std::max((float)gripperJointX_->LowerLimit(0),
	                          std::min(x, (float)gripperJointX_->UpperLimit(0)));
	float target_y = std::max((float)gripperJointY_->LowerLimit(0),
	                          std::min(y, (float)gripperJointY_->UpperLimit(0)));
	float target_z = std::max((float)gripperJointZ_->LowerLimit(0),
	                          std::min(z, (float)gripperJointZ_->UpperLimit(0)));
	if (!(gripperJointX_ && gripperJointY_ && gripperJointZ_)) {
		std::cout << "Could not find all joints to move the gripper!" << std::endl;
	} else {
		if (x != target_x || y != target_y || z != target_z) {
			std::cout << "Gripper Position (" + std::to_string(x) + "," + std::to_string(y) + ","
			               + std::to_string(z) + ") cannot be reached!"
			          << std::endl;
			std::cout << "Gripper moves to (" + std::to_string(target_x) + "," + std::to_string(target_y)
			               + "," + std::to_string(target_z) + ") instead!"
			          << std::endl;
		} else {
			std::cout << "Moving gripper to (" + std::to_string(x) + "," + std::to_string(y) + ","
			               + std::to_string(z) + ")"
			          << std::endl;
		}
		setGripperMoving(true);
		gripper_target_x_ = target_x;
		gripper_target_y_ = target_y;
		gripper_target_z_ = target_z;
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
		if (ends_with(links[i]->GetName(), ending)) {
			return links[i];
		}
	}
	return gazebo::physics::LinkPtr();
}

gazebo::physics::JointPtr
Gripper::getJointEndingWith(physics::ModelPtr model, std::string ending)
{
	std::vector<gazebo::physics::JointPtr> joints = model->GetJoints();
	for (unsigned int i = 0; i < joints.size(); i++) {
		if (ends_with(joints[i]->GetName(), ending)) {
			return joints[i];
		}
	}
	return gazebo::physics::JointPtr();
}

void
Gripper::grabWP()
{
	grippedPuck_ = getNearestPuck();
	if (!grippedPuck_) {
		printf("No Puck found in gripper.!\n");
		return;
	}

	if (!gripperHandLink_) {
		std::cerr << "Link 'gripper_grab' not found in gripper model!" << std::endl;
		return;
	}

	//teleport puck into gripper center
	//gzwrap::Pose3d newPose = gripperHandLink_->GZWRAP_WORLD_POSE();
	//grippedPuck_->SetWorldPose(newPose);

	gazebo::physics::LinkPtr puckLink = getLinkEndingWith(grippedPuck_, "cylinder");
	if (!puckLink) {
		std::cerr << "Link 'cylinder' not found in workpiece model!" << std::endl;
		return;
	}

	puckLink->SetCollideMode("none");
	grabJoint_->Attach(gripperHandLink_, puckLink);

	sendHasPuck(true);
}

physics::ModelPtr
Gripper::getNearestPuck()
{
	physics::ModelPtr nearest;
	gzwrap::Pose3d    gripperPose = getLinkEndingWith(model_, "gripper-hand")->GZWRAP_WORLD_POSE();
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
		grippedPuck_.reset();
		return nullptr;
	}
	std::cout << "Nearest puck: " << nearest->GetName() << std::endl;
	std::cout << "Distance: " << distance << "GRAB AREA: " << RADIUS_GRAB_AREA << std::endl;
	if (distance < RADIUS_GRAB_AREA) {
		return nearest;
	} else {
		grippedPuck_.reset();
		return grippedPuck_;
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
	joint_msg.set_id(grabJoint_->GetId());
	joint_msg.set_parent_id(grabJoint_->GetId());
	if (has_puck) {
		joint_msg.set_child_id(grippedPuck_->GetId());
		joint_msg.set_child(grippedPuck_->GetName());
	} else {
		joint_msg.set_child_id(0);
		joint_msg.set_child("");
	}
	joint_pub_->Publish(joint_msg);
}

void
Gripper::setGripperMoving(bool moving)
{
	gripper_moving_ = moving;

	// update final
	msgs::Int msg;
	if (moving) {
		msg.set_data(0);
	} else if (finger_moving_ == 0) {
		msg.set_data(1);
	} else {
		return;
	}
	final_pub_->Publish(msg);
}

void
Gripper::setFingerMoving(int moving)
{
	finger_moving_ = moving;

	// update final
	msgs::Int msg;
	if (moving) {
		msg.set_data(0);
	} else if (!gripper_moving_) {
		msg.set_data(1);
	} else {
		return;
	}
	final_pub_->Publish(msg);
}

void
Gripper::sendGripperClosed(bool closed)
{
	msgs::Int msg;
	if (closed) {
		msg.set_data(1);
	} else {
		msg.set_data(0);
	}
	gripper_closed_pub_->Publish(msg);
}

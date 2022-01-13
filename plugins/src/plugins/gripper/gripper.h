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

#include <configurable/configurable.h>
#include <gazsim_msgs/GripperCommand.pb.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <fnmatch.h>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <list>
#include <queue>
#include <stdio.h>
#include <string.h>

typedef const boost::shared_ptr<gazsim_msgs::GripperCommand const> ConstGripperCommandPtr;

//config values
#define TOPIC_SET_GRIPPER config->get_string("plugins/gripper/topic-set-gripper").c_str()
#define TOPIC_HOLDS_PUCK config->get_string("plugins/gripper/topic-holds-puck").c_str()
#define TOPIC_JOINT config->get_string("plugins/gripper/topic-joint").c_str()
#define TOPIC_FINAL config->get_string("plugins/gripper/topic-final").c_str()
#define TOPIC_GRIPPER_POSE config->get_string("plugins/gripper/topic-pose").c_str()
#define TOPIC_GRIPPER_CLOSED config->get_string("plugins/gripper/topic-closed").c_str()
#define RADIUS_GRAB_AREA config->get_float("plugins/gripper/radius-grab-area")
#define GRIPPER_TOLERANCE config->get_float("plugins/gripper/gripper-tolerance")
#define GRIPPER_VELOCITY config->get_float("plugins/gripper/gripper-velocity")
#define FINGER_TOLERANCE config->get_float("plugins/gripper/finger-tolerance")
#define FINGER_OPEN_VELOCITY config->get_float("plugins/gripper/finger-open-velocity")
#define FINGER_CLOSE_VELOCITY config->get_float("plugins/gripper/finger-close-velocity")

enum ActionOnUpdate { NOTHING = 0, OPEN = 1, CLOSE = 2, MOVE = 3 } typedef ActionOnUpdate;

namespace gazebo {
/**
   * Provides gripper simulation
   * @author Stefan Profanter
   */
class Gripper : public ModelPlugin, public gazebo_rcll::ConfigurableAspect
{
public:
	Gripper();
	~Gripper();

	//Overridden ModelPlugin-Functions
	virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	virtual void OnUpdate(const common::UpdateInfo &);
	virtual void Reset();

private:
	/// Pointer to the gazbeo model
	physics::ModelPtr model_;
	physics::LinkPtr  gripperHandLink_;
	physics::JointPtr leftFingerJoint_;
	physics::JointPtr rightFingerJoint_;
	physics::JointPtr gripperJointX_;
	physics::JointPtr gripperJointY_;
	physics::JointPtr gripperJointZ_;
	/// Pointer to the update event connection
	event::ConnectionPtr update_connection_;
	///Node for communication to fawkes
	transport::NodePtr node_;
	///name of the gps and the communication channel
	std::string name_;

	physics::ModelPtr grippedPuck_;

	//Gripper Stuff:

	///Set gripper callback
	void on_set_gripper_msg(ConstGripperCommandPtr &msg);

	///Suscriber for SetGripper
	transport::SubscriberPtr set_gripper_sub_;
	/// Publisher for has_puck
	gazebo::transport::PublisherPtr has_puck_pub_;

	/// Publisher to announce which puck is hold by the gripper
	gazebo::transport::PublisherPtr joint_pub_;

	/// Publisher announces if gripper move is final
	gazebo::transport::PublisherPtr final_pub_;

	/// Publisher for gripper poses
	gazebo::transport::PublisherPtr gripper_pose_pub_;

	/// Publisher announces if gripper is closed
	gazebo::transport::PublisherPtr gripper_closed_pub_;

	gazebo::physics::JointPtr grabJoint_;

	int    finger_moving_;
	bool   gripper_moving_;
	double left_last_yaw_;
	double left_second_last_yaw_;
	double right_last_yaw_;
	double right_second_last_yaw_;
	float  gripper_target_x_;
	float  gripper_target_y_;
	float  gripper_target_z_;

	static gazebo::physics::LinkPtr  getLinkEndingWith(physics::ModelPtr model, std::string link);
	static gazebo::physics::JointPtr getJointEndingWith(physics::ModelPtr model, std::string link);

	void close();
	void open();
	void move_gripper(float x, float y, float z);

	void grabWP();
	void sendHasPuck(bool has_puck);
	void setGripperMoving(bool moving);
	void setFingerMoving(int moving);
	void sendGripperClosed(bool closed);

	gazebo::physics::ModelPtr getNearestPuck();
};
} // namespace gazebo

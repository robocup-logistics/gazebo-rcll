/***************************************************************************
 *  mps.h - Plugin to control a simulated MPS
 *
 *  Created: Fri Feb 20 17:15:34 2015
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

#ifndef MPS_H
#define MPS_H

#include "opcua_server_config.h"
#include "subclient.h"

#include <configurable/configurable.h>
#include <gazsim_msgs/NewPuck.pb.h>
#include <gazsim_msgs/WorkpieceCommand.pb.h>
#include <llsf_msgs/MachineCommands.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/MachineReport.pb.h>
#include <opc/ua/server/server.h>
#include <utils/misc/gazebo_api_wrappers.h>

#include <boost/bind.hpp>
#include <condition_variable>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <list>
#include <map>
#include <stdio.h>
#include <string.h>

typedef const boost::shared_ptr<gazsim_msgs::NewPuck const> ConstNewPuckPtr;

namespace gazebo {

enum class MachineSide {
	INPUT  = 1,
	MIDDLE = 2,
	OUTPUT = 3,
};
/**
   * Plugin to control a simulated MPS
   * @author Frederik Zwilling
   */
class Mps : public gazebo_rcll::ConfigurableAspect
{
public:
	Mps(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	virtual ~Mps();

	//add objects and variants
	void init_opcua_server();
	//set the end point and URI
	void start_server();

	virtual void OnUpdate(const common::UpdateInfo &);
	virtual void Reset();
	// with payload_in, action_id_in, ...
	virtual void process_command_in();
	// with payload_base, action_id_base, ...
	virtual void process_command_base();

	void move_conveyor(const MachineSide &side);

protected:
	// use action_id to calculate station type
	Station calculate_station_type_from_command(uint16_t value);

	void                    notify_worker();
	virtual void            worker_loop();
	std::mutex              worker_mutex_;
	std::condition_variable worker_condition_;
	bool                    shutdown_;

	static const std::map<std::string, std::string> name_id_match;

	/// Pointer to the gazbeo model
	physics::ModelPtr model_;
	/// Pointer to the update event connection
	event::ConnectionPtr update_connection_;
	///Node for communication
	transport::NodePtr node_;
	///name of the mps and the communication channel
	std::string name_;

	// Mps Stuff:

	/// Subscriber to get puck positions
	std::vector<transport::SubscriberPtr> puck_subs_;
	///// Subscriber to get machine infos
	//transport::SubscriberPtr machine_info_subscriber_;
	///// Subscriber to get machine infos
	//transport::SubscriberPtr instruct_machine_subscriber_;

	/// Handler for puck positions
	virtual void on_puck_msg(ConstPosePtr &msg);
	/// Handler for machine msgs
	//void on_machine_msg(ConstMachineInfoPtr &msg);
	/// Handler for machine Instruction msgs
	//virtual void on_instruct_machine_msg(ConstInstructMachinePtr &msg);

	//virtual void new_machine_info(ConstMachine &machine);

	transport::SubscriberPtr new_puck_subscriber_;
	virtual void             on_new_puck(ConstNewPuckPtr &msg);

	//void refbox_reply(ConstInstructMachinePtr &msg);

	///Publisher to send machine state
	transport::PublisherPtr set_machne_state_pub_;

	///Publisher to send machine reply
	transport::PublisherPtr machine_reply_pub_;

	///Publisher to send spawn machine tags
	transport::PublisherPtr visPub_;
	void   grabTag(std::string link_name, std::string tag_name, gazebo::physics::JointPtr joint);
	double spawned_tags_last_;
	double created_time_;

	///centers of input and output areas (global)
	virtual float input_x();
	virtual float input_y();
	virtual float output_x();
	virtual float output_y();

	virtual gzwrap::Pose3d middle();
	virtual gzwrap::Pose3d input();
	virtual gzwrap::Pose3d output();

	/// convert puck pose from mps frame to world frame
	gzwrap::Pose3d get_puck_world_pose(double long_side, double short_side, double height = -1.0);

	std::string current_state_;

	//void set_state(State state);

	bool
	pose_hit(const gzwrap::Pose3d &to_test, const gzwrap::Pose3d &reference, double tolerance = -1.0);

	bool puck_in_input(ConstPosePtr &pose);
	bool puck_in_output(ConstPosePtr &pose);
	bool puck_in_middle(ConstPosePtr &pose);
	bool puck_in_input(const gzwrap::Pose3d &pose);
	bool puck_in_output(const gzwrap::Pose3d &pose);
	bool puck_in_middle(const gzwrap::Pose3d &pose);

	physics::WorldPtr world_;

	std::string spawn_puck(const gzwrap::Pose3d &spawn_pose, enum gazsim_msgs::Color base_color);

	// Create a publisher on the ~/factory topic
	transport::PublisherPtr factoryPub;

	/// Publisher for puck command
	transport::PublisherPtr puck_cmd_pub_;

	transport::SubscriberPtr joint_message_sub_;
	void                     on_joint_msg(ConstJointPtr &joint_msg);

	std::map<u_int32_t, std::string> hold_pucks;
	bool                             is_puck_hold(std::string puck_name);

	//stuff for grabing the tag to the right position
	static gazebo::physics::LinkPtr  getLinkEndingWith(physics::ModelPtr model, std::string link);
	static gazebo::physics::JointPtr getJointEndingWith(physics::ModelPtr model, std::string link);
	gazebo::physics::JointPtr        tag_joint_input;
	gazebo::physics::JointPtr        tag_joint_output;
	bool                             grabbed_tags_ = false;

	//config values:
	int number_pucks_;
	//how far is the center of the belt hsifted from the machine center
	float belt_offset_side_;
	//radius of the area where a workpiece is detected by the machine
	float detect_tolerance_;
	//radius of a workpiece
	float puck_size_;
	//height of a puck
	float puck_height_;
	//length of the belt to calculate pos of input/output area
	float belt_length_;
	//Height of the belt
	float belt_height_;
	//Height of the center of the tag
	float tag_height_;
	//Height of the center of the tag
	float tag_size_;
	//At what simulation time to spawn the tag (too early and the tag spawns at (0, 0, 0))
	float       tag_spawn_time_;
	std::string topic_set_machine_state_;
	std::string topic_machine_reply_;
	std::string topic_machine_info_;
	std::string topic_instruct_machine_;
	std::string topic_puck_command_;
	std::string topic_puck_command_result_;
	std::string topic_joint_;

	physics::ModelPtr wp_in_input_;
	physics::ModelPtr wp_in_middle_;
	physics::ModelPtr wp_in_output_;

	OpcUa::UaServer opcua_server_;
	OpcUa::Node     action_id_in_;
	OpcUa::Node     barcode_in_;
	OpcUa::Node     payload1_in_;
	OpcUa::Node     payload2_in_;
	OpcUa::Node     error_in_;
	OpcUa::Node     slidecount_in_;
	OpcUa::Node     enable_in_;
	OpcUa::Node     status_error_in_;
	OpcUa::Node     status_ready_in_;
	OpcUa::Node     status_busy_in_;
	OpcUa::Node     action_id_basic_;
	OpcUa::Node     barcode_basic_;
	OpcUa::Node     payload1_basic_;
	OpcUa::Node     payload2_basic_;
	OpcUa::Node     error_basic_;
	OpcUa::Node     slidecount_basic_;
	OpcUa::Node     enable_basic_;
	OpcUa::Node     status_error_basic_;
	OpcUa::Node     status_ready_basic_;
	OpcUa::Node     status_busy_basic_;

	SubscriptionClient             sclt_in;
	OpcUa::Subscription::SharedPtr sub_in;

	SubscriptionClient             sclt_base;
	OpcUa::Subscription::SharedPtr sub_base;

	Station station_;

	// subscription handle for payloads
	uint32_t handel_action_id_in;
	uint32_t handel_action_id_base;
	uint32_t handle1_in;
	uint32_t handle2_in;
	uint32_t handle1_basic;
	uint32_t handle2_basic;

private:
	std::thread worker;
};
} // namespace gazebo

#endif // MPS_H

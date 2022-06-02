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

#include "mps.h"

#include "durations.h"

#include <opc/ua/protocol/variant.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <fnmatch.h>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <mutex>
#include <stdlib.h>

using namespace gazebo;

// Register this plugin to make it available in the simulator
//GZ_REGISTER_MODEL_PLUGIN(Mps)

const std::map<std::string, std::string> Mps::name_id_match = {
  {"C-CS1I", "tag_01"},  {"C-CS1O", "tag_02"},  {"C-CS2I", "tag_17"},  {"C-CS2O", "tag_18"},
  {"C-RS1I", "tag_33"},  {"C-RS1O", "tag_34"},  {"C-RS2I", "tag_177"}, {"C-RS2O", "tag_178"},
  {"C-BSI", "tag_65"},   {"C-BSO", "tag_66"},   {"C-DSI", "tag_81"},   {"C-DSO", "tag_82"},
  {"M-CS1I", "tag_97"},  {"M-CS1O", "tag_98"},  {"M-CS2I", "tag_113"}, {"M-CS2O", "tag_114"},
  {"M-RS1I", "tag_129"}, {"M-RS1O", "tag_130"}, {"M-RS2I", "tag_145"}, {"M-RS2O", "tag_146"},
  {"M-BSI", "tag_161"},  {"M-BSO", "tag_162"},  {"M-DSI", "tag_49"},   {"M-DSO", "tag_50"},
  {"C-SSO", "tag_194"},  {"C-SSI", "tag_193"},  {"M-SSO", "tag_210"},  {"M-SSI", "tag_209"}};

std::ostream&
gazebo::operator<<(std::ostream& os, const gazebo::MachineSide& m)
{
         return os << static_cast<int>(m);
}

///Constructor
Mps::Mps(physics::ModelPtr _parent, sdf::ElementPtr)
: shutdown_(false), model_(_parent), name_(model_->GetName()), sclt_in(this), sclt_base(this)
{
	auto sinks = spdlog::default_logger()->sinks();
	sinks.push_back(
	  std::make_shared<spdlog::sinks::basic_file_sink_mt>(fmt::format("gazebo-{}.log", name_), true));

	logger = std::make_shared<spdlog::logger>(name_, sinks.begin(), sinks.end());
	logger->set_pattern("[%c] [%^%l%$] [%n]: %v (%s:%# [%!])");
	logger->set_level(spdlog::level::debug);
	logger->flush_on(spdlog::level::info);
	SPDLOG_LOGGER_INFO(logger, "Loading Mps Plugin of model {}", name_);

	number_pucks_            = config->get_int("plugins/mps/number_pucks");
	belt_offset_side_        = config->get_float("plugins/mps/belt_offset_side");
	detect_tolerance_        = config->get_float("plugins/mps/detect_tolerance");
	puck_size_               = config->get_float("plugins/mps/puck_size");
	puck_height_             = config->get_float("plugins/mps/puck_height");
	belt_length_             = config->get_float("plugins/mps/belt_length");
	belt_height_             = config->get_float("plugins/mps/belt_height");
	mps_height_              = config->get_float("plugins/mps/mps_height");
	tag_height_              = config->get_float("plugins/mps/tag_height");
	tag_size_                = config->get_float("plugins/mps/tag_size");
	tag_spawn_time_          = config->get_float("plugins/mps/tag_spawn_time");
	topic_set_machine_state_ = config->get_string("plugins/mps/topic_set_machine_state").c_str();
	topic_machine_reply_     = config->get_string("plugins/mps/topic_machine_reply").c_str();
	topic_machine_info_      = config->get_string("plugins/mps/topic_machine_info").c_str();
	topic_instruct_machine_ =
	  config->get_string("plugins/llsf-refbox-comm/topic-instruct-machine").c_str();
	topic_puck_command_        = config->get_string("plugins/mps/topic_puck_command").c_str();
	topic_puck_command_result_ = config->get_string("plugins/mps/topic_puck_command_result").c_str();
	topic_joint_               = config->get_string("plugins/mps/topic_joint").c_str();

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->update_connection_ =
	  event::Events::ConnectWorldUpdateBegin(boost::bind(&Mps::OnUpdate, this, _1));

	//Create the communication Node for communication with fawkes
	this->node_ = transport::NodePtr(new transport::Node());
	//the namespace is set to the world name!
	this->node_->Init(model_->GetWorld()->GZWRAP_NAME());

	created_time_      = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	spawned_tags_last_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

	//subscribe to machine info
	//this->machine_info_subscriber_ =
	//  this->node_->Subscribe(topic_machine_info_, &Mps::on_machine_msg, this);

	//subscribe to machine info
	//this->instruct_machine_subscriber_ =
	//  this->node_->Subscribe(topic_instruct_machine_, &Mps::on_instruct_machine_msg, this);

	this->new_puck_subscriber_ = node_->Subscribe("~/new_puck", &Mps::on_new_puck, this);

	//Create publisher to spawn tags
	visPub_ = this->node_->Advertise<msgs::Visual>("~/visual", /*number of lights*/ 3 * 12);
	//set_machne_state_pub_ =
	//  this->node_->Advertise<llsf_msgs::SetMachineState>(topic_set_machine_state_);

	//machine_reply_pub_ = this->node_->Advertise<llsf_msgs::MachineReply>(topic_machine_reply_);
	world_ = model_->GetWorld();

	factoryPub         = node_->Advertise<msgs::Factory>("~/factory");
	puck_cmd_pub_      = node_->Advertise<gazsim_msgs::WorkpieceCommand>(topic_puck_command_);
	joint_message_sub_ = node_->Subscribe(topic_joint_, &Mps::on_joint_msg, this);

	//create joints to hold tags
	tag_joint_input = model_->GetWorld()->GZWRAP_PHYSICS()->CreateJoint("revolute", model_);
	tag_joint_input->SetName("tag_joint_input");
	tag_joint_input->SetModel(model_);

	tag_joint_output = model_->GetWorld()->GZWRAP_PHYSICS()->CreateJoint("revolute", model_);
	tag_joint_output->SetName("tag_joint_output");
	tag_joint_output->SetModel(model_);

	worker = std::thread(&Mps::worker_loop, this);
}
///Destructor
Mps::~Mps()
{
	std::unique_lock<std::mutex> lock{worker_mutex_};
	shutdown_ = true;
	lock.unlock();
	notify_worker();
	if (worker.joinable()) {
		worker.join();
	}
	opcua_server_.Stop();
	printf("Destructing Mps Plugin for %s!\n", this->name_.c_str());
}

void
Mps::start_server()
{
	opcua_server_.SetEndpoint(OpcUaConfig::get_endpoint(name_));
	opcua_server_.SetServerURI(OpcUaConfig::get_URI(station_));
	opcua_server_.Start();
	init_opcua_server();
}

void
Mps::init_opcua_server()
{
	OpcUa::Node objects = opcua_server_.GetObjectsNode();
	OpcUa::Node node    = objects.AddObject(2, "DeviceSet")
	                     .AddObject(4, "CPX-E-CEC-C1-PN")
	                     .AddObject(4, "Resources")
	                     .AddObject(4, "Application")
	                     .AddObject(3, "GlobalVars")
	                     .AddObject(4, "G");

	OpcUa::Node node_in   = node.AddObject(4, "In").AddObject(4, "p");
	action_id_in_         = node_in.AddVariable(4, "ActionId", OpcUa::Variant((uint16_t)0));
	barcode_in_           = node_in.AddVariable(4, "BarCode", OpcUa::Variant((uint16_t)0));
	auto data_node_in     = node_in.AddObject(4, "Data");
	payload1_in_          = data_node_in.AddVariable(0, "Payload1", OpcUa::Variant((uint16_t)0));
	payload2_in_          = data_node_in.AddVariable(1, "Payload2", OpcUa::Variant((uint16_t)0));
	error_in_             = node_in.AddVariable(4, "Error", OpcUa::Variant((uint8_t)0));
	slidecount_in_        = node_in.AddVariable(4, "SlideCnt", OpcUa::Variant((uint16_t)0));
	OpcUa::Node status_in = node_in.AddObject(4, "Status");
	enable_in_            = status_in.AddVariable(4, "Enable", OpcUa::Variant(false));
	status_error_in_      = status_in.AddVariable(4, "Error", OpcUa::Variant((uint8_t)0));
	status_ready_in_      = status_in.AddVariable(4, "Ready", OpcUa::Variant(false));
	status_busy_in_       = status_in.AddVariable(4, "Busy", OpcUa::Variant(false));

	OpcUa::Node node_basic = node.AddObject(4, "Basic").AddObject(4, "p");
	action_id_basic_       = node_basic.AddVariable(4, "ActionId", OpcUa::Variant((uint16_t)0));
	barcode_basic_         = node_basic.AddVariable(4, "BarCode", OpcUa::Variant((uint16_t)0));
	auto data_node_basic   = node_basic.AddObject(4, "Data");
	payload1_basic_        = data_node_basic.AddVariable(0, "Payload1", OpcUa::Variant((uint16_t)0));
	payload2_basic_        = data_node_basic.AddVariable(1, "Payload2", OpcUa::Variant((uint16_t)0));
	error_basic_           = node_basic.AddVariable(4, "Error", OpcUa::Variant((uint8_t)0));
	slidecount_basic_      = node_basic.AddVariable(4, "SlideCnt", OpcUa::Variant((uint16_t)0));
	OpcUa::Node status_basic = node_basic.AddObject(4, "Status");
	enable_basic_            = status_basic.AddVariable(4, "Enable", OpcUa::Variant(false));
	status_error_basic_      = status_basic.AddVariable(4, "Error", OpcUa::Variant((uint8_t)0));
	status_ready_basic_      = status_basic.AddVariable(4, "Ready", OpcUa::Variant(false));
	status_busy_basic_       = status_basic.AddVariable(4, "Busy", OpcUa::Variant(false));

	sclt_in.set_callback_funk(&Mps::notify_worker);
	sub_in              = opcua_server_.CreateSubscription(100, sclt_in);
	handel_action_id_in = sub_in->SubscribeDataChange(action_id_in_);

	sclt_base.set_callback_funk(&Mps::notify_worker);
	sub_base              = opcua_server_.CreateSubscription(100, sclt_base);
	handel_action_id_base = sub_base->SubscribeDataChange(action_id_basic_);
}

void
Mps::process_command_in()
{
	uint16_t value = uint16_t(action_id_in_.GetValue());
	if (value == 0) {
		return;
	}
	SPDLOG_LOGGER_DEBUG(logger, "Processing command {}", value);
	if (calculate_station_type_from_command(value) != station_) {
		SPDLOG_LOGGER_INFO(logger, "Different station");
		return;
	}
	if (value < station_) {
		if (value != 0) {
			SPDLOG_LOGGER_WARN(logger, "Unexpected action id {}", value);
		}
		return;
	}
	Operation op = Operation(value - station_);
	SPDLOG_LOGGER_DEBUG(logger, "Processing op {}", op);
	switch (op) {
	case Operation::OPERATION_MOVE_CONVEYOR:
		move_conveyor(MachineSide((uint16_t)payload1_in_.GetValue()));
		break;
	default: SPDLOG_LOGGER_DEBUG(logger, "Operation {}  is not implemented", op);
	}
}

void
Mps::process_command_base()
{
}

void
Mps::move_conveyor(const MachineSide &side)
{
	status_busy_in_.SetValue(true);
	std::this_thread::sleep_for(move_duration);
	gzwrap::Pose3d    target_pose = output();
	physics::ModelPtr wp;
	switch (side) {
	case MachineSide::INPUT:
		wp = wp_in_middle_;
		if (!wp) {
			SPDLOG_LOGGER_WARN(logger, "No workpiece in machine's middle ({})", middle());
			return;
		}
		SPDLOG_LOGGER_INFO(logger, "Moving workpiece {} from middle to input", wp->GetName());
		target_pose = input();
		break;
	case MachineSide::MIDDLE:
		wp = wp_in_input_;
		if (!wp) {
			SPDLOG_LOGGER_WARN(logger, "No workpiece in machine's input ({})", input());
			return;
		}
		SPDLOG_LOGGER_INFO(logger, "Moving workpiece {} from input to middle", wp->GetName());
		target_pose = middle();
		break;
	case MachineSide::OUTPUT:
		wp = wp_in_middle_;
		if (!wp) {
			SPDLOG_LOGGER_WARN(logger, "No workpiece in machine's middle ({})", middle());
			return;
		}
		SPDLOG_LOGGER_INFO(logger, "Moving workpiece {} from middle to output", wp->GetName());
		target_pose = output();
		break;
	default:
		SPDLOG_LOGGER_WARN(logger,
		                   "Unexpected side {}, expected one of {}, {}, {}",
		                   side,
		                   MachineSide::INPUT,
		                   MachineSide::MIDDLE,
		                   MachineSide::OUTPUT);
		return;
	}
	wp->SetWorldPose(target_pose);
	action_id_in_.SetValue((uint16_t)0);
	payload1_in_.SetValue((uint16_t)0);
	switch (side) {
	case MachineSide::INPUT:
		wp_in_middle_.reset();
		wp_in_input_ = wp;
		status_ready_in_.SetValue(true);
		break;
	case MachineSide::MIDDLE:
		wp_in_input_.reset();
		wp_in_middle_ = wp;
		break;
	case MachineSide::OUTPUT:
		wp_in_middle_.reset();
		wp_in_output_ = wp;
		status_ready_in_.SetValue(true);
		break;
	}
	status_busy_in_.SetValue(false);
}

Station
Mps::calculate_station_type_from_command(uint16_t value)
{
	return Station(value - (value % 100));
}

void
Mps::notify_worker()
{
	worker_condition_.notify_one();
}

void
Mps::worker_loop()
{
	while (!shutdown_) {
		std::unique_lock<std::mutex> lock{worker_mutex_};
		worker_condition_.wait(lock);
		lock.unlock();
		process_command_base();
		process_command_in();
	}
}

/** Called by the world update start event
 */
void
Mps::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	if (!grabbed_tags_) {
		std::string input_tag_name  = name_id_match.at(name_ + "I");
		std::string output_tag_name = name_id_match.at(name_ + "O");

		physics::BasePtr input_tag  = model_->GetWorld()->GZWRAP_BASE_BY_NAME(input_tag_name);
		physics::BasePtr output_tag = model_->GetWorld()->GZWRAP_BASE_BY_NAME(output_tag_name);

		if (input_tag && output_tag) {
			//Spawn tags (in Init is to early because it would be spawned at origin)
			printf("***** %s: Grabbing Tags\n", name_.c_str());
			grabTag("mps_tag_input", name_ + "I", tag_joint_input);
			grabTag("mps_tag_output", name_ + "O", tag_joint_output);
			grabbed_tags_ = true;
		} else {
			printf("***** %s: Tags not there, yet\n", name_.c_str());
		}
	}
}

/** on Gazebo reset
 */
void
Mps::Reset()
{
}

/** Functions for recieving puck locations Messages
 * @param msg message
 */
void
Mps::on_puck_msg(ConstPosePtr &msg)
{
	if (!wp_in_input_ && puck_in_input(msg)) {
		wp_in_input_ = world_->ModelByName(msg->name());
		if (!wp_in_input_) {
			SPDLOG_LOGGER_WARN(logger, "Workpiece {} is input, but could not find model!", msg->name());
		} else {
			SPDLOG_LOGGER_INFO(logger, "Found workpiece {} in input", wp_in_input_->GetName());
		}
	} else if (wp_in_input_ && msg->name() == wp_in_input_->GetName() && !puck_in_input(msg)) {
		SPDLOG_LOGGER_INFO(logger, "Workpiece {} no longer in input", wp_in_input_->GetName());
		wp_in_input_.reset();
		status_ready_in_.SetValue(false);
	} else if (wp_in_output_ && msg->name() == wp_in_output_->GetName() && !puck_in_output(msg)) {
		SPDLOG_LOGGER_INFO(logger, "Workpiece {} no longer in output", wp_in_output_->GetName());
		wp_in_output_.reset();
		status_ready_in_.SetValue(false);
	}
}

/**
 * Find the tag with the id matching to the tag_name (e.g. C-BSI), grap it to mount it at the side of the mps (where the link link_name is placed)
 */
void
Mps::grabTag(std::string link_name, std::string tag_name, gazebo::physics::JointPtr joint)
{
	//get tag_id from tag_name
	tag_name = name_id_match.at(tag_name);

	//get link of mps
	gazebo::physics::LinkPtr gripperLink = getLinkEndingWith(model_, link_name.c_str());
	if (!gripperLink) {
		printf("MPS: can't find mps link %s to attach tag\n", link_name.c_str());
		return;
	}

	//find link of tag
	gazebo::physics::ModelPtr tag = world_->GZWRAP_MODEL_BY_NAME(tag_name);
	if (!tag) {
		printf("MPS: can't find tag with name %s\n", tag_name.c_str());
		return;
	}
	gazebo::physics::LinkPtr tagLink = getLinkEndingWith(tag, "link");
	if (!tagLink) {
		printf("MPS: can't find link of tag with name %s\n", tag_name.c_str());
		return;
	}

	//teleport tag to right position
	gzwrap::Pose3d gripperPose = gripperLink->GZWRAP_WORLD_POSE();
	printf("Teleport Tag to %f\n", gripperPose.GZWRAP_POS.GZWRAP_X);
	tag->SetWorldPose(gripperPose);

	joint->Load(gripperLink, tagLink, gzwrap::Pose3d());
	joint->Attach(gripperLink, tagLink);

	joint->SetAxis(0, gzwrap::Vector3d::UnitZ);

#if GAZEBO_MAJOR_VERSION >= 8
	joint->SetUpperLimit(0, 0);
	joint->SetLowerLimit(0, 0);
#else
	joint->SetHighStop(0, gazebo::math::Angle(0.0f));
	joint->SetLowStop(0, gazebo::math::Angle(0.0f));
#endif

	// printf("MPS %s: attached tag %s\n", name_.c_str(), tag_name.c_str());
}

//compute locations of input and output (not sure about the sides jet)
float
Mps::output_x()
{
	double mps_x   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_X;
	double mps_ori = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
	return mps_x + belt_offset_side_ * cos(mps_ori) + (belt_length_ / 2 - puck_size_) * sin(mps_ori);
}

float
Mps::output_y()
{
	double mps_y   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_Y;
	double mps_ori = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
	return mps_y + belt_offset_side_ * sin(mps_ori) - (belt_length_ / 2 - puck_size_) * cos(mps_ori);
}

float
Mps::input_x()
{
	double mps_x   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_X;
	double mps_ori = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
	return mps_x + belt_offset_side_ * cos(mps_ori) - (belt_length_ / 2 - puck_size_) * sin(mps_ori);
}

float
Mps::input_y()
{
	double mps_y   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_Y;
	double mps_ori = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
	return mps_y + belt_offset_side_ * sin(mps_ori) + (belt_length_ / 2 - puck_size_) * cos(mps_ori);
}

gzwrap::Pose3d
Mps::input()
{
	return gzwrap::Pose3d(input_x(), input_y(), belt_height_, 0, 0, 0);
}

gzwrap::Pose3d
Mps::output()
{
	return gzwrap::Pose3d(output_x(), output_y(), belt_height_, 0, 0, 0);
}

gzwrap::Pose3d
Mps::middle()
{
	return gzwrap::Pose3d(
	  (input_x() + output_x()) / 2, (input_y() + output_y()) / 2, belt_height_, 0, 0, 0);
}

bool
Mps::pose_hit(const gzwrap::Pose3d &to_test, const gzwrap::Pose3d &reference, double tolerance)
{
	if (tolerance == -1.0)
		tolerance = detect_tolerance_;
	return (to_test.GZWRAP_POS - reference.GZWRAP_POS).GZWRAP_LENGTH() < tolerance;
}

bool
Mps::puck_in_input(ConstPosePtr &pose)
{
	double dist =
	  sqrt((pose->position().x() - input_x()) * (pose->position().x() - input_x())
	       + (pose->position().y() - input_y()) * (pose->position().y() - input_y())
	       + (pose->position().z() - belt_height_) * (pose->position().z() - belt_height_));
	return dist < detect_tolerance_;
}

bool
Mps::puck_in_output(ConstPosePtr &pose)
{
	double dist =
	  sqrt((pose->position().x() - output_x()) * (pose->position().x() - output_x())
	       + (pose->position().y() - output_y()) * (pose->position().y() - output_y())
	       + (pose->position().z() - belt_height_) * (pose->position().z() - belt_height_));
	return dist < detect_tolerance_;
}

bool
Mps::puck_in_middle(ConstPosePtr &pose)
{
	return puck_in_middle(gazebo::msgs::ConvertIgn(*pose));
}

bool
Mps::puck_in_input(const gzwrap::Pose3d &pose)
{
	return (pose.GZWRAP_POS - input().GZWRAP_POS).GZWRAP_LENGTH() < detect_tolerance_;
}

bool
Mps::puck_in_output(const gzwrap::Pose3d &pose)
{
	return (pose.GZWRAP_POS - output().GZWRAP_POS).GZWRAP_LENGTH() < detect_tolerance_;
}

bool
Mps::puck_in_middle(const gzwrap::Pose3d &pose)
{
	return (pose.GZWRAP_POS - middle().GZWRAP_POS).GZWRAP_LENGTH() < detect_tolerance_;
}

void
Mps::on_new_puck(ConstNewPuckPtr &msg)
{
	this->puck_subs_.push_back(this->node_->Subscribe(msg->gps_topic(), &Mps::on_puck_msg, this));
}

std::string
Mps::spawn_puck(const gzwrap::Pose3d &spawn_pose, gazsim_msgs::Color base_color)
{
	printf("spawning puck for %s\n", name_.c_str());
	msgs::Factory new_puck_msg;

	//use the workpiece_base sdf and replace the model name
	//get the new puck name
	std::string new_name = "puck_" + std::to_string(rand() % 1000000);
	//Get sdf content
	std::string sdf_path = getenv("GAZEBO_RCLL");
	sdf_path += "/models/workpiece_base/model.sdf";
	std::ifstream raw_sdf_file(sdf_path.c_str());
	//exchange name
	std::string new_sdf;
	if (raw_sdf_file.is_open()) {
		std::string raw_sdf((std::istreambuf_iterator<char>(raw_sdf_file)),
		                    std::istreambuf_iterator<char>());
		std::string old_name = "workpiece_base";
		std::size_t name_pos = raw_sdf.find(old_name);
		if (name_pos == std::string::npos) {
			return "";
		}
		new_sdf               = raw_sdf.erase(name_pos, old_name.length()).insert(name_pos, new_name);
		std::string old_color = "1.0 0.35 0.0 1";
		std::string new_color;
		std::string color_string;
		switch (base_color) {
		case gazsim_msgs::Color::RED:
			new_color    = "1.0 0.0 0.0 1";
			color_string = "RED";
			break;
		case gazsim_msgs::Color::BLACK:
			new_color    = "0.2 0.2 0.2 1";
			color_string = "BLACK";
			break;
		case gazsim_msgs::Color::SILVER:
			new_color    = "0.8 0.8 0.8 1";
			color_string = "SILVER";
			break;
		default:
			printf("%s should spawn with an unsupported base color %s\n",
			       new_name.c_str(),
			       gazsim_msgs::Color_Name(base_color).c_str());
			return "";
			break;
		}
		std::size_t color_pos;
		while ((color_pos = new_sdf.find(old_color)) != std::string::npos) {
			new_sdf   = new_sdf.erase(color_pos, old_color.length()).insert(color_pos, new_color);
			color_pos = new_sdf.find(old_color);
		}
		std::string plugin_string = "<plugin name=\"Puck\" filename=\"libpuck.so\"/>";
		std::size_t string_pos    = new_sdf.find(plugin_string);
		new_sdf                   = new_sdf.erase(string_pos, plugin_string.length())
		            .insert(string_pos,
		                    "<plugin name=\"Puck\" filename=\"libpuck.so\"><baseColor>" + color_string
		                      + "</baseColor></plugin>");

	} else {
		printf("Cant find workpiece_base sdf file:%s", sdf_path.c_str());
		return "";
	}

	new_puck_msg.set_sdf(new_sdf.c_str());
	new_puck_msg.set_clone_model_name(new_name.c_str());
#if GAZEBO_MAJOR_VERSION > 5 && GAZEBO_MAJOR_VERSION < 8
	msgs::Set(new_puck_msg.mutable_pose(), spawn_pose.Ign());
#else
	msgs::Set(new_puck_msg.mutable_pose(), spawn_pose);
#endif
	factoryPub->Publish(new_puck_msg);
	return new_name;
}

gzwrap::Pose3d
Mps::get_puck_world_pose(double long_side, double short_side, double height)
{
	if (height == -1.0)
		height = belt_height_;

	double mps_x   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_X;
	double mps_y   = this->model_->GZWRAP_WORLD_POSE().GZWRAP_POS_Y;
	double mps_ori = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;

	double x = mps_x + (belt_offset_side_ + long_side) * cos(mps_ori)
	           - ((belt_length_ + short_side) / 2 - puck_size_) * sin(mps_ori);
	double y = mps_y + (belt_offset_side_ + long_side) * sin(mps_ori)
	           + ((belt_length_ + short_side) / 2 - puck_size_) * cos(mps_ori);

	return gzwrap::Pose3d(x, y, height, 0, 0, 0);
}

void
Mps::on_joint_msg(ConstJointPtr &joint_msg)
{
	hold_pucks[joint_msg->id()] = joint_msg->child();
	//printf("%s got joint command on joint %i with child %s\n", name_.c_str(), joint_msg->id(), joint_msg->child().c_str());
}

bool
Mps::is_puck_hold(std::string puck_name)
{
	//printf("%s is testing if %s is hold\n", name_.c_str(), puck_name.c_str());
	for (auto iter : hold_pucks) {
		//printf("%s is testing for %s\n", name_.c_str(), iter.second.c_str());
		if (puck_name == iter.second.c_str()) {
			//printf("%s is found %s in hold\n", name_.c_str(), iter.second.c_str());
			return true;
		}
	}
	//printf("%s did not find %s", name_.c_str(), puck_name.c_str());
	return false;
}

inline bool
ends_with(std::string const &value, std::string const &ending)
{
	if (ending.size() > value.size())
		return false;
	return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

gazebo::physics::LinkPtr
Mps::getLinkEndingWith(physics::ModelPtr model, std::string ending)
{
	std::vector<gazebo::physics::LinkPtr> links = model->GetLinks();
	for (unsigned int i = 0; i < links.size(); i++) {
		if (ends_with(links[i]->GetName(), ending))
			return links[i];
	}
	return gazebo::physics::LinkPtr();
}

gazebo::physics::JointPtr
Mps::getJointEndingWith(physics::ModelPtr model, std::string ending)
{
	std::vector<gazebo::physics::JointPtr> joints = model->GetJoints();
	for (unsigned int i = 0; i < joints.size(); i++) {
		if (ends_with(joints[i]->GetName(), ending))
			return joints[i];
	}
	return gazebo::physics::JointPtr();
}

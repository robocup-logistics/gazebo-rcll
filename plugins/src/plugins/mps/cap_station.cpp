/***************************************************************************
 *  cap_station.h - controls a cap station mps
 *
 *  Generated: Wed Apr 22 12:48:29 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "cap_station.h"

#include "durations.h"

#include <utils/misc/gazebo_api_wrappers.h>

using namespace gazebo;

CapStation::CapStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) : Mps(_parent, _sdf)
{
	station_ = Station::STATION_CAP;
	start_server();
	spawn_three_pucks();
	workpiece_result_subscriber_ =
	  node_->Subscribe(topic_puck_command_result_, &CapStation::on_puck_result, this);
	stored_cap_color_  = gazsim_msgs::Color::NONE;
	puck_spawned_time_ = created_time_;
}

void
CapStation::process_command_in()
{
	Mps::process_command_in();

	uint16_t value = uint16_t(action_id_in_.GetValue());
	if (value == 0) {
		return;
	}
	if (calculate_station_type_from_command(value) != station_) {
		return;
	}
	Operation oper = Operation(value - station_);
	if (oper != Operation::OPERATION_CAP_ACTION) {
		//SPDLOG_LOGGER_WARN(logger, "Unexpected operation {} on station {}", oper, station_);
		return;
	}
	auto op = Operation(uint16_t(payload1_in_.GetValue()));
	switch (op) {
	case Operation::OPERATION_CAP_RETRIEVE: retrieve_cap(); break;
	case Operation::OPERATION_CAP_MOUNT: mount_cap(); break;
	default: SPDLOG_LOGGER_WARN(logger, "Unexpected Op while processing workpiece: {}", op); break;
	}
}

void
CapStation::mount_cap()
{
	if (!wp_in_middle_) {
		SPDLOG_LOGGER_WARN(logger, "Cannot mount cap, no workpiece in the middle!");
		return;
	}
	if (!puck_in_middle(wp_in_middle_->WorldPose())) {
		SPDLOG_LOGGER_WARN(logger,
		                   "Cannot mount cap, workpiece {} should be in the middle but is not",
		                   wp_in_middle_->GetName());
		return;
	}
	SPDLOG_LOGGER_INFO(logger, "Mounting cap");
	status_busy_in_.SetValue(true);
	if (stored_cap_color_ != gazsim_msgs::Color::NONE) {
		SPDLOG_LOGGER_INFO(logger,
		                   "{} mounts cap on {} with color {}",
		                   name_,
		                   wp_in_middle_->GetName(),
		                   gazsim_msgs::Color_Name(stored_cap_color_));

		gazsim_msgs::WorkpieceCommand cmd_msg = gazsim_msgs::WorkpieceCommand();
		cmd_msg.set_puck_name(wp_in_middle_->GetName());

		cmd_msg.set_command(gazsim_msgs::Command::ADD_CAP);
		cmd_msg.add_color(stored_cap_color_);

		gazebo::msgs::Visual vis_msg;
		vis_msg.set_parent_name(name_ + "::body");
		vis_msg.set_name(name_ + "::body::have_cap");
		gazebo::msgs::Set(vis_msg.mutable_material()->mutable_diffuse(), gzwrap::Color(0.3, 0, 0));
		visPub_->Publish(vis_msg);
		puck_cmd_pub_->Publish(cmd_msg);
		stored_cap_color_ = gazsim_msgs::Color::NONE;
	} else {
		SPDLOG_LOGGER_WARN(logger, "{} can't mount cap without a cap loaded first", name_);
	}
	action_id_in_.SetValue((uint16_t)0);
	payload1_in_.SetValue((uint16_t)0);
	std::this_thread::sleep_for(cap_op_duration);
	status_busy_in_.SetValue(false);
}

void
CapStation::retrieve_cap()
{
	if (!wp_in_middle_) {
		SPDLOG_LOGGER_WARN(logger, "Cannot retrieve cap, no workpiece in the middle!");
		return;
	}
	SPDLOG_LOGGER_INFO(logger, "Retrieving cap");
	status_busy_in_.SetValue(true);
	gazsim_msgs::WorkpieceCommand cmd_msg = gazsim_msgs::WorkpieceCommand();
	cmd_msg.set_puck_name(wp_in_middle_->GetName());
	SPDLOG_LOGGER_INFO(logger, "{} retrieves cap from {}", name_, wp_in_middle_->GetName());
	cmd_msg.set_command(gazsim_msgs::Command::REMOVE_CAP);
	puck_cmd_pub_->Publish(cmd_msg);
	action_id_in_.SetValue((uint16_t)0);
	payload1_in_.SetValue((uint16_t)0);
	std::this_thread::sleep_for(cap_op_duration);
	status_busy_in_.SetValue(false);
}

void
CapStation::spawn_three_pucks()
{
	spawn_puck(shelf_left_pose(), gazsim_msgs::Color::RED);
	spawn_puck(shelf_middle_pose(), gazsim_msgs::Color::RED);
	spawn_puck(shelf_right_pose(), gazsim_msgs::Color::RED);
	init_cap_state[0] = false;
	init_cap_state[1] = false;
	init_cap_state[2] = false;
}
void
CapStation::init_caps()
{
	gazsim_msgs::Color _color = gazsim_msgs::Color::NONE;

	if (name_.find("CS1") != std::string::npos) {
		_color = gazsim_msgs::Color::GREY;
	} else if (name_.find("CS2") != std::string::npos) {
		_color = gazsim_msgs::Color::BLACK;
	}

	if (_color != gazsim_msgs::Color::NONE) {
		physics::ModelPtr puck_list[3]  = {puck_in_shelf_left_,
                                      puck_in_shelf_middle_,
                                      puck_in_shelf_right_};
		gzwrap::Pose3d puck_pos_list[3] = {shelf_left_pose(), shelf_middle_pose(), shelf_right_pose()};
		for (int i = 0; i < 3; i++) {
			if (!init_cap_state[i] && puck_list[i]) {
				gazsim_msgs::WorkpieceCommand cmd;
				cmd.set_command(gazsim_msgs::Command::ADD_CAP);
				cmd.add_color(_color);
				cmd.set_puck_name(puck_list[i]->GetName());
				puck_cmd_pub_->Publish(cmd);
				init_cap_state[i] = true;
				break;
			}
		}
	}
}

void
CapStation::OnUpdate(const common::UpdateInfo &info)
{
	//updata_time = (updata_time + 1) % 3;
	Mps::OnUpdate(info);
	if (model_->GetWorld()->GZWRAP_SIM_TIME().Double() - puck_spawned_time_ < SPAWN_PUCK_TIME) {
		return;
	}

	if (puck_in_shelf_left_
	    && !pose_hit(puck_in_shelf_left_->GZWRAP_WORLD_POSE(), shelf_left_pose(), 0.1))
		puck_in_shelf_left_ = nullptr;
	if (puck_in_shelf_middle_
	    && !pose_hit(puck_in_shelf_middle_->GZWRAP_WORLD_POSE(), shelf_middle_pose(), 0.1))
		puck_in_shelf_middle_ = nullptr;
	if (puck_in_shelf_right_
	    && !pose_hit(puck_in_shelf_right_->GZWRAP_WORLD_POSE(), shelf_right_pose(), 0.1))
		puck_in_shelf_right_ = nullptr;
	if (!puck_in_shelf_right_ && !puck_in_shelf_middle_ && !puck_in_shelf_left_) {
		//shelf is empty -> refill
		spawn_three_pucks();
		puck_spawned_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	}
	init_caps();
}

void
CapStation::on_new_puck(ConstNewPuckPtr &msg)
{
	Mps::on_new_puck(msg);

	//get model
	physics::Model_V models = world_->GZWRAP_MODELS();
	for (physics::ModelPtr model : models) {
		if (model->GetName() == msg->puck_name()) {
			if (puck_in_shelf_left_ != model && pose_in_shelf_left(model->GZWRAP_WORLD_POSE())) {
				puck_in_shelf_left_ = model;
			} else if (puck_in_shelf_middle_ != model
			           && pose_in_shelf_middle(model->GZWRAP_WORLD_POSE())) {
				puck_in_shelf_middle_ = model;
			} else if (puck_in_shelf_right_ != model && pose_in_shelf_right(model->GZWRAP_WORLD_POSE())) {
				puck_in_shelf_right_ = model;
			}
		}
	}
}

void
CapStation::on_puck_result(ConstWorkpieceResultPtr &result)
{
	printf("CAPSTATION: on_puck_result: %s\n", result->puck_name().c_str());

	if (wp_in_middle_ && result->puck_name() == wp_in_middle_->GetName()) {
		printf("%s got cap from %s with color %s\n",
		       name_.c_str(),
		       result->puck_name().c_str(),
		       gazsim_msgs::Color_Name(result->color()).c_str());
		if (result->color() != gazsim_msgs::Color::NONE) {
			stored_cap_color_ = result->color();
			gazebo::msgs::Visual vis_msg;
			vis_msg.set_parent_name(name_ + "::body");
			vis_msg.set_name(name_ + "::body::have_cap");
			gazebo::msgs::Set(vis_msg.mutable_material()->mutable_diffuse(), gzwrap::Color(1, 0, 0));
			visPub_->Publish(vis_msg);
		}
	}
}

gzwrap::Pose3d
CapStation::shelf_left_pose()
{
	return get_puck_world_pose(-0.1, 0, belt_height_ + 0.005);
}

gzwrap::Pose3d
CapStation::shelf_middle_pose()
{
	return get_puck_world_pose(-0.2, 0, belt_height_ + 0.005);
}

gzwrap::Pose3d
CapStation::shelf_right_pose()
{
	return get_puck_world_pose(-0.3, 0, belt_height_ + 0.005);
}

bool
CapStation::pose_in_shelf_left(const gzwrap::Pose3d &puck_pose)
{
	return pose_hit(puck_pose, shelf_left_pose(), 0.05);
}

bool
CapStation::pose_in_shelf_middle(const gzwrap::Pose3d &puck_pose)
{
	return pose_hit(puck_pose, shelf_middle_pose(), 0.05);
}

bool
CapStation::pose_in_shelf_right(const gzwrap::Pose3d &puck_pose)
{
	return pose_hit(puck_pose, shelf_right_pose(), 0.05);
}

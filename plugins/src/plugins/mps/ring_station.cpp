/***************************************************************************
 *  ring_station.h - controls a ring station mps
 *
 *  Generated: Wed Apr 22 13:48:39 2015
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

#include "ring_station.h"

#include "durations.h"

using namespace gazebo;

RingStation::RingStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) : Mps(_parent, _sdf)
{
	station_ = Station::STATION_RING;
	start_server();
	add_base_publisher_ = node_->Advertise<llsf_msgs::MachineAddBase>(TOPIC_MACHINE_ADD_BASE);
	number_bases_       = 0;
}

void
RingStation::process_command_in()
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
	if (oper != Operation::OPERATION_MOUNT_RING) {
		return;
	}

	auto feeder   = uint16_t(payload1_in_.GetValue());
	auto payload2 = uint16_t(payload2_in_.GetValue());
	action_id_in_.SetValue(uint16_t(0));
	payload1_in_.SetValue(uint16_t(0));
	payload2_in_.SetValue(uint16_t(0));
	gazsim_msgs::Color color;
	if (payload2 == 1) {
		color = gazsim_msgs::Color::BLUE;
	} else if (payload2 == 2) {
		color = gazsim_msgs::Color::GREEN;
	} else if (payload2 == 3) {
		color = gazsim_msgs::Color::ORANGE;
	} else if (payload2 == 4) {
		color = gazsim_msgs::Color::YELLOW;
	} else {
		SPDLOG_LOGGER_WARN(logger,
		                   "Unexpected ring color {}, expected a value in the range [1,4]",
		                   color);
	}

	if (feeder != 1 && feeder != 2) {
		SPDLOG_LOGGER_WARN(logger, "Unexpected feeder {}, expected 1 or 2", feeder);
		return;
	}
	mount_ring(color);
}

void
RingStation::mount_ring(gazsim_msgs::Color color)
{
	if (!wp_in_middle_) {
		SPDLOG_LOGGER_WARN(logger, "Cannot mount ring, no workpiece in the middle!");
		return;
	}
	if (!puck_in_middle(wp_in_middle_->WorldPose())) {
		SPDLOG_LOGGER_WARN(logger,
		                   "Cannot mount ring, workpiece {} should be in the middle but is not",
		                   wp_in_middle_->GetName());
		return;
	}
	SPDLOG_LOGGER_INFO(logger, "Mounting ring");
	status_busy_in_.SetValue(true);
	gazsim_msgs::WorkpieceCommand cmd;
	cmd.set_command(gazsim_msgs::Command::ADD_RING);
	cmd.add_color(color);
	cmd.set_puck_name(wp_in_middle_->GetName());
	puck_cmd_pub_->Publish(cmd);
	std::this_thread::sleep_for(ring_op_duration);
	status_busy_in_.SetValue(false);
}

void
RingStation::publish_indicator(bool active, int number)
{
	gazebo::msgs::Visual msg;
	msg.set_parent_name(name_ + "::body");
	msg.set_name(name_ + "::body::base_" + std::to_string(number));
#if GAZEBO_MAJOR_VERSION > 5
	gazebo::msgs::Set(
	  msg.mutable_pose(),
	  ignition::math::Pose3d(-0.35 + (number * 0.11), 0, belt_height_ + 0.3, 0, 0, 0));
#else
	gazebo::msgs::Set(msg.mutable_pose(),
	                  gazebo::math::Pose(-0.35 + (number * 0.11), 0, belt_height_ + 0.3, 0, 0, 0));
#endif
	if (active) {
		msgs::Set(msg.mutable_material()->mutable_diffuse(), gzwrap::Color(1, 0, 0));
	} else {
		msgs::Set(msg.mutable_material()->mutable_diffuse(), gzwrap::Color(.3f, 0, 0));
	}
	visPub_->Publish(msg);
}

bool
RingStation::puck_on_slide(const gzwrap::Pose3d &pose)
{
	return (pose.GZWRAP_POS - add_base_pose().GZWRAP_POS).GZWRAP_LENGTH() < detect_tolerance_;
}

bool
RingStation::puck_on_slide(ConstPosePtr &pose)
{
	return puck_on_slide(gazebo::msgs::ConvertIgn(*pose));
}

void
RingStation::on_puck_msg(ConstPosePtr &msg)
{
	Mps::on_puck_msg(msg);
	if (puck_on_slide(msg)
	    && std::find(begin(wps_on_slide_), end(wps_on_slide_), msg->name()) == wps_on_slide_.end()) {
		SPDLOG_LOGGER_INFO(logger, "Adding base to ring station {}", name_);
		wps_on_slide_.insert(msg->name());
		slidecount_in_.SetValue(uint16_t(wps_on_slide_.size()));
	}
}

void
RingStation::add_base()
{
	// TODO reimplement
	printf("Adding Base to %s\n", name_.c_str());
	//llsf_msgs::MachineAddBase add_base_msg;
	//add_base_msg.set_machine_name(name_);
	//add_base_publisher_->Publish(add_base_msg);
	//publish_indicator(true, number_bases_++);
}

gzwrap::Pose3d
RingStation::add_base_pose()
{
	return get_puck_world_pose(-0.25, 0, mps_height_);
}

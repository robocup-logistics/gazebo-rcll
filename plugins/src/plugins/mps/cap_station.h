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

#ifndef CAP_STATION_H
#define CAP_STATION_H

#include "mps.h"

#define SPAWN_PUCK_TIME config->get_int("plugins/mps/cap-station/spawn_puck_time")
#define PROB_SHELF_BROKEN config->get_float("plugins/mps/cap-station/prob-shelf-broken")
#define PROB_CS_SLIDE_INPUT_BROKEN config->get_float("plugins/mps/cap-station/prob-slide-input-broken")
#define PROB_CS_SLIDE_OUTPUT_BROKEN config->get_float("plugins/mps/cap-station/prob-slide-output-broken")
#define REBREAK_CS_INTERVAL config->get_float("plugins/mps/cap-station/rebreak-interval")

typedef const boost::shared_ptr<const gazsim_msgs::WorkpieceResult> ConstWorkpieceResultPtr;

namespace gazebo {

class CapStation : public Mps
{
public:
  CapStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
  void on_puck_msg(ConstPosePtr &msg);
  void on_new_puck(ConstNewPuckPtr &msg);
  void OnUpdate(const common::UpdateInfo &info);
  void new_machine_info(ConstMachine &machine);
  void on_instruct_machine_msg(ConstInstructMachinePtr &msg);
  void on_puck_result(ConstWorkpieceResultPtr &result);
  
  gzwrap::Pose3d shelf_left_pose();
  gzwrap::Pose3d shelf_middle_pose();
  gzwrap::Pose3d shelf_right_pose();
  
  bool pose_in_shelf_left(const gzwrap::Pose3d &puck_pose);
  bool pose_in_shelf_middle(const gzwrap::Pose3d &puck_pose);
  bool pose_in_shelf_right(const gzwrap::Pose3d &puck_pose);

  bool pose_in_front;

  physics::ModelPtr puck_in_shelf_left_;
  physics::ModelPtr puck_in_shelf_middle_;
  physics::ModelPtr puck_in_shelf_right_;
  
  llsf_msgs::CSOp task_;
  gazsim_msgs::Color stored_cap_color_;
  std::string puck_in_processing_name_;
  
  transport::SubscriberPtr workpiece_result_subscriber_;
  
  double puck_spawned_time_;
  
  void work_puck(std::string puck_name);
private:
  bool shelfBroken;
  //this function generates random values to decide about the broken state of the cap_Station
  void decide_broken_state();
  //this function unlocks the pucks on the shelf, if necessary
  void clean_up_shelf();
  //this function unlocks and remove the puck in the output
  void clean_up_output();
  //this function removes the puck from the input
  void clean_up_input();
  //this function locks all pucks on the shelf of the cap station
  void shelf_lock();
  common::Time last_time_rebreak_;
};

}

#endif // CAP_STATION_H

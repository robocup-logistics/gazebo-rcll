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

#ifndef RING_STATION_H
#define RING_STATION_H

#define TOPIC_MACHINE_ADD_BASE config->get_string("plugins/mps/ring-station/topic_machine_add_base").c_str()
#define MAX_NUM_BASES config->get_int("plugins/mps/ring-station/max_num_bases")

#include "mps.h"

namespace gazebo {

class RingStation : public Mps
{
public:
  RingStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
  void on_puck_msg(ConstPosePtr &msg);
  
  void new_machine_info(ConstMachine &machine);
  void on_instruct_machine_msg(ConstInstructMachinePtr &msg);
  
  std::string puck_in_processing_name_;
  gazsim_msgs::Color color_to_put_;
  
  void add_base();
  gzwrap::Pose3d add_base_pose();
  u_int32_t number_bases_;
  
  gazebo::transport::PublisherPtr add_base_publisher_;
  void publish_indicator(bool active, int number);
};

}

#endif // RING_STATION_H

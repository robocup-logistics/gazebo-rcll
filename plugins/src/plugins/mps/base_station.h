/***************************************************************************
 *  base_station.h - controls a basesation mps
 *
 *  Generated: Wed Apr 22 13:25:39 2015
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

#ifndef BASESTATION_H
#define BASESTATION_H

#include "mps.h"


#define PROB_BS_SLIDE_BROKEN config->get_float("plugins/mps/base-station/prob-slide-broken")
#define REBREAK_BS_INTERVAL config->get_float("plugins/mps/base-station/rebreak-interval")

namespace gazebo
{

class BaseStation : public Mps
{
public:
  BaseStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
private:
  void on_puck_msg(ConstPosePtr &msg);
  void new_machine_info(ConstMachine &machine);
  void on_instruct_machine_msg(ConstInstructMachinePtr &msg);
  std::string have_puck_;
  
  
  void on_new_puck(ConstNewPuckPtr &msg);

  //this function generates random values to decide about the broken state of the base_station
  void decide_broken_state();
  common::Time last_time_rebreak_;
};

}
#endif // BASESTATION_H

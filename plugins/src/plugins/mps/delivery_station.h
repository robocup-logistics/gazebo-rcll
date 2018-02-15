/***************************************************************************
 *  delivery_station.h - controls a delivery station mps
 *
 *  Generated: Wed Apr 22 14:32:39 2015
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

#ifndef DELIVERY_STATION_H
#define DELIVERY_STATION_H

#include "mps.h"

namespace gazebo
{

class DeliveryStation : public Mps
{
public:
  DeliveryStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
  void on_puck_msg(ConstPosePtr &msg);
  void new_machine_info(ConstMachine &machine);
  void on_instruct_machine_msg(ConstInstructMachinePtr &msg);
  void deliver();

  uint selected_gate_;
  physics::ModelPtr puck_;
};

}

#endif // DELIVERY_STATION_H

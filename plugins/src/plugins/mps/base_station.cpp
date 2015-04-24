/***************************************************************************
 *  base_station.cpp - controls a basesation mps
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

#include "base_station.h"

using namespace gazebo;

BaseStation::BaseStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) :
  Mps(_parent,_sdf)
{
  have_puck_ = "";
  
  factoryPub = node_->Advertise<msgs::Factory>("~/factory");
}

void BaseStation::on_puck_msg(ConstPosePtr &msg)
{
  if(msg->name() == have_puck_ &&
     !puck_in_input(msg) &&
     !puck_in_output(msg))
  {
    have_puck_ = "";
    set_state(State::RETRIEVED);
  }
}

void BaseStation::new_machine_info(ConstMachine &machine)
{
}

/***************************************************************************
 *  mps_opcua__loader.h - loads an mps opcua server for each mps
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

#ifndef MPSOPCUALOADER_H
#define MPSOPCUALOADER_H

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <configurable/configurable.h>
#include <core/exception.h>
#include <mps_comm/mps_server.h>

namespace gazebo {

class MpsOpcUaLoader : public WorldPlugin,
                       public gazebo_rcll::ConfigurableAspect

{
public:
  MpsOpcUaLoader();
  ~MpsOpcUaLoader();

  // Overridden WorldPlugin-Functions
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/);
  virtual void Reset();

private:
  /// Pointer to the gazbeo world
  physics::WorldPtr world_;

  mps_comm::OPCServer *mps_;
};

} // namespace gazebo

#endif // MPSLOADER_H

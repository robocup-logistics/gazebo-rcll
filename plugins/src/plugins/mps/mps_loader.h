/***************************************************************************
 *  mps_loader.h - loads an mps of a specific type for the plugin
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

#ifndef MPSLOADER_H
#define MPSLOADER_H

#include <gazebo/gazebo.hh>
#include "mps.h"

namespace gazebo
{

class MpsLoader : public ModelPlugin
{
public:
  MpsLoader();
  ~MpsLoader();
  
  //Overridden ModelPlugin-Functions
  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  virtual void OnUpdate(const common::UpdateInfo &);
  virtual void Reset();
  
private:
  Mps *mps_;
};

}

#endif // MPSLOADER_H

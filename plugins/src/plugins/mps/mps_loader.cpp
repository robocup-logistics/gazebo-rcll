/***************************************************************************
 *  mps_loader.cpp - loads an mps of a specific type for the plugin
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

#include "mps_loader.h"
#include "base_station.h"
#include "ring_station.h"
#include "cap_station.h"
#include "delivery_station.h"
#include "storage_station.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(MpsLoader)

MpsLoader::MpsLoader()
{

}

MpsLoader::~MpsLoader()
{
  delete mps_;
  printf("Destructing Mps Plugin!\n");
}

void MpsLoader::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  std::string name = _parent->GetName();
  //set the machine type
  if(name.find("BS")!=std::string::npos)
  {
    printf("detected machine type: base \n");
    mps_ = new BaseStation(_parent, sdf);
  }
  if(name.find("SS")!=std::string::npos)
  {
    printf("detected machine type: Storage \n");
    mps_ = new StorageStation(_parent, sdf);
  }
  else if(name.find("CS")!=std::string::npos)
  {
    printf("detected machine type: cap \n");
    mps_= new CapStation(_parent, sdf);
  }
  else if(name.find("RS")!=std::string::npos)
  {
    printf("detected machine type: ring \n");
    mps_ = new RingStation(_parent, sdf);
  }
  else if(name.find("DS")!=std::string::npos)
  {
    printf("detected machine type: delivery \n");
    mps_ = new DeliveryStation(_parent, sdf);
  }
  else
  {
    printf("unknowen machine: %s\n",name.c_str());
  }
}

void MpsLoader::OnUpdate(const common::UpdateInfo &)
{
  
}

void MpsLoader::Reset()
{
  
}



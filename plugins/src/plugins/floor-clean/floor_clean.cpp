/***************************************************************************
 *  floor-clean.cpp - Cleans laying pucks on the ground
 *
 *  Created: Fri Jan 12 2018
 *  Copyright  2018 Morian Sonnet
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <math.h>
#include <cfloat>

#include <utils/misc/gazebo_api_wrappers.h>

#include "floor_clean.h"

#include <random>

using namespace gazebo;


///Constructor
FloorClean::FloorClean()
{
}

///Destructor
FloorClean::~FloorClean()
{
  printf("Destructing FloorClean Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void FloorClean::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) 
{
    if(FLOOR_CLEAN_OFF) return; //this plugin is turned off, so do nothing
  // Store the pointer to the model
  this->world_ = _world;

  //get the model-name
  printf("Loading FloorClean WorldPlugin\n");


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FloorClean::OnUpdate, this, _1));

  last_clean_t_ = world_->GetSimTime() + common::Time(30.0);
  x_to_put_ = 0;
  y_to_put_ = -FIELD_Y_SIZE;
  z_to_put_ = 0.55*PUCK_HEIGHT;
}


/** Called by the world update start event
 */
void FloorClean::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    double delta_t_to_last_clean = (world_->GetSimTime() - last_clean_t_).Double();
    if(delta_t_to_last_clean < CLEAN_INTERVAL)return;   //nothing to do bc waiting time not over


    std::cout << "Cleaning the floor now" << std::endl;
    removeFloorPucks(getFloorPucks()); //remove all pucks laying on the floor in the field

    last_clean_t_ = world_->GetSimTime(); //update clean time (even when no pucks were moved)
    std::cout << "Done with floor cleaning" << std::endl;
    return;
}

/** on Gazebo reset
 */
void FloorClean::Reset()
{
  last_clean_t_ = world_->GetSimTime();
}


void FloorClean::setPuckPose(physics::ModelPtr puck)
{
  puck->SetWorldPose(gzwrap::Pose3d(x_to_put_,y_to_put_,z_to_put_,0,0,0,0));
  x_to_put_ += 4.0*PUCK_SIZE;
}

std::vector<physics::ModelPtr> FloorClean::getFloorPucks() 
{
  std::cout << "Collecting all floor pucks now" << std::endl;
  std::vector<physics::ModelPtr> FloorPucks;
  unsigned int modelCount = world_->GZWRAP_MODEL_COUNT();
  //filter returned list by name. Each puck starts with "puck", e.g. "puck0", "puck1", ... and then find the nearest puck
  for(unsigned int i = 0; i < modelCount; i++){
    physics::ModelPtr model;
    model = world_->GZWRAP_MODEL_BY_INDEX(i);
    if (fnmatch("puck*",model->GetName().c_str(),FNM_CASEFOLD) == 0){
      //std::cout << "Z_POS " << model->GetName() << " : " << model->GZWRAP_WORLD_POSE().GZWRAP_POS_Z << std::endl;
      //std::cout << "Y_POS " << model->GetName() << " : " << model->GZWRAP_WORLD_POSE().GZWRAP_POS_Y << std::endl;
      //std::cout << "X_POS " << model->GetName() << " : " << model->GZWRAP_WORLD_POSE().GZWRAP_POS_X << std::endl;
      if(
              model->GZWRAP_WORLD_POSE().GZWRAP_POS_Y > 0 && model->GZWRAP_WORLD_POSE().GZWRAP_POS_Y < FIELD_Y_SIZE
              && std::abs(model->GZWRAP_WORLD_POSE().GZWRAP_POS_X) < FIELD_X_SIZE
              && (model->GZWRAP_WORLD_POSE().GZWRAP_POS_Z < CLEAN_THRESHOLD || model->GetLink("puck_waste"))) //puck is either laying on the floor or has wasted attribute
      {
          //puck is on floor level and inside the field, so need to be removed
          FloorPucks.push_back(model);
      }
    }
  }
  return FloorPucks;
}

void FloorClean::removeFloorPucks(std::vector<physics::ModelPtr> FloorPucks)
{
    std::cout << "Removing all the floor pucks now" << std::endl;
    for(auto floorPuck: FloorPucks)
    {
        std::cout << "Removing " << floorPuck->GetName() << std::endl;
        setPuckPose(floorPuck);
    }
}


/***************************************************************************
 *  floor-clean.h - Cleans laying pucks on the ground
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <list>
#include <string.h>
#include <fnmatch.h>

#include <boost/thread/mutex.hpp>
#include <configurable/configurable.h>

#include <random>


//config values
#define CLEAN_THRESHOLD config->get_float("plugins/floor-clean/clean-threshold")
#define FIELD_X_SIZE config->get_float("plugins/floor-clean/field-x-size")
#define FIELD_Y_SIZE config->get_float("plugins/floor-clean/field-y-size")
#define CLEAN_INTERVAL config->get_float("plugins/floor-clean/clean-interval")
#define PUCK_HEIGHT config->get_float("plugins/mps/puck_height");
#define PUCK_SIZE config->get_float("plugins/mps/puck_size");

#define FLOOR_CLEAN_OFF config->get_bool("plugins/floor-clean/floor-clean-off")



namespace gazebo
{
  /**
   * Cleans puck on floor
   * @author Morian Sonnet
   */
  class FloorClean : public WorldPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    FloorClean();
    ~FloorClean();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    /// Pointer to the gazbeo model
    physics::WorldPtr world_;
    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_;


    //FloorClean Stuff:
    void removeFloorPucks(std::vector<physics::ModelPtr> FloorPucks);
    std::vector<physics::ModelPtr> getFloorPucks();
    void setPuckPose(physics::ModelPtr puck);
    double x_to_put_;
    double y_to_put_;
    double z_to_put_;
    common::Time last_clean_t_;
  };
  GZ_REGISTER_WORLD_PLUGIN(FloorClean)
}

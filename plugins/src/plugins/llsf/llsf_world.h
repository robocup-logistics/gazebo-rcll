/***************************************************************************
 *  llsf_world.h - The main plugin for the llsf field
 *
 *  Created: Sun Aug 18 14:55:33 2013
 *  Copyright  2013  Frederik Zwilling
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


#include <gazebo/gazebo.hh>

#include "data_table.h"
#include "light_control.h"
#include "puck_localization.h"
#include "rfid_sensors.h"
#include "time_sync.h"
#include "field_referee.h"
#include "simulation_control.h"

namespace gazebo
{
  /**
   * Main plugin for the LLSF field
   */
  class LlsfWorldPlugin : public WorldPlugin
  {
  public:
    ///Constructor
    LlsfWorldPlugin();
    ///Destructor
    ~LlsfWorldPlugin();

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);


  private:
    ///update function
    void Update();
    event::ConnectionPtr update_connection_;

    ///Node for communication
    transport::NodePtr node_;
    
    physics::WorldPtr world_;

    ///Table with simulation data
    LlsfDataTable *table_;

    ///Controller of machine light signals
    LightControl *light_control_;

    PuckLocalization *puck_localization_;

    ///the field referee removes finished pucks
    FieldReferee *field_referee_;
    
    ///checks if there is a puck under the rfid
    RfidSensors *rfid_sensors_;

    ///Sync the time with fawkes and the refbox
    TimeSync *time_sync_;

    ///Stop gazebo on request
    SimulationControl *simulation_control_;

    double puck_update_frequency_;
    double time_sync_frequency_;
    double last_puck_update_;
    double last_time_sync_;
  };
  GZ_REGISTER_WORLD_PLUGIN(LlsfWorldPlugin)
}

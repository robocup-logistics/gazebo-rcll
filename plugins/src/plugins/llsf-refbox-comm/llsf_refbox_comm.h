/***************************************************************************
 *  llsf_refbox_comm.h - World plugin for the refbox connection in the llsf
 *
 *  Created: Fri Mar 06 16:09:42 2015
 *  Copyright  2015  Frederik Zwilling
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


namespace gazebo
{
  /**
   * World plugin for the refbox connection in the llsf
   * Basically it gets the peer msgs from the refbox and publishes these msgs in gazebo
   */
  class LlsfRefboxCommPlugin : public WorldPlugin
  {
  public:
    ///Constructor
    LlsfRefboxCommPlugin();
    ///Destructor
    ~LlsfRefboxCommPlugin();

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);


  private:
    ///update function
    void Update();
    event::ConnectionPtr update_connection_;

    ///Node for communication
    transport::NodePtr node_;
    
    physics::WorldPtr world_;
  };
  GZ_REGISTER_WORLD_PLUGIN(LlsfRefboxCommPlugin)
}

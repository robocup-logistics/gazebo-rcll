/***************************************************************************
 *  mps_placement.h - Plugin to place the MPSs as specified by the refbox
 *
 *  Created: Thu Apr 16 12:16:41 2015
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <list>
#include <string.h>
#include <gazsim_msgs/WorkpieceCommand.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/GameState.pb.h>

//typedefs for sending the messages over the gazebo node
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const boost::shared_ptr<llsf_msgs::GameState const> ConstGameStatePtr;

//config values
#define TOPIC_MACHINE_INFO "~/LLSFRbSim/MachineInfo/"
#define TOPIC_GAME_STATE "~/LLSFRbSim/GameState/"
#define WAIT_TIME_BEFORE_PLACEMENT 15
#define ZONE_HEIGHT 1.5
#define ZONE_WIDTH 2.0


namespace gazebo
{
  /**
   * Plugin to place the MPSs as specified by the refbox
   * @author Frederik Zwilling
   */
  class MpsPlacementPlugin : public WorldPlugin
  {
  public:
    MpsPlacementPlugin();
   ~MpsPlacementPlugin();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    virtual void Reset();

  private:
    /// Pointer to the gazbeo world
    physics::WorldPtr world_;
    ///Node for communication
    transport::NodePtr node_;

    // MpsPlacementPlugin Stuff:
    
    /// Subscriber to get Machine Info from refbox
    transport::SubscriberPtr machine_info_sub_;
    
    /// Subscriber to get Game state msgs
    transport::SubscriberPtr game_state_sub_;

    /// Spawn machine at a position
    void spawn_mps(const math::Pose &spawn_pose, std::string model_name);

    ///Remove existing MPS (e.g. before spawning them at other location)
    void remove_existing_mps();

    /// Handler for getting refbox msgs
    void on_machine_info_msg(ConstMachineInfoPtr &msg);
    void on_game_state_msg(ConstGameStatePtr &msg);
    
    bool machines_placed_;
    bool is_game_started_;
    int random_seed_base_;

    // Create a publisher on the ~/factory topic to spawn models
    transport::PublisherPtr factoryPub;
    transport::PublisherPtr modelPub;
  };
  GZ_REGISTER_WORLD_PLUGIN(MpsPlacementPlugin)
}

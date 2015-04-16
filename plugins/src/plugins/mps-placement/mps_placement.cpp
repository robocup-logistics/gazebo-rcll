/***************************************************************************
 *  mps_placement.cpp - Plugin to place the MPSs as specified by the refbox
 *
 *  Created: Thu Apr 16 12:44:00 2015
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

#include <math.h>

#include "mps_placement.h"

using namespace gazebo;

///Constructor
MpsPlacementPlugin::MpsPlacementPlugin()
{
}
///Destructor
MpsPlacementPlugin::~MpsPlacementPlugin()
{
  printf("Destructing MpsPlacementPlugin Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void MpsPlacementPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  world_ = _world;

  printf("Loading MpsPlacementPlugin WorldPlugin\n");

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->node_->Init(world_->GetName());

  //subscribe for refbox msgs
  machine_info_sub_ = node_->Subscribe(std::string(TOPIC_MACHINE_INFO), &MpsPlacementPlugin::on_machine_info_msg, this);
  game_state_sub_ = node_->Subscribe(std::string(TOPIC_GAME_STATE), &MpsPlacementPlugin::on_game_state_msg, this);
  
  machines_placed_ = false;
  is_game_started_ = false;
}

/** on Gazebo reset
 */
void MpsPlacementPlugin::Reset()
{
  machines_placed_ = false;
}

/** Functions for recieving a machine info msg
 * @param msg message
 */ 
void MpsPlacementPlugin::on_machine_info_msg(ConstMachineInfoPtr &msg)
{  
  if(machines_placed_ || !is_game_started_){
    return;
  }

  // don't set positions before simulation is initialized
  if(world_->GetSimTime().Double() < WAIT_TIME_BEFORE_PLACEMENT){
    return;
  }

  printf("MpsPlacementPlugin: Placing MPSs according to Machine Info from Refbox\n");
  //sim time for random seed
  int time = (int) world_->GetSimTime().Double();
  
  // go through all machines
  for(int i = 0; i < msg->machines_size(); i++){
    llsf_msgs::Machine machine_msg = msg->machines(i);
    std::string mps_name = machine_msg.name();

    //find machine in world
    physics::ModelPtr mps = world_->GetModel(mps_name);
    
    if(!mps){
      printf("MpsPlacementPlugin: Could not find mps %s\n", mps_name.c_str());
    }
    else{
      int zone = (int) machine_msg.zone();
      
      //determine position from Zone
      int zone_cyan = zone;
      if (zone_cyan > 12) {
	zone_cyan -= 12;
      }
      int zone_x = (zone_cyan-1) / 4; //integer devision
      int zone_y = (zone_cyan-1) % 4;
      float zone_mid_x = zone_x * ZONE_WIDTH + 0.5 * ZONE_WIDTH;
      float zone_mid_y = zone_y * ZONE_HEIGHT + 0.5 * ZONE_HEIGHT;

      //move mps away from wall
      if(zone_x == 2){
	zone_mid_x -= 0.25 * ZONE_WIDTH;
      }
      if(zone_y == 0){
	zone_mid_y += 0.25 * ZONE_HEIGHT;
      }
      else if(zone_y == 3){
	zone_mid_y -= 0.25 * ZONE_HEIGHT;
      }

      //randomize orientation
      srand(time * zone_cyan);
      float ori = rand() % 100 -50;
      ori *= 2.0 * M_PI / 50.0;
      
      
      if(zone > 12){
	//Mirrow mps on the Magenta half
        zone_mid_x = -zone_mid_x;
	ori -= M_PI / 2.0;
	ori = -ori;
	ori += M_PI / 2.0;
      }
      
      printf("MpsPlacementPlugin: Seting %s into zone %d, (%f,%f, %f)\n", mps_name.c_str(), zone, zone_mid_x, zone_mid_y, ori);
      mps->SetStatic(false);
      mps->SetWorldPose(math::Pose(zone_mid_x, zone_mid_y, 0, 0, 0, ori));
      mps->SetStatic(true);
    }
  }
  
  machines_placed_ = true;
}

/** Functions for recieving a game state msg
 * We want to knwo if the game is started because the refbox assigns
 *  the zones to the machines not in the PRE_GAME phase
 * @param msg message
 */ 
void MpsPlacementPlugin::on_game_state_msg(ConstGameStatePtr &msg)
{  
  if(!is_game_started_ && msg->phase() != llsf_msgs::GameState::PRE_GAME){
    printf("MpsPlacementPlugin: Game started\n");
    is_game_started_ = true;
  }
  if(is_game_started_ && msg->phase() == llsf_msgs::GameState::PRE_GAME){
    printf("MpsPlacementPlugin: Game stopped\n");
    is_game_started_ = false;
  }
}

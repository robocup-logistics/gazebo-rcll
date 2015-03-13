/***************************************************************************
 *  mps.h - Plugin to control a simulated MPS
 *
 *  Created: Fri Feb 20 17:15:34 2015
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

//amount of pucks to listen for
#define NUMBER_PUCKS 20
//how far is the center of the belt hsifted from the machine center
#define BELT_OFFSET_SIDE 0.025
//radius of the area where a workpiece is detected by the machine
#define DETECT_TOLERANCE 0.03
//radius of a workpiece
#define PUCK_SIZE 0.02
//length of the belt to calculate pos of input/output area
#define BELT_LENGTH 0.35
//Height of the belt
#define BELT_HEIGHT 0.92
//Height of the center of the tag
#define TAG_HEIGHT 0.54
//At what simulation time to spawn the tag (too early and the tag spawns at (0, 0, 0))
#define TAG_SPAWN_TIME 5.0


namespace gazebo
{
  typedef enum MachineType{
    Base,
    Cap,
    Ring,
    Delivery,
    Unknown,
  } MachineType;
  /**
   * Plugin to control a simulated MPS
   * @author Frederik Zwilling
   */
  class Mps : public ModelPlugin
  {
  public:
    Mps();
   ~Mps();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    /// Pointer to the gazbeo model
    physics::ModelPtr model_;
    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_;
    ///Node for communication
    transport::NodePtr node_;
    ///name of the mps and the communication channel
    std::string name_;

    // Mps Stuff:
    
    /// Subscriber to get puck positions
    transport::SubscriberPtr puck_subs_[NUMBER_PUCKS];

    /// Handler for puck positions
    void on_puck_msg(ConstPosePtr &msg);
    
    ///Publisher to send spawn machine tags
    transport::PublisherPtr visPub_;
    void spawnTag(std::string visual_name, std::string tag_name, float x, float y, float ori);
    bool spawned_tags_;

    ///centers of input and output areas (global)
    float input_x_, input_y_, output_x_, output_y_;
    // the type of this mps
    MachineType machine_type_;
  };
}

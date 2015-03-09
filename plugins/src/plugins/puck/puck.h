/***************************************************************************
 *  puck.h - Plugin to control a simulated workpiece
 *
 *  Created: Fri Feb 20 17:15:34 2015
 *  Copyright  2015  Randolph Maaßen
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
#include <stack>
#include <string.h>
#include <gazsim_msgs/WorkpieceCommand.pb.h>

typedef const boost::shared_ptr<gazsim_msgs::WorkpieceCommand const> ConstWorkpieceCommandPtr;
/// The height of one ring
#define RING_HEIGHT 0.008 //meter
/// The height of one cap
#define CAP_HEIGHT 0.004 //meter
/// The height of the workpiece base
#define WORKPIECE_HEIGHT 0.0225 //meter

namespace gazebo
{
  /**
   * Plugin to control a simulated Puck
   * @author Randolph Maaßen
   */
  class Puck : public ModelPlugin
  {
  public:
    Puck();
   ~Puck();

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
    ///name of the puck and the communication channel
    std::string name_;

    // Puck Stuff:
    
    /// Subscriber to get commands for model ring addition
    transport::SubscriberPtr command_subscriber;

    /// Handler for command messages
    void on_command_msg(ConstWorkpieceCommandPtr &cmd);
    /// Add one ring on command
    void add_ring(gazsim_msgs::Color clr);
    /// Add a cap on command
    void add_cap(gazsim_msgs::Color clr);

    /// The number of stored rings
    size_t ring_count_;

    /// Check, if we have a cap on top
    bool have_cap;

    /// Publisher to send visual changes to gazebo
    transport::PublisherPtr visual_pub_;
  };
}

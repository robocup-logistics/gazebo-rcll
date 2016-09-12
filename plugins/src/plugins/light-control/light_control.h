/***************************************************************************
 *  light_control.h - Plugin to control the light signals on an MPS
 *
 *  Created: Sat Feb 21 19:11:25 2015
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
#include <llsf_msgs/MachineInfo.pb.h>
#include <configurable/configurable.h>


//typedefs for sending the messages over the gazebo node
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;

//config values
#define TOPIC_MACHINE_INFO config->get_string("plugins/light-control/topic-machine-info").c_str()


namespace gazebo
{
  typedef enum Color
  {
    RED,
    YELLOW,
    GREEN
  } Color;

  typedef enum LightState
  {
    OFF,
    ON,
    BLINK
  } LightState;


  namespace msgs
  {
    class Visual;
  }

  /**
   * Plugin to control the light signals on an MPS
   * @author Frederik Zwilling
   */
  class LightControl : public ModelPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    LightControl();
   ~LightControl();

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
    ///name of the light signal models
    std::string name_;
    ///pointer to the world
    physics::WorldPtr world_;

    // Light_Control Stuff:

    LightState state_red_, state_yellow_, state_green_;
    LightState prev_state_red_, prev_state_yellow_, prev_state_green_;

    /// Subscriber to get msgs about the light status
    transport::SubscriberPtr light_msg_sub_;

    /// Handler for light status msg
    void on_light_msg(ConstMachineInfoPtr &msg);

    ///Publisher to send visual changes to gazebo
    transport::PublisherPtr visPub_;
    void change_light(std::string machine_name, Color color, LightState &state, LightState &prev_state);

    ///time variable to send in intervals
    double last_sent_time_;

    ///name of the machine containing the light signal
    std::string machine_name_;
  };
}

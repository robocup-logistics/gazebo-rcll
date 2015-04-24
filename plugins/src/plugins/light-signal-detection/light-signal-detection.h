/***************************************************************************
 *  light-signal-detection.h - provides ground truth light signal detection of the nearest achine in front of the robotino
 *
 *  Created: Mon Mar 30 16:15:38 2015
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


//typedefs for sending the messages over the gazebo node
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;

//config values
#define TOPIC_MACHINE_INFO "~/LLSFRbSim/MachineInfo/"
#define RADIUS_DETECTION_AREA 0.4
//Search area where the robot is looking for the signal relative to the robots center
#define SEARCH_AREA_REL_X 0.6
#define SEARCH_AREA_REL_Y 0.4
#define SEND_INTERVAL 0.5
#define VISIBILITY_HISTORY_INCREASE_PER_SECOND 30 //usually camera frame rate


namespace gazebo
{
  /**
   * Provides ground Truth position
   * @author Frederik Zwilling
   */
  class LightSignalDetection : public ModelPlugin
  {
  public:
    LightSignalDetection();
   ~LightSignalDetection();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    /// Pointer to the gazbeo model
    physics::ModelPtr model_;
    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_;
    ///Node for communication to fawkes
    transport::NodePtr node_;
    ///Node for communication in gazebo
    transport::NodePtr world_node_;
    ///name of the communication channel and the sensor
    std::string name_;

    ///time variable to send in intervals
    double last_sent_time_;

    //Light-detection Stuff:
    ///Functions for sending information to fawkes:
    void send_light_detection();

    //remember light state in front of the robot
    llsf_msgs::LightState state_red_, state_yellow_, state_green_;
    /// Subscriber to get msgs about the light status
    transport::SubscriberPtr light_msg_sub_;
    /// Handler for light status msg
    void on_light_msg(ConstMachineInfoPtr &msg);
    void save_light_signal(llsf_msgs::Machine machine);

    //is the light currently detected?
    bool visible_;
    //how long was the light detected?
    int visibility_history_;
    double visible_since_;

    //robot position
    math::Pose robot_pose_;

    ///Publisher for Detected light signal
    transport::PublisherPtr light_signal_pub_;
  };
}

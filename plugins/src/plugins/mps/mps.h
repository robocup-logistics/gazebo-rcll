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

#ifndef MPS_H
#define MPS_H

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
#include <llsf_msgs/MachineCommands.pb.h>
#include <gazsim_msgs/NewPuck.pb.h>
#include <map>
#include <configurable/configurable.h>

//amount of pucks to listen for
#define NUMBER_PUCKS config->get_int("plugins/mps/number_pucks")
//how far is the center of the belt hsifted from the machine center
#define BELT_OFFSET_SIDE config->get_float("plugins/mps/belt_offset_side")
//radius of the area where a workpiece is detected by the machine
#define DETECT_TOLERANCE config->get_float("plugins/mps/detect_tolerance")
//radius of a workpiece
#define PUCK_SIZE config->get_float("plugins/mps/puck_size")
//height of a puck
#define PUCK_HEIGHT config->get_float("plugins/mps/puck_height")
//length of the belt to calculate pos of input/output area
#define BELT_LENGTH config->get_float("plugins/mps/belt_length")
//Height of the belt
#define BELT_HEIGHT config->get_float("plugins/mps/belt_height")
//Height of the center of the tag
#define TAG_HEIGHT config->get_float("plugins/mps/tag_height")
//Height of the center of the tag
#define TAG_SIZE config->get_float("plugins/mps/tag_size")
//At what simulation time to spawn the tag (too early and the tag spawns at (0, 0, 0))
#define TAG_SPAWN_TIME config->get_float("plugins/mps/tag_spawn_time")
#define TOPIC_SET_MACHINE_STATE config->get_string("plugins/mps/topic_set_machine_state").c_str()
#define TOPIC_MACHINE_INFO config->get_string("plugins/mps/topic_machine_info").c_str()
#define TOPIC_PUCK_COMMAND config->get_string("plugins/mps/topic_puck_command").c_str()
#define TOPIC_PUCK_COMMAND_RESULT config->get_string("plugins/mps/topic_puck_command_result").c_str()
#define TOPIC_JOINT config->get_string("plugins/mps/topic_joint").c_str()


typedef const boost::shared_ptr<llsf_msgs::SetMachineState const> ConstSetMachineStatePtr;
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const llsf_msgs::Machine ConstMachine;
typedef llsf_msgs::MachineState State;
typedef const boost::shared_ptr<gazsim_msgs::NewPuck const> ConstNewPuckPtr;

namespace gazebo
{
  /**
   * Plugin to control a simulated MPS
   * @author Frederik Zwilling
   */
  class Mps: public gazebo_rcll::ConfigurableAspect
  {
  public:
    Mps(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual ~Mps();

    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  protected:
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
    std::vector<transport::SubscriberPtr> puck_subs_;
    /// Subscriber to get machine infos
    transport::SubscriberPtr machine_info_subscriber_;

    /// Handler for puck positions
    virtual void on_puck_msg(ConstPosePtr &msg);
    /// Handler for machine msgs
    void on_machine_msg(ConstMachineInfoPtr &msg);
    virtual void new_machine_info(ConstMachine &machine);
    
    transport::SubscriberPtr new_puck_subscriber_;
    virtual void on_new_puck(ConstNewPuckPtr &msg);
    
    ///Publisher to send machine state
    transport::PublisherPtr set_machne_state_pub_;
    
    ///Publisher to send spawn machine tags
    transport::PublisherPtr visPub_;
    void grabTag(std::string link_name, std::string tag_name, gazebo::physics::JointPtr joint);
    double spawned_tags_last_;
    double created_time_;

    ///centers of input and output areas (global)
    virtual float input_x();
    virtual float input_y();
    virtual float output_x();
    virtual float output_y();
    
    virtual math::Pose input();
    virtual math::Pose output();
    
    /// convert puck pose from mps frame to world frame
    math::Pose get_puck_world_pose(double long_side, double short_side, double height = -1.0);
    
    std::string current_state_;
    
    void set_state(State state);
    
    bool pose_hit(const math::Pose &to_test, const math::Pose &reference, double tolerance = -1.0);
    
    bool puck_in_input(ConstPosePtr &pose);
    bool puck_in_output(ConstPosePtr &pose);
    bool puck_in_input(const math::Pose &pose);
    bool puck_in_output(const math::Pose &pose);
    
    physics::WorldPtr world_;
    
    void spawn_puck(const math::Pose &spawn_pose, enum gazsim_msgs::Color base_color);
    
    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub;
    
    /// Publisher for puck command
    transport::PublisherPtr puck_cmd_pub_;
    
    transport::SubscriberPtr joint_message_sub_;
    void on_joint_msg(ConstJointPtr &joint_msg);
    
    std::map<u_int32_t,std::string> hold_pucks;
    bool is_puck_hold(std::string puck_name);

    //stuff for grabing the tag to the right position
    static gazebo::physics::LinkPtr getLinkEndingWith(physics::ModelPtr model, std::string link);
    static gazebo::physics::JointPtr getJointEndingWith(physics::ModelPtr model, std::string link);
    gazebo::physics::JointPtr tag_joint_input;
    gazebo::physics::JointPtr tag_joint_output;
    bool grabbed_tags_ = false;
  };
}

#endif // MPS_H

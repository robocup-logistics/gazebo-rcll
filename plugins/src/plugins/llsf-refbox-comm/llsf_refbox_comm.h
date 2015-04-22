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
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>

#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/MachineCommands.pb.h>
#include <llsf_msgs/SimTimeSync.pb.h>
#include <llsf_msgs/PuckInfo.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/GameInfo.pb.h>
#include <gazsim_msgs/SimTime.pb.h>

//typedefs for sending the messages over the gazebo node
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const boost::shared_ptr<llsf_msgs::PlacePuckUnderMachine const> ConstPlacePuckUnderMachinePtr;
typedef const boost::shared_ptr<llsf_msgs::RemovePuckFromMachine const> ConstRemovePuckFromMachinePtr;
typedef const boost::shared_ptr<gazsim_msgs::SimTime const> ConstSimTimePtr;
typedef const boost::shared_ptr<llsf_msgs::SetGameState const> ConstSetGameStatePtr;
typedef const boost::shared_ptr<llsf_msgs::SetGamePhase const> ConstSetGamePhasePtr;
typedef const boost::shared_ptr<llsf_msgs::SetTeamName const> ConstSetTeamNamePtr;
typedef const boost::shared_ptr<llsf_msgs::SetMachineState const> ConstSetMachineStatePtr;


//config values
#define PROTO_DIR "/plugins/src/libs/llsf_msgs"
#define REFBOX_HOST "127.0.0.1"
#define REFBOX_PORT 4444
#define RECONNECT_INTERVAL 10 //in s
//Max number of reconnect attempts (due to crash when tried to connect often)
#define RECONNECT_ATTEMPTS 50
#define TOPIC_MACHINE_INFO "~/LLSFRbSim/MachineInfo/"
#define TOPIC_GAME_STATE "~/LLSFRbSim/GameState/"
#define TOPIC_TIME "~/gazsim/time-sync/"
#define TOPIC_SET_GAME_STATE "~/LLSFRbSim/SetGameState/"
#define TOPIC_SET_GAME_PHASE "~/LLSFRbSim/SetGamePhase/"
#define TOPIC_SET_TEAM_NAME "~/LLSFRbSim/SetTeamName/"
#define TOPIC_SET_MACHINE_STATE "~/LLSFRbSim/SetMachineState/"

namespace protobuf_comm {
  class ProtobufStreamClient;
}

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

    ///on refbox connection established
    void client_connected();
    ///on refbox connection lost
    void client_disconnected(const boost::system::error_code &error);
    ///on message from the refbox
    void client_msg(uint16_t comp_id, uint16_t msg_type,
		    std::shared_ptr<google::protobuf::Message> msg);

  private:
    ///update function
    void Update();
    event::ConnectionPtr update_connection_;

    ///Node for communication
    transport::NodePtr node_;
    physics::WorldPtr world_;

    protobuf_comm::ProtobufStreamClient *client_;
    std::vector<std::string> proto_dirs_;
    protobuf_comm::MessageRegister      *message_register_;

    //Publisher and subscriber for the connection to gazebo
    gazebo::transport::PublisherPtr machine_info_pub_;
    gazebo::transport::PublisherPtr game_state_pub_;
    /* gazebo::transport::SubscriberPtr place_puck_under_machine_sub_; */
    /* gazebo::transport::SubscriberPtr remove_puck_under_machine_sub_; */
    gazebo::transport::SubscriberPtr time_sync_sub_;
    gazebo::transport::SubscriberPtr set_game_state_sub_;
    gazebo::transport::SubscriberPtr set_game_phase_sub_;
    gazebo::transport::SubscriberPtr set_team_name_sub_;
    gazebo::transport::SubscriberPtr set_machine_state_sub_;

    //handler methods
    /* void on_puck_place_msg(ConstPlacePuckUnderMachinePtr &msg); */
    /* void on_puck_remove_msg(ConstRemovePuckFromMachinePtr &msg); */
    void on_time_sync_msg(ConstSimTimePtr &msg);
    void on_set_game_state_msg(ConstSetGameStatePtr &msg);
    void on_set_game_phase_msg(ConstSetGamePhasePtr &msg);
    void on_set_team_name_msg(ConstSetTeamNamePtr &msg);
    void on_set_machine_state_msg(ConstSetMachineStatePtr &msg);

    //helper variables
    bool connected_;
    double last_connect_try_;
    int connect_tries_;

    void create_client();
    
  };
  GZ_REGISTER_WORLD_PLUGIN(LlsfRefboxCommPlugin)
}

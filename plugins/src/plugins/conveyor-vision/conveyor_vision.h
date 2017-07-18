/***************************************************************************
 *  conveyor_vision.h - Plugin for a conveyor vision sensor on a model
 *
 *  Created: Sun Jul 12 16:02:29 2015
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
#include <list>
#include <string.h>
#include <llsf_msgs/ConveyorVisionResult.pb.h>
#include <llsf_msgs/Pose3D.pb.h>
#include <configurable/configurable.h>

#define RADIUS_DETECTION_AREA config->get_float("plugins/conveyor-vision/radius-detection-area")
//Search area where the robot is looking for the conveyor relative to the robots center
#define SEARCH_AREA_REL_X config->get_float("plugins/conveyor-vision/search-area-rel-x")
#define SEARCH_AREA_REL_Y config->get_float("plugins/conveyor-vision/search-area-rel-y")
//amount of pucks to listen for
#define NUMBER_PUCKS number_pucks_
//how far is the center of the belt hsifted from the machine center
#define BELT_OFFSET_SIDE belt_offset_side_
//radius of the area where a workpiece is detected by the machine
#define DETECT_TOLERANCE detect_tolerance_
//radius of a workpiece
#define PUCK_SIZE puck_size_
//height of a puck
#define PUCK_HEIGHT puck_height_
//length of the belt to calculate pos of input/output area
#define BELT_LENGTH belt_length_
//height of the belt
#define BELT_HEIGHT belt_height_
//height of the center of the tag
#define TAG_HEIGHT tag_height_
//height of the center of the tag
#define TAG_SIZE tag_size_


namespace gazebo
{   
  /** @class Gyro
   * Plugin for a conveyor vision sensor on a model
   * @author Randolph Maaßen
   */
  class ConveyorVision : public ModelPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    ///Constructor
    ConveyorVision();

    ///Destructor
    ~ConveyorVision();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    /// Pointer to the model
    physics::ModelPtr model_;
    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_;
    ///Node for communication to fawkes
    transport::NodePtr node_;
    ///name of the gyro and the communication channel
    std::string name_;
    ///Subscriber for SetConveyor
    transport::SubscriberPtr set_conveyor_sub_;
    void on_set_conveyor_msg(ConstIntPtr &msg);

    //config values:
    int number_pucks_;
    //how far is the center of the belt hsifted from the machine center
    float belt_offset_side_;
    //radius of the area where a workpiece is detected by the machine
    float detect_tolerance_;
    //radius of a workpiece
    float puck_size_;
    //height of a puck
    float puck_height_;
    //length of the belt to calculate pos of input/output area
    float belt_length_;
    //Height of the belt
    float belt_height_;
    //Height of the center of the tag
    float tag_height_;
    //Height of the center of the tag
    float tag_size_;
    //Offset from height
    float offset_z_;

    
    ///time variable to send in intervals
    double last_sent_time_;

    ///time interval between to gyro msgs
    double send_interval_;


    //Gyro Stuff:
    ///Sending conveyor results to fawkes:
    void send_conveyor_result();

    ///Publisher for conveyr results
    transport::PublisherPtr conveyor_pub_;  
  };
}

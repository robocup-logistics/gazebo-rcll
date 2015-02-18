/***************************************************************************
 *  gripper_laser_sensor.h - simulates a laser sensor at the gripper
 *                         to detect the machines
 *
 *  Created: Fri Aug 30 17:05:55 2013
 *  Copyright  2013  Frederik Zwilling
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
#include "simDevice.h"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{
  /** Is this the sensor on the left or right side?
   */
  typedef enum Side
  {
    LEFT, RIGHT
  } Side;

  /** 
   *  simulates a laser sensor at the gripper
   *  to detect the machines
   * @author Frederik Zwilling
   */  
  class GripperLaserSensor : public SimDevice
  {
    public: 
    
    //Constructor
    GripperLaserSensor(physics::ModelPtr, transport::NodePtr, sensors::SensorPtr sensorPtr, Side side);

    ///Destructor
    ~GripperLaserSensor();


    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();


    ///what happens if the sensor has new laser data
    void on_new_laser_scans();

    private:
    
    ///connection of the sensor
    event::ConnectionPtr new_laser_scans_connection_;

    ///Pointer to the hokuyo sensor
    sensors::RaySensorPtr parent_sensor_;

    ///Publisher for communication to fawkes
    transport::PublisherPtr gripper_laser_sensor_pub_;

    ///is this the sensor on the left or right?
    Side side_;
  };
}

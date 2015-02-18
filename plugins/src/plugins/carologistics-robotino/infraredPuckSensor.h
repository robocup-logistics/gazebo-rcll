/***************************************************************************
 *  infraredPuckSensor.h - simulates the infrared sensor to detect a puck in
 *                         front of the robotino
 *
 *  Created: Thu Aug 29 21:26:03 2013
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
  /**
   *  This class simulates the infrared sensor to detect a puck 
   *  in front of the robotino by using a ray-sensor
   */
  class InfraredPuckSensor : public SimDevice
  {
    public: 
    
    //Constructor
    InfraredPuckSensor(physics::ModelPtr, transport::NodePtr, sensors::SensorPtr sensorPtr);

    ///Destructor
    ~InfraredPuckSensor();


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
    transport::PublisherPtr infrared_puck_sensor_pub_;

  };
}

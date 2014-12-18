/***************************************************************************
 *  laserSensor.cpp - Provides laser sensor data
 *
 *  Created: Thu Aug 29 21:30:52 2013
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
#include <math.h>
#include <string.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "infraredPuckSensor.h"
#include "../../msgs/Float.pb.h"

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 * @param sensorPtr Pointer to the laser sensor of the model
 */
InfraredPuckSensor::InfraredPuckSensor(physics::ModelPtr model, transport::NodePtr node, sensors::SensorPtr sensorPtr)
 : SimDevice(model, node)
{
  this->parent_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorPtr);
}
InfraredPuckSensor::~InfraredPuckSensor()
{
}

void InfraredPuckSensor::init()
{
  printf("Initialize InfraredPuckSensor \n");

  //Register OnNewLaserScans function
  this->new_laser_scans_connection_ = this->parent_sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&InfraredPuckSensor::on_new_laser_scans, this));
}

void InfraredPuckSensor::create_publishers()
{
  this->infrared_puck_sensor_pub_ = this->node->Advertise<gazsim_msgs::Float>("~/RobotinoSim/InfraredPuckSensor/");
}

void InfraredPuckSensor::create_subscribers()
{
}

void InfraredPuckSensor::update()
{
  //sending the laser scans happens in OnNewLaserScans()
}

void InfraredPuckSensor::on_new_laser_scans()
{
  //if(infrared_puck_sensor_pub_->HasConnections())
  {
    //Get relevant data
    //int numRays = parent_sensor_->GetRangeCount();
    //float angleMin = parent_sensor_->GetAngleMin().Radian();
    //float angleMax = parent_sensor_->GetAngleMax().Radian();
    //float angleStep = parent_sensor_->GetAngleResolution();
    //float rangeMin = parent_sensor_->GetRangeMin();
    //float rangeMax = parent_sensor_->GetRangeMax();
    

    //create Protobuf message
    gazsim_msgs::Float laserMsg;
    //laser has only one beam
    laserMsg.set_value(parent_sensor_->GetRange(0));
    
    //send message
    infrared_puck_sensor_pub_->Publish(laserMsg);
    }
}

/***************************************************************************
 *  puck.cpp - Plugin to control a simulated Workpiece
 *
 *  Created: Fri Feb 20 17:15:54 2015
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

#include <math.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazsim_msgs/NewPuck.pb.h>

#include "puck.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Puck)

///Constructor
Puck::Puck()
{
}
///Destructor
Puck::~Puck()
{
  printf("Destructing Puck Plugin for %s!\n",this->name().c_str());
}

inline std::string Puck::name()
{
  return model_->GetName();
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Puck::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  printf("Loading Puck Plugin of model %s\n", _parent->GetName().c_str());
  // Store the pointer to the model
  this->model_ = _parent;  

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Puck::OnUpdate, this, _1));

  // Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  // the namespace is set to the world name!
  this->node_->Init(model_->GetWorld()->GetName());

  // register visual publisher
  this->visual_pub_ = this->node_->Advertise<msgs::Visual>("~/visual");
  
  // initialize without rings or cap
  this->ring_count_ = 0;
  this->have_cap = false;
  
  this->new_puck_publisher = this->node_->Advertise<gazsim_msgs::NewPuck>("~/new_puck");
  gazsim_msgs::NewPuck new_puck_msg;
  
  new_puck_msg.set_puck_name(name());
  new_puck_msg.set_gps_topic("~/"+name()+"/gazsim/gps/");
  new_puck_publisher->Publish(new_puck_msg);
  
  
  
  // subscribe for puck commands
  this->command_subscriber = this->node_->Subscribe(std::string("~/pucks/cmd"), &Puck::on_command_msg, this);
  
  // publisher for workpiece command results
  this->workpiece_result_pub_ = node_->Advertise<gazsim_msgs::WorkpieceResult>("~/pucks/cmd/result");
  
  base_color_ = gazsim_msgs::Color::RED;
}

/** Called by the world update start event
 */
void Puck::OnUpdate(const common::UpdateInfo & /*_info*/)
{
}

/** on Gazebo reset
 */
void Puck::Reset()
{
}



/** Functions for recieving puck locations Messages
 * @param ring the new ring to put on top
 */ 
void Puck::on_command_msg(ConstWorkpieceCommandPtr &cmd)
{
  if(cmd->puck_name() != name())
  {
    return;
  }
  printf("puck %s recieved command: ",this->name().c_str());
  std::string ring_string = "";
  switch(cmd->command())
  {
    case gazsim_msgs::Command::ADD_RING:
      printf("add ring with color: %s\n",gazsim_msgs::Color_Name(cmd->color()).c_str());
      this->add_ring(cmd->color());
      break;
    case gazsim_msgs::Command::ADD_CAP:
      printf("add cap with color: %s\n",gazsim_msgs::Color_Name(cmd->color()).c_str());
      this->add_cap(cmd->color());
      break;
    case gazsim_msgs::Command::REMOVE_CAP:
      printf("remove cap, providing cap color %s\n", gazsim_msgs::Color_Name(this->cap_color_).c_str());
      remove_cap();
      break;
    case gazsim_msgs::Command::DELIVER:
      for(size_t i = 0; i < ring_count_; i++)
      {
        ring_string += gazsim_msgs::Color_Name(ring_colors_[i]) + ", ";
      }
      printf("delivering a %s base with %zu rings, colored %s and a %s cap",
	     gazsim_msgs::Color_Name(base_color_).c_str(),
             ring_count_,
             ring_string.c_str(),
             gazsim_msgs::Color_Name(cap_color_).c_str());
	     
      break;
    default:
      printf("unknowen");
      break;
  }
}

void Puck::add_ring(gazsim_msgs::Color clr)
{
  
  // create the ring name and add a new ring
  std::string ring_name = std::string("ring_") + std::to_string(this->ring_count_);
  
  msgs::Visual visual_msg = create_visual_msg(ring_name, RING_HEIGHT, clr);

  ring_colors_.push_back(clr);
  
  // publish visual change
  this->visual_pub_->Publish(visual_msg);
  this->ring_count_++;

}

void Puck::add_cap(gazsim_msgs::Color clr)
{
  msgs::Visual vis_msg = create_visual_msg("cap", CAP_HEIGHT, clr);
  
  this->visual_pub_->Publish(vis_msg);
  this->have_cap = true;
  cap_color_ = clr;
}

void Puck::remove_cap()
{
  msgs::Visual vis_msg = create_visual_msg("cap", CAP_HEIGHT, gazsim_msgs::Color::RED);
  vis_msg.set_visible(false);
  
  visual_pub_->Publish(vis_msg);
  gazsim_msgs::WorkpieceResult msg;
  msg.set_puck_name(name());
  msg.set_color(cap_color_);
  this->workpiece_result_pub_->Publish(msg);
  have_cap = false;
}


msgs::Visual Puck::create_visual_msg(std::string element_name, double element_height, gazsim_msgs::Color clr)
{
  std::string parent_name = this->name().c_str();
  //model_->GetLink("cylinder")->GetParent()->GetName();
  // create a massage for visual control
  gazebo::msgs::Visual visual_msg;
  // the parent of the new visual is the workpiece itself
  visual_msg.set_parent_name(parent_name + "::cylinder");
  //visual_msg.set_parent_id(model_->GetLink("cylinder")->GetParent()->GetId());
  // set the name of the object
  visual_msg.set_name(parent_name + std::string("::cylinder::") + element_name);
  // no need for shadows on the visual
  visual_msg.set_cast_shadows(false);
  // get  a geometryfor the visual
  gazebo::msgs::Geometry *geom_msg = visual_msg.mutable_geometry();
  // the geomery is roughly a cylinder
  geom_msg->set_type(msgs::Geometry::CYLINDER);
  // this model is a cylinder, so the x and y params of its bounding box
  // should be equal, the double radius. so set the radius of the addition
  // according to it
  geom_msg->mutable_cylinder()->set_radius(this->model_->GetBoundingBox().GetXLength()/2);

  // get a height, where to spawn the new visual
  double vis_middle = WORKPIECE_HEIGHT;
  std::string color_name = gazsim_msgs::Color_Name(clr);
  printf("%s has recieved a %s %s\n", this->name().c_str(), color_name.c_str(), element_name.c_str());
  // calcualte the height for the next ring
  vis_middle = (WORKPIECE_HEIGHT/2) + this->ring_count_ * RING_HEIGHT + (element_height/2);
  printf("vis_height is: %f\n",vis_middle);
  // the height of a ring, in meters
  geom_msg->mutable_cylinder()->set_length(element_height);
  //set the color according to the message
  switch(clr){
    case gazsim_msgs::Color::RED:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(1,0,0));
      break;
    case gazsim_msgs::Color::BLUE:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(0,0,1));
      break;
    case gazsim_msgs::Color::GREEN:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(0,1,0));
      break;
    case gazsim_msgs::Color::BLACK:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(0,0,0));
      msgs::Set(visual_msg.mutable_material()->mutable_emissive(), common::Color(0,0,0,0));
      msgs::Set(visual_msg.mutable_material()->mutable_ambient(), common::Color(0,0,0,0));

      break;
    case gazsim_msgs::Color::YELLOW:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(255, 255, 0));
      break;
    case gazsim_msgs::Color::ORANGE:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(255, 127, 0));
      break;
    case gazsim_msgs::Color::GREY:
    default:
      msgs::Set(visual_msg.mutable_material()->mutable_diffuse(), common::Color(0.2,0.2,0.2));
      msgs::Set(visual_msg.mutable_material()->mutable_emissive(), common::Color(0.2,0.2,0.2,0.2));
      msgs::Set(visual_msg.mutable_material()->mutable_ambient(), common::Color(0.2,0.2,0.2,0.2));
      break;
  }
  // set the calculated pose for the visual
  msgs::Set(visual_msg.mutable_pose(),math::Pose(0,0,vis_middle,0,0,0));
  return visual_msg;
}

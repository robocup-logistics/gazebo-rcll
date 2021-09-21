/***************************************************************************
 *  tag.h - Plugin to spawn the right tag pattern and publish the pose.
 *          The name of the spawned model has to be something like
 *            'prefix/tag_01/suffix' to display the right tag pattern
 *
 *  Created: Fri Oct 16 17:59:54 2015
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

#ifndef TAG_H
#define TAG_H

#include <configurable/configurable.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <list>
#include <stdio.h>
#include <string.h>

#define TAG_SIZE config->get_float("plugins/tag/tag_size")
//At what simulation time to spawn the tag (too early and the tag spawns at (0, 0, 0))
#define TAG_SPAWN_TIME config->get_float("plugins/tag/tag_spawn_time")
#define TOPIC_TAG_POSE config->get_string("plugins/tag/topic_tag_pose").c_str()

namespace gazebo {
/**
   * Plugin to spawn the right tag pattern and publish the pose
   * @author Frederik Zwilling
   */
class Tag : public ModelPlugin, public gazebo_rcll::ConfigurableAspect
{
public:
	Tag();
	~Tag();

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
	///name of the tag and the communication channel
	std::string name_;

	///Publisher to send spawn tag patterns
	transport::PublisherPtr visPub_;
	double                  spawned_tags_last_;
	double                  created_time_;

	physics::WorldPtr world_;
};
} // namespace gazebo

#endif // TAG_H

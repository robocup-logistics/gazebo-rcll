/***************************************************************************
 *  mps_loader.cpp - loads an mps of a specific type for the plugin
 *
 *  Generated: Wed Apr 22 12:48:29 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "mps_opcua.h"
//#include "base_station.h"
//#include "ring_station.h"
//#include "cap_station.h"
//#include "delivery_station.h"
//#include "storage_station.h"

using namespace gazebo;
using namespace gazebo_rcll;
using namespace mps_comm;

// Register this plugin to make it available in the simulator
GZ_REGISTER_WORLD_PLUGIN(MpsOpcUaLoader)

MpsOpcUaLoader::MpsOpcUaLoader() : WorldPlugin() {
  printf("Starting MpsOpcUa Plugin!\n");
}

MpsOpcUaLoader::~MpsOpcUaLoader() { printf("Destructing MpsOpcUa Plugin!\n"); }

void MpsOpcUaLoader::Load(physics::WorldPtr _world, sdf::ElementPtr sdf) {

  try {
    if (config->get_bool("plugins/mps-opcua/enable")) {

      std::string prefix = "plugins/mps-opcua/stations/";
      std::set<std::string> mps_configs;
      std::set<std::string> ignored_mps_configs;

      std::shared_ptr<Configuration::ValueIterator> i(
          config->search(prefix.c_str()));
      while (i->next()) {
        std::string mps_name = std::string(i->path()).substr(prefix.length());
        mps_name = mps_name.substr(0, mps_name.find("/"));

        if ((mps_configs.find(mps_name) == mps_configs.end()) &&
            (ignored_mps_configs.find(mps_name) == ignored_mps_configs.end())) {

          std::string cfg_prefix = prefix + mps_name + "/";

          bool active = true;
          try {
            active = config->get_bool((cfg_prefix + "active").c_str());
          } catch (fawkes::Exception &e) {
          } // ignored, assume enabled

          if (active) {
            std::string mpstype =
                config->get_string((cfg_prefix + "type").c_str());
            std::string mpsip =
                config->get_string((cfg_prefix + "host").c_str());
            unsigned int port = config->get_uint((cfg_prefix + "port").c_str());

            std::string endpoint = std::string("opc.tcp://") + mpsip +
                                   std::string(":") + std::to_string(port) +
                                   std::string("/");

            printf(
                ("Opc Server starting for machine " + mps_name + "\n").c_str());
            printf(("Opc Sercver endpoint " + endpoint + "\n").c_str());
            mps_server_[mps_name] = std::make_shared<mps_comm::OPCServer>(
                mps_name, mpstype, mpsip, port);
            mps_server_[mps_name]->run_server();
            printf(
                ("Opc Server started for machine " + mps_name + "\n").c_str());
          }
          mps_configs.insert(mps_name);
        } else {
          ignored_mps_configs.insert(mps_name);
        }
      }
    }
  } catch (fawkes::Exception &e) {

    printf(e.what());
  }
}

void MpsOpcUaLoader::Reset() {}

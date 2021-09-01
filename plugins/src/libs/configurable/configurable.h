/***************************************************************************
 *  configurable.h - Configurable aspect for Gazebo plugins
 *
 *  Created: Mon Feb 15 15:00:18 2016
 *  Copyright  2016 Frederik Zwilling
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

#ifndef __ASPECT_CONFIGURABLE_H_
#define __ASPECT_CONFIGURABLE_H_

#include <config/yaml.h>

namespace gazebo_rcll {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ConfigurableAspect
{
public:
	ConfigurableAspect();
	~ConfigurableAspect();

protected:
	Configuration *config;
};

} // namespace gazebo_rcll

#endif

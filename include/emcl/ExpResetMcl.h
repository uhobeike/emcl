/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef EXP_PF_H__
#define EXP_PF_H__

#include "emcl/Mcl.h"
/*
#include <vector>
#include <sstream>
#include <random>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
*/

#include "std_msgs/UInt8.h"

namespace emcl {

class ExpResetMcl : public Mcl
{
public: 
	ExpResetMcl(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			double alpha_th, double open_space_th,
			double expansion_radius_position, double expansion_radius_orientation);
	~ExpResetMcl();

	bool sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, sensor_msgs::LaserScan &mode_scan, std_msgs::UInt8 &mode_scan_pattern);
private:
	double alpha_threshold_;
	double open_space_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	void expansionReset(void);
};

}

#endif

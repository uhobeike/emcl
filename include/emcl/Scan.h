/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef SCAN_H__
#define SCAN_H__

#include <iostream>
#include <vector>

namespace emcl {

class Scan
{
public: 
	int seq_;
	int scan_increment_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	double range_max_;
	double range_min_;

	double lidar_pose_x_;
	double lidar_pose_y_;
	double lidar_pose_yaw_;

	std::vector<double> ranges_;
	std::vector<double> thin_out_ranges_;

	std::vector<int> angles_;

	std::vector<uint16_t> directions_16bit_;

	Scan& operator=(const Scan &s);
	int countValidBeams(double *rate = NULL);
	int countValidThinOutBeams();
	bool valid(double range);
};

}

#endif

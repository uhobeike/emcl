/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef PARTICLE_H__
#define PARTICLE_H__

#include "emcl/Pose.h"
#include "emcl/LikelihoodFieldMap.h"
#include "emcl/Scan.h"

namespace emcl {



class Particle
{
public:
	Particle(double x, double y, double t, double w);

	double likelihood(LikelihoodFieldMap *map, Scan &scan);
	double likelihood(LikelihoodFieldMap *map, Particle &scan);
	double likelihood(LikelihoodFieldMap *map, Scan &scan, Particle &particle);
	void randomScan(Particle &p);

	Pose p_;
	double w_;
	Scan s_;

	Particle operator =(const Particle &p);

};

}

#endif

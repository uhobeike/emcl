/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/Particle.h"
#include "emcl/ParticleFilter.h"
#include <cmath>
#include <random>
#include <ros/ros.h>

namespace emcl {

int MIN = 0;
int MAX = 330;

constexpr int RAND_NUMS_TO_GENERATE = 2;

Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*ParticleFilter::cos_[t] 
				- scan.lidar_pose_y_*ParticleFilter::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*ParticleFilter::sin_[t] 
				+ scan.lidar_pose_y_*ParticleFilter::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);
	
	#if 1

	std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<int> distr(MIN, MAX);
	std::vector<int> result;
	
	for (int i=0;i<RAND_NUMS_TO_GENERATE;i++)
		result.push_back(distr(eng));
	std::sort(result.begin(), result.end());

	// if ((result[1] - result[0]) > 150)
	// 	result[1] = result[1] - 140;
	// std::sort(result.begin(), result.end());

	// std::cout << result[0] << ", " << result[1] << ", " <<  result.size() << "\n";
	
	double ans = 0.0;
	int test_count = 0;
	double scan_size_old = scan.thin_out_ranges_.size();	

	scan.thin_out_ranges_.clear();
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){

		if((not scan.valid(scan.ranges_[i])) || (i>=result[0] && i<=result[1])){
			continue;
		}

		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * ParticleFilter::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * ParticleFilter::sin_[a];
		
		ans += map->likelihood(lx, ly);
		test_count++;
		
		scan.thin_out_ranges_.push_back(scan.ranges_[i]);
	}
	// ans *= 1.0 + scan.thin_out_ranges_.size()/330;
	if(ans == 0)
		ans = 0.1;
	// std::cout << test_count << ", " << ans << "\n";

	#else
	double ans = 0.0;

	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * ParticleFilter::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * ParticleFilter::sin_[a];

		ans += map->likelihood(lx, ly);
	}
	
	#endif
	return ans;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}

/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/ExpResetMcl.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>

namespace emcl {

ExpResetMcl::ExpResetMcl(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
				double alpha_th, double open_space_th,
				double expansion_radius_position, double expansion_radius_orientation)
	: alpha_threshold_(alpha_th), open_space_threshold_(open_space_th),
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation), Mcl::Mcl(p, num, scan, odom_model, map)
{
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

  static bool init_random_scan_angle_flag = true;
  static Particle pra(0,0,0,0);
  if (init_random_scan_angle_flag)
  {
    int angle_num = 5;
    int angle_size = 360;
    int angle_size_min = 0;
    int angle_size_max = 90;
    pra.angles_.resize(angle_num);
    for (int i = 0; i < angle_num; i++)
    {
      for (int j = 0; j < angle_size; j++)
      {
        if((angle_size_min<=j) && (j<=angle_size_max) && (i < angle_num-1))
          continue;
        pra.angles_[i].push_back(j);
      }
      angle_size_min += 90;
      angle_size_max += 90;
    }    

    for(auto &p : particles_)
		  p.randomScan(p, pra);
    
    init_random_scan_angle_flag = false;
  }
  
  // for (auto& value : pra.angles_[4])
  // {
  //   std::cout << value;
  // }
  // std::cout << "\n";

  for (auto &p : particles_)
  {
    std::cout << p.angle_ << ", ";
  }
  std::cout << "\n";

	int i = 0;
	if (!inv) {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	} else {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_){
    double w = 0;
    w = p.likelihood(map_.get(), scan, p);
    w = (w/p.angles_[p.angle_].size())*100;
		p.w_ *= w;
    std::cout << w << ",";
  }
  std::cout << "\n";

	alpha_ = normalizeBelief()/valid_beams;
	//alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version
	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		expansionReset();
		for(auto &p : particles_)
			p.w_ *= p.likelihood(map_.get(), scan);
	}

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
}

void ExpResetMcl::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

}

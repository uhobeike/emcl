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

bool ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, sensor_msgs::LaserScan &mode_scan, std_msgs::UInt8 &mode_scan_pattern)
{
	if(processed_seq_ == scan_.seq_)
		return false;

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
    int angle_size = scan.ranges_.size();
    int angle_size_min = 0;
    int angle_size_max = scan.ranges_.size()/4;
    pra.angles_.resize(10);
    for (int i = 0; i < angle_num; i++)
    {
      for (int j = 0; j < angle_size; j++)
      {
        if((angle_size_min<=j) && (j<=angle_size_max) && (i < angle_num-1))
          continue;
        pra.angles_[i].push_back(j);
      }
      angle_size_min += scan.ranges_.size()/4;
      angle_size_max += scan.ranges_.size()/4;
    }
    angle_num = 9;
    angle_size_min = 0;
    angle_size_max = scan.ranges_.size()/2;
    for (int i = 5; i < angle_num; i++)
    {
      for (int j = 0; j < angle_size; j++)
      {
        if((angle_size_min<=j) && (j<=angle_size_max) && (i != 7))
          continue;
        else if (!((angle_size_min<=j) && (j<=angle_size_max)) && (i == 7))
          continue;
        if (i==5)
          pra.angles_[5].push_back(j);
        if (i==8)
          pra.angles_[6].push_back(j);
      }
      if (i != 6){
        angle_size_min += scan.ranges_.size()/4;
        angle_size_max += scan.ranges_.size()/4;
      }
    }

    angle_num = 9;
    angle_size_min = scan.ranges_.size()/4;
    angle_size_max = scan.ranges_.size()/2;
    for (int i = 7; i < angle_num; i++)
    {
      for (int j = 0; j < angle_size; j++)
      {
        if(((angle_size_min>=j) || ((j>=angle_size_max) && (angle_size_min*3>j))) && (i == 7))
          pra.angles_[7].push_back(j);
        else if (!((angle_size_min>=j) || ((j>=angle_size_max) && (angle_size_min*3>j))) && (i == 8))
          pra.angles_[8].push_back(j);
      }
    }

    angle_num = 10;
    angle_size_min = scan.ranges_.size()/8;
    angle_size_max = scan.ranges_.size()/4;
    for (int i = 9; i < angle_num; i++)
    {
      for (int j = 0; j < angle_size; j++)
      {
        if(((angle_size_min>=j) || ((j>=(angle_size_max+angle_size_min)) && ((angle_size_max*2+angle_size_min)>j)) || ((j>=(angle_size_max*3+angle_size_min)) && (scan.ranges_.size()>j))) && (i == 9))
          pra.angles_[9].push_back(j);
      }
    }

    // angle_num = 13;
    // angle_size = scan.ranges_.size();
    // angle_size_min = 0;
    // angle_size_max = scan.ranges_.size()/4;
    // for (int i = 9; i < angle_num; i++)
    // {
    //   for (int j = 0; j < angle_size; j++)
    //   {
    //     if(!((angle_size_min<=j) && (j<=angle_size_max)))
    //       continue;
    //     pra.angles_[i].push_back(j);
    //   }
    //   angle_size_min += scan.ranges_.size()/4;
    //   angle_size_max += scan.ranges_.size()/4;
    // }

    for(auto &p : particles_)
		  p.randomScan(p, pra);
    
    init_random_scan_angle_flag = false;
  }
  
  // for (auto& value : pra.angles_[7])
  // {
  //   std::cout << value << ",";
  // }
  // std::cout << "\n";

  std::vector<int8_t> scan_angle;

  for (auto &p : particles_)
  {
    // std::cout << p.angle_ << ", ";
    scan_angle.push_back(p.angle_);
  }
  // std::cout << "\n";

  // 集計する
  std::vector<size_t> count(256, 0);
  for(const auto &x : scan_angle){
      ++count[x];
  }
  auto max_iterator = std::max_element(count.begin(), count.end());
  size_t mode = std::distance(count.begin(), max_iterator);
  // std::cout << "最頻値：" << mode << std::endl;

  mode_scan_pattern.data = static_cast<int>(mode);

  int cnt = 0;
  int cnt2 = 0;
  for (auto &sr : mode_scan.ranges)
  {
    if (pra.angles_[mode].size() > cnt2){

      if (pra.angles_[mode].at(cnt2) == cnt){
        ++cnt2;
      }
      else
        sr = 0;
    }
    else
      sr = 0;


    ++cnt;
    // std::cout << cnt << ", "<< cnt2 << ", " << pra.angles_[mode].size() << std::endl;
  }

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
		return false;

	for(auto &p : particles_){
    double w = 0;
    uint16_t valid_beams_cnt = 0;
    w = p.likelihood(map_.get(), scan, p, valid_beams_cnt);
    w = (w/(p.angles_[p.angle_].size() - valid_beams_cnt))*100;
		p.w_ *= w;
    // std::cout << int(w) << ",";
  }
  // std::cout << "\n";

	alpha_ = normalizeBelief()/100;
	//alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version
	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);

  static bool exp_flag = false;
	if(alpha_ < alpha_threshold_ ){
		ROS_INFO("RESET");
		expansionReset();
    for(auto &p : particles_){
      double w = 0;
      w = p.likelihood(map_.get(), scan);
      w = (w/p.angles_[p.angle_].size())*100;
      p.w_ *= w;
      // std::cout << w << ",";
    }
    exp_flag = true;
	}

	if(normalizeBelief() > 0.000001){
		resampling(exp_flag);
    exp_flag = false;
  }
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
  return true;
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

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Collects together all includes for flirtlib, and a struct to wrap
 * the various objects
 *
 * \author Bhaskara Marthi
 */

#ifndef FLIRTLIB_ROS_FLIRTLIB_H
#define FLIRTLIB_ROS_FLIRTLIB_H

#define BOOST_NO_HASH

#include <flirtlib/feature/Detector.h>
#include <flirtlib/feature/ShapeContext.h>
#include <flirtlib/feature/BetaGrid.h>
#include <flirtlib/feature/RangeDetector.h>
#include <flirtlib/feature/CurvatureDetector.h>
#include <flirtlib/feature/NormalBlobDetector.h>
#include <flirtlib/feature/NormalEdgeDetector.h>
#include <flirtlib/feature/RansacFeatureSetMatcher.h>
#include <flirtlib/feature/RansacMultiFeatureSetMatcher.h>
#include <flirtlib/sensorstream/CarmenLog.h>
#include <flirtlib/sensorstream/LogSensorStream.h>
#include <flirtlib/sensorstream/SensorStream.h>
#include <flirtlib/utils/SimpleMinMaxPeakFinder.h>
#include <flirtlib/utils/HistogramDistances.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <ros/ros.h>

namespace flirtlib_ros
{

typedef std::vector<InterestPoint*> InterestPointVec;

class FlirtlibFeatures
{
public:
  /// Initialize Flirtlib objects based on ros parameters.  Specifying the
  /// node handle argument allows selecting the ros namespace in which the
  /// parameters will be searched for.
  FlirtlibFeatures (ros::NodeHandle nh = ros::NodeHandle("~"));

  /// Extracts the features and their descriptors
  InterestPointVec extractFeatures (sensor_msgs::LaserScan::ConstPtr scan) const;

  /// Extracts points from a scan and transforms them to a different frame
  InterestPointVec extractFeatures (const sensor_msgs::LaserScan& scan, const tf::StampedTransform& transform) const;

  /// Extracts the features from multiple scans
  InterestPointVec extractFeatures(const std::vector<sensor_msgs::LaserScan>& scans, const std::vector<tf::StampedTransform>& transforms) const;

  /// Returns the ransac matcher, should be replaced by a generic matching interface
  /// that supports different matching
  boost::shared_ptr<RansacFeatureSetMatcher> matcher()
  {
    return ransac_;
  }

 private:

  ros::NodeHandle nh_;

  std::string detector_type_;
  std::string descriptor_type_;
  std::string matcher_type_;
  double detector_scale_;
  double detector_minimum_spanning_trees_num_;
  double detector_base_sigma_;
  double detector_sigma_step_;
  int detector_window_;
  double descriptor_min_rho_;
  double descriptor_max_rho_;
  double descriptor_bin_rho_;
  double descriptor_bin_phi_;

  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
};

} // namespace

#endif // include guard

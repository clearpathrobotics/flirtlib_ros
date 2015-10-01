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
 * Implementation of flirtlib.h
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <boost/foreach.hpp>
#include <boost/iterator/zip_iterator.hpp>

namespace flirtlib_ros
{

using std::string;
namespace sm=sensor_msgs;



FlirtlibFeatures::FlirtlibFeatures (ros::NodeHandle nh)
  : nh_(nh)
  , detector_type_("curvature")
  , descriptor_type_("Shape Context")
  , matcher_type_("ransac")
  , detector_scale_(5.0)
  , detector_minimum_spanning_trees_num_(2.0)
  , detector_base_sigma_(0.2)
  , detector_sigma_step_(1.4)
  , detector_window_(3)
  , descriptor_min_rho_(0.02)
  , descriptor_max_rho_(0.5)
  , descriptor_bin_rho_(8)
  , descriptor_bin_phi_(30)
{

  nh_.param("detector_type",  detector_type_,  detector_type_);
  nh_.param("descriptor_type", descriptor_type_, descriptor_type_);
  nh_.param("detector_scale", detector_scale_, detector_scale_);
  nh_.param("detector_minimum_spanning_trees_num", detector_minimum_spanning_trees_num_, detector_minimum_spanning_trees_num_);
  nh_.param("detector_base_sigma", detector_base_sigma_, detector_base_sigma_);
  nh_.param("detector_sigma_step", detector_sigma_step_, detector_sigma_step_);
  nh_.param("detector_window_", detector_window_, detector_window_);
  nh_.param("descriptor_min_rho", descriptor_min_rho_, descriptor_min_rho_);
  nh_.param("descriptor_max_rho", descriptor_max_rho_, descriptor_max_rho_);
  nh_.param("descriptor_bin_rho", descriptor_bin_rho_, descriptor_bin_rho_);
  nh_.param("descriptor_bin_phi", descriptor_bin_phi_, descriptor_bin_phi_);

  peak_finder_.reset(new SimpleMinMaxPeakFinder(0.34, 0.001));
  histogram_dist_.reset(new SymmetricChi2Distance<double>());

  peak_finder_ = boost::shared_ptr<SimpleMinMaxPeakFinder>(new SimpleMinMaxPeakFinder(0.34, 0.001));
  histogram_dist_ = boost::shared_ptr<HistogramDistance<double> >(new SymmetricChi2Distance<double>());

  // Detector
  if (detector_type_ == "curvature")
  {
    CurvatureDetector * det = new CurvatureDetector(peak_finder_.get(), detector_scale_, detector_base_sigma_,
                                                    detector_sigma_step_,detector_minimum_spanning_trees_num_);
    det->setUseMaxRange(false);
    detector_ = boost::shared_ptr<Detector>(det);
  }
  else if (detector_type_ == "normal_edge")
  {
    detector_ .reset(new NormalEdgeDetector(peak_finder_.get(), detector_scale_, detector_base_sigma_,
                                            detector_sigma_step_,  detector_window_));
  }
  else if (detector_type_ == "normal blob")
  {
    detector_.reset(new NormalBlobDetector(peak_finder_.get(),detector_scale_, detector_base_sigma_,
                                           detector_sigma_step_,  detector_window_));
  }
  else if (detector_type_ == "range")
  {
    detector_.reset(new RangeDetector(peak_finder_.get(), detector_scale_, detector_base_sigma_,
                                      detector_sigma_step_));
  }

  // Descriptor
  if(descriptor_type_ == "Beta Grid")
  {
    BetaGridGenerator * gen = new BetaGridGenerator(descriptor_min_rho_, descriptor_max_rho_,
                                                    descriptor_bin_rho_, descriptor_bin_phi_);
    gen->setDistanceFunction(histogram_dist_.get());
    descriptor_ .reset(gen);
  }
  if(descriptor_type_ == "Shape Context")
  {
    ShapeContextGenerator * gen = new ShapeContextGenerator(descriptor_min_rho_, descriptor_max_rho_,
                                                            descriptor_bin_rho_, descriptor_bin_phi_);
    gen->setDistanceFunction(histogram_dist_.get());
    descriptor_ .reset(gen);
  }

  ransac_.reset(new RansacFeatureSetMatcher(0.0299, 0.95, 0.2, 0.1,0.0184, false));
}

// Extract flirtlib features
InterestPointVec
FlirtlibFeatures::extractFeatures (sm::LaserScan::ConstPtr scan) const
{
  boost::shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(descriptor_->describe(*p, *reading));
  return pts;
}

// Extract flirtlib features
InterestPointVec
FlirtlibFeatures::extractFeatures (const sm::LaserScan& scan, const tf::StampedTransform& transform) const
{
  boost::shared_ptr<LaserReading> reading = fromRos(scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts)
  {
    p->setDescriptor(descriptor_->describe(*p, *reading));
    tf::Vector3 point(p->getPosition().x, p->getPosition().y, 0);
    point = transform*point;
    p->setPosition(OrientedPoint2D(point.getX(),point.getY(), p->getPosition().theta));
  }
  return pts;
}


// Extract from multiple scans
InterestPointVec
FlirtlibFeatures::extractFeatures (const std::vector<sensor_msgs::LaserScan>& scans, const std::vector<tf::StampedTransform>& transforms) const
{
  InterestPointVec pts;
  std::vector<sensor_msgs::LaserScan>::const_iterator scans_iter = scans.begin();
  std::vector<tf::StampedTransform>::const_iterator tf_iter = transforms.begin();
  for(; scans_iter != scans.end(); ++scans_iter, ++tf_iter)
  {
    InterestPointVec v = extractFeatures(*scans_iter, *tf_iter);
    pts.insert(pts.end(), v.begin(), v.end());
  }

  return pts;

}


} // namespace





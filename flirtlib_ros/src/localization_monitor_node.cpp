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
 * Ros node that updates and localizes using a set of saved scans.  The idea
 * is that you can run this all the time in the background, and it will both
 * do startup localization, and update itself over time as the map changes.
 * 
 * Requires:
 * - The nav stack (amcl, move_base) to be running
 * - Incoming laser scans on base_scan topic
 * 
 * Provides
 * - At most once per execution, will publish a pose on initialpose,
 *   if it decides the robot is poorly localized and it finds a good match among
 *   the saved scans.
 * - Visualization of interest point, reference scan locations
 * - Saves new scans to the db; tries to cover x-y-theta space with scans.
 *
 * Parameters:
 * - min_num_matches: #feature matches required for a succesful scan match.
 * - min_successful_navs: Number of succesful navigations that must be observed
 *   before we start saving scans.  Defaults to 1.
 * - db_name: Name of database.  Defaults to flirtlib_place_rec.
 * - db_host: Hostname where db is running.  Defaults to localhost.
 * - quality_threshold: Bound on quality parameter to consider robot 
 *   well-localized.  Defaults to 0.25.
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/localization_monitor.h>
#include <flirtlib_ros/conversions.h>
#include <flirtlib_ros/common.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mongo_ros/message_collection.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseActionResult.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;
namespace mbm=move_base_msgs;

using std::string;
using std::vector;
using boost::format;
using boost::shared_ptr;

typedef boost::mutex::scoped_lock Lock;
typedef mr::MessageWithMetadata<RefScanRos>::ConstPtr DBScan;
typedef vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;
typedef vector<RefScan> RefScans;

class Node
{
public:

  Node ();

  // Update given new laser scan.
  void scanCB (sm::LaserScan::ConstPtr scan);
  
  // Save the map the first time it comes in
  void mapCB (const nm::OccupancyGrid& g);
  
  // Used to count successful navigations, as an indicator that we're
  // well localized.
  void navCB (const mbm::MoveBaseActionResult& m);
  
private:
  
  // Extract FLIRT features from a scan
  InterestPointVec extractFeatures (sm::LaserScan::ConstPtr scan) const;

  // Update given that we're well localized
  void updateLocalized (sm::LaserScan::ConstPtr scan, const gm::Pose& p);
  
  // Update given that we're not well localized
  void updateUnlocalized (sm::LaserScan::ConstPtr scan);
  
  // Republish the set of reference scan poses.
  void publishRefScans () const;
  
  // Compensate for base movement between when the scan was taken and now
  tf::Transform compensateOdometry (const tf::Pose& sensor_pose,
                                    const string& frame, const ros::Time& t1,
                                    const ros::Time& t2);
  
  /************************************************************
   * Needed during init
   ************************************************************/
  
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  
  /************************************************************
   * Parameters
   ************************************************************/ 

  // Minimum number of feature matches needed for succesful scan match
  unsigned min_num_matches_; 
  
  // Minimum number of navigations to happen before we'll save new scans
  unsigned min_successful_navs_;
  
  // Name of the database
  string db_name_;
  
  // Host where the db is running
  string db_host_;
  
  // Rate at which we'll consider new scans
  ros::Rate update_rate_;
  
  // Bound on quality to consider the robot well localized
  double quality_threshold_;
  
  /************************************************************
   * Flirtlib objects
   ************************************************************/
  
  shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  shared_ptr<HistogramDistance<double> > histogram_dist_;
  shared_ptr<Detector> detector_;
  shared_ptr<DescriptorGenerator> descriptor_;
  shared_ptr<RansacFeatureSetMatcher> ransac_;

  /************************************************************
   * Mutable state
   ************************************************************/
  
  // Local copy of db scans. 
  vector<RefScan> ref_scans_;
  
  // Offset between laser and base frames
  tf::Transform laser_offset_;
  
  // How many times have we published a succesful match on initialpose so far
  unsigned match_counter_;
  
  // How many successful navigations have we observed
  unsigned successful_navs_;
  
  /************************************************************
   * Associated objects
   ************************************************************/

  // Evaluates localization quality based on the scan and the static map
  shared_ptr<ScanPoseEvaluator> evaluator_;
  
  // Db collection of scans
  mr::MessageCollection<RefScanRos> scans_;
  
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher ref_scan_pose_pub_;
  ros::Publisher pose_est_pub_;
  ros::Publisher match_pose_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber nav_result_sub_;
};


Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 5.0;
  const double dmst = 2.0;
  const double base_sigma = 0.2;
  const double sigma_step = 1.4;
  CurvatureDetector* det = new CurvatureDetector(peak_finder, scale, base_sigma,
                                                 sigma_step, dmst);
  det->setUseMaxRange(false);
  return det;
}

DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
{
  const double min_rho = 0.02;
  const double max_rho = 0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}


// Constructor sets up ros comms and flirtlib objects, loads scans from db,
// and figures out the offset between the base and the laser frames
Node::Node () :
  min_num_matches_(getPrivateParam<int>("min_num_matches", 8)),
  min_successful_navs_(getPrivateParam<int>("min_successful_navs", 1)),
  db_name_(getPrivateParam<string>("db_name", "flirtlib_place_rec")),
  db_host_(getPrivateParam<string>("db_host", "localhost")),
  update_rate_(1.0), quality_threshold_(0.25),
  peak_finder_(new SimpleMinMaxPeakFinder(0.34, 0.001)),
  histogram_dist_(new SymmetricChi2Distance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.95, 0.4, 0.4,
                                      0.0384, false)),
  match_counter_(0), successful_navs_(0),
  scans_(db_name_, "scans", db_host_),
  scan_sub_(nh_.subscribe("base_scan", 10, &Node::scanCB, this)),
  map_sub_(nh_.subscribe("map", 10, &Node::mapCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  ref_scan_pose_pub_(nh_.advertise<gm::PoseArray>("ref_scan_poses", 10, true)),
  pose_est_pub_(nh_.advertise<gm::PoseStamped>("pose_estimate", 1)),
  match_pose_pub_(nh_.advertise<gm::PoseArray>("match_poses", 1)),
  initial_pose_pub_(nh_.advertise<gm::PoseWithCovarianceStamped>("initialpose", 1)),
  nav_result_sub_(nh_.subscribe("move_base/result", 10, &Node::navCB, this))
{
  ROS_DEBUG_NAMED("init", "Waiting for laser offset");
  laser_offset_ = getLaserOffset(tf_);

  ROS_DEBUG_NAMED("init", "Loading scans from db");
  BOOST_FOREACH (const mr::MessageWithMetadata<RefScanRos>::ConstPtr m,
                 scans_.queryResults(mr::Query(), false)) 
    ref_scans_.push_back(fromRos(*m));
  publishRefScans();
  ROS_INFO("Localization monitor initialized with %zu scans", ref_scans_.size());
}

// Update counter of successful navs observed
void Node::navCB (const mbm::MoveBaseActionResult& res)
{
  if (res.status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    successful_navs_++;
    if (successful_navs_==min_successful_navs_)
    {
      ROS_INFO ("Have now observed %u successful navigations.  Can "
                "now start saving scans", successful_navs_);
    }
  }
}

// Publish the poses of the reference scans (for visualization)
void Node::publishRefScans () const
{
  gm::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "/map";
  BOOST_FOREACH (const RefScan& s, ref_scans_) 
    poses.poses.push_back(s.pose);
  ref_scan_pose_pub_.publish(poses);
}

// Use the map to initialize the localization evaluator
void Node::mapCB (const nm::OccupancyGrid& g)
{
  Lock l(mutex_);
  ROS_INFO("Received map; setting scan evaluator");
  evaluator_.reset(new ScanPoseEvaluator(g));
  ROS_INFO("Scan evaluator initialized");
}

// Extract flirtlib features
InterestPointVec Node::extractFeatures (sm::LaserScan::ConstPtr scan) const
{
  shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(descriptor_->describe(*p, *reading));
  return pts;
}



// If we're not well localized, try to localize by matching against the scans in the db
void Node::updateUnlocalized (sm::LaserScan::ConstPtr scan)
{
  // Extract features from curent scan
  gm::Pose current = getCurrentPose(tf_, "base_footprint");
  ROS_INFO("Not well localized");
  InterestPointVec pts = extractFeatures(scan);
  marker_pub_.publish(interestPointMarkers(pts, current));
      
  // Set up 
  gm::PoseArray match_poses;
  match_poses.header.frame_id = "/map";
  match_poses.header.stamp = ros::Time::now();
  unsigned best_num_matches = 0;
  gm::Pose best_pose;
  
  // Iterate over reference scans and match against each one, looking for the
  // one with the largest number of feature matches
  BOOST_FOREACH (const RefScan& ref_scan, ref_scans_) 
  {
    Correspondences matches;
    OrientedPoint2D trans;
    ransac_->matchSets(ref_scan.raw_pts, pts, trans, matches);
    const unsigned num_matches = matches.size();
    if (num_matches > min_num_matches_) 
    {
      ROS_INFO_NAMED ("match", "Found %d matches with ref scan at "
                      "%.2f, %.2f, %.2f", num_matches,
                      ref_scan.pose.position.x, ref_scan.pose.position.y,
                      tf::getYaw(ref_scan.pose.orientation));
      match_poses.poses.push_back(ref_scan.pose);
      const gm::Pose laser_pose = transformPose(ref_scan.pose, trans);
      if (num_matches > best_num_matches)
      {
        best_num_matches = num_matches;
        best_pose = laser_pose;
      }
    }
  }

  // Only proceed if there are a sufficient number of feature matches
  if (best_num_matches<min_num_matches_)
    return;

  match_pose_pub_.publish(match_poses);
  const double quality = (*evaluator_)(*scan, best_pose);
  ROS_INFO ("Quality is %.2f", quality);
  
  // Do a further check of scan quality before committing to this
  // We'll always publish a visualization, but we'll only publish on the
  // initialpose topic once (so after that, this node will go into a state where
  // all it does is possibly save new scans).
  if (quality < quality_threshold_)
  {
    ROS_INFO("Found a good match");
    
    /*const gm::Pose adjusted_pose = evaluator_->adjustPose(*scan, best_pose,
                                                          .3, .05, .5);
    ROS_INFO_STREAM ("Adjusted " << best_pose << " to " << adjusted_pose);
    */
    // This is for visualization only
    const ros::Time now = ros::Time::now();
    tf::Pose adjusted_pose(compensateOdometry(poseMsgToTf(best_pose),
                                              scan->header.frame_id,
                                              scan->header.stamp, now));
    

    gm::PoseStamped estimated_pose;
    tf::poseTFToMsg(adjusted_pose*laser_offset_, estimated_pose.pose);
    estimated_pose.header.frame_id = "/map";
    estimated_pose.header.stamp = now;
    pose_est_pub_.publish(estimated_pose);
    
    // Only publish initialpose once
    if (match_counter_==0)
    {
      match_counter_++;
      gm::PoseWithCovarianceStamped initial_pose;
      initial_pose.header.frame_id = "/map";
      initial_pose.header.stamp = scan->header.stamp;
      initial_pose.pose.pose = estimated_pose.pose;
      initial_pose_pub_.publish(initial_pose);
    }
  }
}

/// Compensate for base movement between scan time and current time using 
/// odometry
tf::Transform Node::compensateOdometry (const tf::Pose& pose,
                                        const string& frame,
                                        const ros::Time& t1,
                                        const ros::Time& t2)
{
  const string FIXED_FRAME = "odom_combined";
  tf_.waitForTransform(FIXED_FRAME, frame, t1, ros::Duration(0.05));
  tf_.waitForTransform(FIXED_FRAME, frame, t2, ros::Duration(0.05));

  tf::StampedTransform current_odom, prev_odom;
  tf_.lookupTransform(FIXED_FRAME, frame, t2, current_odom);
  tf_.lookupTransform(FIXED_FRAME, frame, t1, prev_odom);
  tf::Transform current_to_prev = prev_odom.inverse()*current_odom;
  return pose*current_to_prev;
}
                                        

// Search the db for a nearby scan.  If none is found, and also we've
// successfully navigated previously (as a further cue that we're well
// localized), then add this scan to the db, and remove any
// nearby old scans.
void Node::updateLocalized (sm::LaserScan::ConstPtr scan,
                            const gm::Pose& current)
{
  ROS_INFO("Well localized");
  const double DPOS = 0.7;
  const double DTHETA = 1;
  const double DT = 86400; // one day
  
  // Query the db for nearby scans
  const double t = ros::Time::now().toSec();
  const double x = current.position.x;
  const double x0 = x - DPOS;
  const double x1 = x + DPOS;
  const double y = current.position.y;
  const double y0 = y - DPOS;
  const double y1 = y + DPOS;
  const double th = tf::getYaw(current.orientation);
  const double th0 = th - DTHETA;
  const double th1 = th + DTHETA;
  const double min_time = t-DT;
  format f("{x : {$gt: %.4f, $lt: %.4f}, y : {$gt: %.4f, $lt: %.4f}, "
           "theta: {$gt: %.4f, $lt: %.4f}, creation_time: {$gt: %.8f} }");
  string str = (f % x0 % x1 % y0 % y1 % th0 % th1 % min_time).str();
  mongo::Query q = mongo::fromjson(str);
  vector<DBScan> scans = scans_.pullAllResults(q, true);
  
  // Possibly save this new scan
  if (scans.size()<1 && successful_navs_>=min_successful_navs_) 
  {
    // First remove old scans that are nearby
    format old("{x : {$gt: %.4f, $lt: %.4f}, y : {$gt: %.4f, $lt: %.4f}, "
               "theta: {$gt: %.4f, $lt: %.4f} }");
    const string old_query_str = (old % x0 % x1 % y0 % y1 % th0 % th1).str();
    const mongo::Query old_query = mongo::fromjson(old_query_str);
    const vector<DBScan> old_scans = scans_.pullAllResults(old_query, true);
    scans_.removeMessages(old_query);
    BOOST_FOREACH (const DBScan& old_scan, old_scans) 
    {
      ROS_INFO ("Removed old scan at (%.4f, %.4f, %.f) taken at %.4f",
                old_scan->lookupDouble("x"), old_scan->lookupDouble("y"),
                old_scan->lookupDouble("theta"),
                old_scan->lookupDouble("creation_time"));
    }

    // Now add this new scan
    InterestPointVec pts = extractFeatures(scan);
    sm::LaserScan::Ptr stripped_scan(new sm::LaserScan(*scan));
    stripped_scan->ranges.clear();
    stripped_scan->intensities.clear();
    RefScan ref_scan(stripped_scan, current, pts);
    ref_scans_.push_back(ref_scan);
    scans_.insert(toRos(ref_scan),
                  mr::Metadata("x", x, "y", y, "theta", th));
    ROS_DEBUG_NAMED ("save_scan", "Saved scan at %.2f, %.2f, %.2f", x, y, th);
    
    publishRefScans();
  }
}


// Given current scan and pose, update state.  If well localized, we'll 
// consider adding this new scan to the db.  If not, we'll try to localize
// using the db.
void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  // First, look up the sensor pose at the time of the scan
  const std::string frame = scan->header.frame_id;
  tf_.waitForTransform("/map", frame, scan->header.stamp, ros::Duration(0.5));
  tf::StampedTransform trans;
  try
  {
    tf_.lookupTransform("/map", frame, scan->header.stamp, trans);
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO_STREAM ("Skipping scan due to tf exception " << e.what());
    return;
  }
  
  // Have we initialized the evaluator yet?
  Lock l(mutex_);
  if (!evaluator_)
  {
    ROS_INFO_STREAM ("Skipping scan as evaluator not yet initialized");
    return;
  }
    
  // Evaluate how well this scan is localized
  const gm::Pose pose = tfTransformToPose(trans);
  const double dist = (*evaluator_)(*scan, pose);
  ROS_INFO_THROTTLE (1.0, "Localization quality is %.2f", dist);
  
  if (dist < quality_threshold_)
    updateLocalized(scan, pose);
  else
    updateUnlocalized(scan);
  
  // Make this callback happen at a bounded rate
  update_rate_.sleep();
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor_node");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}

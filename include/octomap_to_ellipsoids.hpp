/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */
#ifndef OCTOMAP_TO_ELLIPSOIDS_HPP
#define OCTOMAP_TO_ELLIPSOIDS_HPP

#include "ellipsoids_markers.hpp"
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "gmr.h"

class OctomapToEllipsoids
{
public:

    OctomapToEllipsoids();
    ~OctomapToEllipsoids();

    bool startNode(int argc, char **argv);

private:

    void publishEllipsoids();

    void getOctomap( octomap_msgs::Octomap::ConstPtr octomap );
    void getMoveitOctomap( moveit_msgs::PlanningScene::ConstPtr msg );
    void computeGMMs( const std::vector<gmm::Vector>& points, const gmm::BBox& box );
    void getEllipsoids();
    void ellipsoidsToMarkers();

    void saveCloudToFile();
    bool loadCloudFromFile();
    void getPointCloud();
    bool isInBoundingBox( double x, double y, double z ) const;

    void getGmmPointsFromOctomap( std::vector<gmm::Vector>& points, gmm::BBox& box );
    void getGmmPointsFromPCL( std::vector<gmm::Vector>& points, gmm::BBox& box );

    ros::Publisher viz_pub_;
    ros::Publisher ellipsoid_pub_;
    ros::Subscriber sub_octomap_;
    ros::NodeHandle* nh_;

    bool publish_as_markers_;
    bool use_moveit_octomap_;
    bool use_stored_pointcloud_;
    double spin_rate_;
    std::string octomap_topic_;
    std::vector<double> bounding_box_;
    int nb_ellipsoids_;

    int messages_received_;
    octomap::OcTree* octomap_;
    std::string frame_id_;
    bool new_octomap_;

    boost::mutex gmm_mutex_;
    boost::mutex octomap_mutex_;

    std::vector<Eigen::Vector3d> mean_;
    std::vector<Eigen::Matrix3d> covariance_;

    boost::shared_ptr<EllipsoidsToMarkers> display_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

#endif // OCTOMAP_TO_ELLIPSOIDS_HPP


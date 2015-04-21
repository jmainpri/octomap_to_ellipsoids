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
#ifndef ELLIPSOIDS_MARKERS_HPP
#define ELLIPSOIDS_MARKERS_HPP

// Warning
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>

#include <string>

class EllipsoidsToMarkers
{
public:
    EllipsoidsToMarkers();
    ~EllipsoidsToMarkers();

    visualization_msgs::Marker boxMarker(int id, const std::vector<double>& box );
    visualization_msgs::Marker pointcloudMarker(int id, const pcl::PointCloud<pcl::PointXYZ>& cloud );
    visualization_msgs::Marker vectorMarker( int id, const Eigen::Vector3d& vector, const Eigen::Vector3d& position );
    visualization_msgs::Marker ellipsoidMarker( int& id, const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance, std::vector<visualization_msgs::Marker>& eigen_vectors );

    std::string frame_id_;

    tf::TransformBroadcaster br_;
};

#endif

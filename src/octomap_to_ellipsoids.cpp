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

#include "octomap_to_ellipsoids.hpp"

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <ellipsoid_msgs/EllipsoidArray.h>
#include <ellipsoid_msgs/Ellipsoid.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <stdio.h>

using std::cout;
using std::endl;

OctomapToEllipsoids::OctomapToEllipsoids()
{
    spin_rate_ = 10.;
    messages_received_ = 0;
    octomap_ = NULL;
    frame_id_ = "/camera_link";
    new_octomap_ = false;
    nb_ellipsoids_ = 10;
    bounding_box_.resize(6);
    bounding_box_[0] = 0;
    bounding_box_[1] = 0;
    bounding_box_[2] = 0;
    bounding_box_[3] = 0;
    bounding_box_[4] = 0;
    bounding_box_[5] = 0;
}

OctomapToEllipsoids::~OctomapToEllipsoids()
{

}

void OctomapToEllipsoids::publishEllipsoids()
{
    ellipsoid_msgs::EllipsoidArray array;

    // Set frame id and stamp
    array.frame_id = frame_id_;
    array.header.stamp = ros::Time::now();

    for( size_t i=0; i<mean_.size(); i++ )
    {
        // getting the distribution eigen vectors and values
        Eigen::EigenSolver<Eigen::MatrixXd> es( covariance_[i] );
        Eigen::VectorXcd eig_val = es.eigenvalues();
        Eigen::MatrixXcd V = es.eigenvectors();

        // building the rotation matrix
        Eigen::Vector3d eig_x = V.col(0).real().normalized();
        Eigen::Vector3d eig_y = V.col(1).real().normalized();
        Eigen::Vector3d eig_z = eig_x.cross( eig_y );

        // Get orientation
        Eigen::Matrix3d m;
        m.col(0) = eig_x;
        m.col(1) = eig_y;
        m.col(2) = eig_z;
        Eigen::Quaterniond q( m );

        ellipsoid_msgs::Ellipsoid ellipsoid;

        // Set mean
        ellipsoid.pose.position.x = mean_[i][0];
        ellipsoid.pose.position.y = mean_[i][1];
        ellipsoid.pose.position.z = mean_[i][2];

        // Set orientation
        ellipsoid.pose.orientation.x = q.x();
        ellipsoid.pose.orientation.y = q.y();
        ellipsoid.pose.orientation.z = q.z();
        ellipsoid.pose.orientation.w = q.w();

        // Set eigen values
        ellipsoid.eigen_val_x = eig_val[0].real();
        ellipsoid.eigen_val_y = eig_val[1].real();
        ellipsoid.eigen_val_z = eig_val[2].real();

        // Array
        array.ellipsoids.push_back( ellipsoid );
    }

    ellipsoid_pub_.publish( array );
}

void OctomapToEllipsoids::ellipsoidsToMarkers()
{
    {
        boost::mutex::scoped_lock lock( gmm_mutex_ );

        if( messages_received_ == 0 )
            return;

        cout << "publish markers" << endl;

        // Set frame id for the publishing
        display_->frame_id_ = frame_id_;

        int id = 0;
        for( size_t i=0; i<mean_.size(); i++ )
        {
            // Warning, this removes dull cluster, do that properly
            if( mean_[i] != Eigen::VectorXd::Zero(mean_[i].size()) )
            {
                std::vector<visualization_msgs::Marker> eigen_values;

                // cout << "mean[" << i << "] : " << mean_[i].transpose() << endl;

                id = i;

                visualization_msgs::Marker marker = display_->ellipsoidMarker( id, mean_[i], covariance_[i], eigen_values );
                viz_pub_.publish( marker );

                for( size_t j=0; j<eigen_values.size(); j++ )
                    viz_pub_.publish( eigen_values[j] );
            }
        }
    }

    visualization_msgs::Marker marker = display_->boxMarker( 0, bounding_box_ );
    viz_pub_.publish( marker );

    // Save to file
    saveCloudToFile();

    // ros::Duration(0.2).sleep();
}

void OctomapToEllipsoids::getEllipsoids()
{
    if( new_octomap_ )
    {
        // Get points from octomap
        std::vector<gmm::Vector> points;
        gmm::BBox bounding_box;
        getGmmPointsFromOctomap( points, bounding_box );

        // compute GMMs
        if( !points.empty() )
            computeGMMs( points, bounding_box );

        new_octomap_ = false;
    }
}


void OctomapToEllipsoids::saveCloudToFile()
{
    if( octomap_ == NULL )
        return;

    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    {
        boost::mutex::scoped_lock lock( octomap_mutex_) ;

        std::size_t octree_depth = octomap_->getTreeDepth();

        // 14 for sparser tree
        for( octomap::OcTree::iterator it = octomap_->begin( octree_depth ), end = octomap_->end(); it != end; ++it)
            // for( octomap::OcTree::leaf_iterator it = octomap->begin_leafs(), end=octomap->end_leafs(); it!= end; ++it)
        {
            if( octomap_->isNodeOccupied(*it) )
            {
                pcl::PointXYZ pos;
                pos.x = it.getX();
                pos.y = it.getY();
                pos.z = it.getZ();

                if( isInBoundingBox( pos.x, pos.y, pos.z ) )
                    cloud_->points.push_back( pos );
            }
        }
    }

    cloud_->width     = 1;
    cloud_->height    = cloud_->points.size();
    cloud_->is_dense  = false;

    if( cloud_->height > 0 )
    {
        pcl::io::savePCDFileASCII( "test_pcd.pcd", *cloud_ );

        // Send point cloud
        visualization_msgs::Marker marker = display_->pointcloudMarker( 0, *cloud_ );
        viz_pub_.publish( marker );
    }
}

bool OctomapToEllipsoids::loadCloudFromFile()
{
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    if( pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud_) == -1 ) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return false;
    }
    std::cout << "Loaded "
              << cloud_->width * cloud_->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    /**
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
    **/

    return true;
}

bool OctomapToEllipsoids::isInBoundingBox( double x, double y, double z ) const
{
    if( x < bounding_box_[0] )
        return false;
    if( x > bounding_box_[1] )
        return false;

    if( y < bounding_box_[2] )
        return false;
    if( y > bounding_box_[3] )
        return false;

    if( z < bounding_box_[4] )
        return false;
    if( z > bounding_box_[5] )
        return false;

    return true;
}

void OctomapToEllipsoids::getGmmPointsFromOctomap( std::vector<gmm::Vector>& points, gmm::BBox& box )
{
    // Get points into gmm structure
    points.clear();
    unsigned int nb_leaf_occipied = 0;
    double minX, minY, minZ, maxX, maxY, maxZ;

    {
        boost::mutex::scoped_lock lock( octomap_mutex_ );

        if( octomap_ == NULL )
            return;

        // get properties of octree
        std::size_t octree_depth = octomap_->getTreeDepth();
        octomap_->getMetricMin( minX, minY, minZ );
        octomap_->getMetricMax( maxX, maxY, maxZ );

        // 14 for sparser tree
        for( octomap::OcTree::iterator it = octomap_->begin( 16 ), end = octomap_->end(); it != end; ++it)
            // for( octomap::OcTree::leaf_iterator it = octomap->begin_leafs(), end=octomap->end_leafs(); it!= end; ++it)
        {
            if( octomap_->isNodeOccupied(*it) )
            {
                gmm::Vector pos(3);
                pos[0] = it.getX();
                pos[1] = it.getY();
                pos[2] = it.getZ();

                if( isInBoundingBox( pos[0], pos[1], pos[2] ) )
                    points.push_back( pos );

                nb_leaf_occipied ++;

                //            if( it.getSize() > 0.02 )
                //            {
                //                cout << "size : " << it.getSize() << endl;
                //                double cell_probability = it->getOccupancy();
                //                cout << "cell_probability : " << cell_probability << " , id : "  << nb_leaf_occipied << endl;
                //            }
            }
        }
    }

    box.xmin = minX, box.ymin = minY, box.zmin = minZ;
    box.xmax = maxX, box.ymax = maxY, box.zmax = maxZ;

    cout << "nb of points : " << nb_leaf_occipied << endl;
}

void OctomapToEllipsoids::getGmmPointsFromPCL( std::vector<gmm::Vector>& points, gmm::BBox& box )
{
    points.resize( cloud_->points.size() );

    box.xmin = std::numeric_limits<double>::max();
    box.xmax = std::numeric_limits<double>::min();
    box.ymin = std::numeric_limits<double>::max();
    box.ymax = std::numeric_limits<double>::min();
    box.zmin = std::numeric_limits<double>::max();
    box.zmax = std::numeric_limits<double>::min();

    for( int i=0; i<cloud_->points.size(); i++ )
    {
        gmm::Vector pos(3);
        pos[0] = cloud_->points[i].x;
        pos[1] = cloud_->points[i].y;
        pos[2] = cloud_->points[i].z;
        points[i] = pos;

        if( pos[0] < box.xmin ) box.xmin = pos[0];
        if( pos[0] > box.xmax ) box.xmax = pos[0];
        if( pos[1] < box.ymin ) box.ymin = pos[1];
        if( pos[1] > box.ymax ) box.ymax = pos[1];
        if( pos[2] < box.zmin ) box.zmin = pos[2];
        if( pos[2] > box.zmax ) box.zmax = pos[2];

        // cout << "point " << i << " is " << cloud_->points[i] << endl;
    }

    messages_received_++;
}

void OctomapToEllipsoids::getPointCloud()
{
    visualization_msgs::Marker marker = display_->pointcloudMarker( 0, *cloud_ );
    viz_pub_.publish( marker );

    // Get points from octomap
    std::vector<gmm::Vector> points;
    gmm::BBox bounding_box;
    getGmmPointsFromPCL( points, bounding_box );

    // compute GMMs
    computeGMMs( points, bounding_box );
}

void OctomapToEllipsoids::computeGMMs( const std::vector<gmm::Vector>& points, const gmm::BBox& box )
{
    gmm::Matrix dataset( points.size(), 3 );

    for( unsigned int i=0; i<points.size(); i++ )
    {
        dataset.SetRow( points[i], i );
        Eigen::Vector3d p;
        p[0] = points[i][0];
        p[1] = points[i][1];
        p[2] = points[i][2];
        // cout << "points[" << i << "] : " << p.transpose() << endl;
    }

    gmm::GaussianMixture g;

    // g.initEM_3DBox( 3, dataset, box ); // initialize the model, // Number of states in the GMM
    g.initEM_Kmeans( nb_ellipsoids_, dataset );
    g.doEM( dataset, 20 ); // performs EM

    {
        boost::mutex::scoped_lock lock( gmm_mutex_ );

        mean_.resize( g.nState );
        for(int s=0; s<g.nState; s++ )
            for(int i=0; i<g.dim; i++ )
                mean_[s](i) = g.mu(s,i);

        for(int s=0; s<g.nState; s++ )
            cout << "mean[" << s << "] : " << mean_[s].transpose() << endl;

        covariance_.resize( g.nState );
        for( int s=0; s<g.nState; s++ )
            for( int j=0;j<g.dim; j++)
                for( int i=0; i<g.dim; i++ )
                    covariance_[s](i,j) = g.sigma[s](i,j);
    }
}

void OctomapToEllipsoids::getOctomap( octomap_msgs::Octomap::ConstPtr msg )
{
    ROS_INFO( "Received OctomapBinary at t : %f", msg->header.stamp.toSec() );
    ROS_INFO( "Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

    ++messages_received_;

    if( !msg->data.empty() )
    {
        boost::mutex::scoped_lock lock( octomap_mutex_ );


        // creating octree
        if( octomap_ ) {
            delete octomap_;
            octomap_ = NULL;
        }

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if( tree ){
            octomap_ = dynamic_cast<octomap::OcTree*>(tree);
        }
    }

    if( !octomap_ )
    {
        ROS_ERROR( "Failed to create octree structure" );
        return;
    }
}

void OctomapToEllipsoids::getMoveitOctomap( moveit_msgs::PlanningScene::ConstPtr msg )
{
    ROS_INFO( "Received Moveit Octomap at t : %f", msg->world.octomap.header.stamp.toSec() );
    ROS_INFO( "Received Moveit Octomap at (size: %d bytes)", (int)msg->world.octomap.octomap.data.size());

    ++messages_received_;

    if( !msg->world.octomap.octomap.data.empty() )
    {
        boost::mutex::scoped_lock lock( octomap_mutex_ );

        // Get Frame
        frame_id_ = msg->world.octomap.header.frame_id;

        cout << "Octomap frame id is : " << frame_id_ << endl;

        // creating octree
        if( octomap_ ) {
            delete octomap_;
            octomap_ = NULL;
        }

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap( msg->world.octomap.octomap );
        if( tree ){
            octomap_ = dynamic_cast<octomap::OcTree*>(tree);
        }

        new_octomap_ = true;
    }

    if( !octomap_ )
    {
        ROS_ERROR( "Failed to create octree structure" );
        return;
    }
}

bool OctomapToEllipsoids::startNode(int argc, char **argv)
{
    argc = 0;
    argv = NULL;

    ros::init( argc, argv, "octomap_to_ellipsoids" /*, ros::init_options::NoSigintHandler*/ );
    nh_ = new ros::NodeHandle();

    ros::NodeHandle nhp("~");
    nhp.param(std::string("use_stored_point_cloud"), use_stored_pointcloud_, bool(true));
    nhp.param(std::string("use_moveit_octomap"), use_moveit_octomap_, bool(false));
    nhp.param(std::string("publish_as_markers"), publish_as_markers_, bool(true));
    nhp.param(std::string("octomap_topic"), octomap_topic_, std::string("/octomap_full"));
    nhp.param(std::string("nb_ellipsoids"), nb_ellipsoids_, int(10));

    // Get bounding box
    std::string bounding_box;
    nhp.param(std::string("ellipsoid_bounding_box"), bounding_box, std::string("-1 1 -1 1 -1 1"));

    std::stringstream linestream( bounding_box );
    for(unsigned i=0; i < bounding_box_.size(); i++)
    {
        std::string item;
        std::getline( linestream, item, ' ');
        std::istringstream ss( item );
        ss >> bounding_box_[i];
        cout << "bounding box : " <<  bounding_box_[i] << endl;
    }

    // nhp.param(std::string("arm_config_topic"), arm_config_topic, std::string("/l_arm_controller/state"));
    // nhp.param(std::string("arm_command_action"), arm_command_action, std::string("/l_arm_controller/joint_trajectory_action"));

    ROS_INFO( "publish_as_markers_ : %d", publish_as_markers_ );
    ROS_INFO( "octomap_topic : %s", octomap_topic_.c_str() );

    // Create display object
    display_ = boost::shared_ptr<EllipsoidsToMarkers>(new EllipsoidsToMarkers());

    if( use_stored_pointcloud_ )
    {
        loadCloudFromFile();
    }
    else if ( use_moveit_octomap_ )
    {
        cout << "subscribe to moveit" << endl;
        // Moveit Octomap subscriber
        sub_octomap_ = nh_->subscribe<moveit_msgs::PlanningScene>( octomap_topic_, 1, boost::bind( &OctomapToEllipsoids::getMoveitOctomap, this, _1 ) );
    }
    else
    {
        // Octomap subscriber
        sub_octomap_ = nh_->subscribe<octomap_msgs::Octomap>( octomap_topic_, 1, boost::bind( &OctomapToEllipsoids::getOctomap, this, _1 ) );
    }

    // Publisher for markers
    viz_pub_ = nh_->advertise<visualization_msgs::Marker>("ellipsoid_markers", 1, true);

    // Publisher for ellipsoids
    ellipsoid_pub_ = nh_->advertise<ellipsoid_msgs::EllipsoidArray>("ellipsoid_publisher", 1, true);

    // Spin node
    ros::Rate spin_rate(spin_rate_);
    while( ros::ok() )
    {
        if( use_stored_pointcloud_ )

            getPointCloud();
        else
            getEllipsoids();

        // Set ellipsoids to markers
        if( publish_as_markers_ )
            ellipsoidsToMarkers();

        // Publish Ellipsoids
        publishEllipsoids();

        // Process callbacks
        ros::spinOnce();
        // Spin
        spin_rate.sleep();
    }

    return true;
}

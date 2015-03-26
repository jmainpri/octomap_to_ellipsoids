#include "ellipsoids_markers.hpp"

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::cout;
using std::endl;

EllipsoidsToMarkers::EllipsoidsToMarkers()
{
    frame_id_ = "/camera_link";
}

EllipsoidsToMarkers::~EllipsoidsToMarkers()
{

}

// used to paint the autovectors
visualization_msgs::Marker EllipsoidsToMarkers::pointcloudMarker(int id, const pcl::PointCloud<pcl::PointXYZ>& cloud )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    // Populate the options
    marker.ns = "point_cloud";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    marker.scale.x = .02;
    marker.scale.y = .02;
    marker.scale.z = .02;

    marker.points.resize( cloud.points.size() );
    marker.colors.resize( cloud.points.size() );

    double alpha = 1.0;

    // Add all cells in collision
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        // Set point
        marker.points[i].x = cloud.points[i].x;
        marker.points[i].y = cloud.points[i].y;
        marker.points[i].z = cloud.points[i].z;
        // Color it
        marker.colors[i].a = alpha;
        marker.colors[i].b = 0.0;
        marker.colors[i].g = 0.0;
        marker.colors[i].r = 1.0;
    }
    return marker;
}

// used to paint the autovectors
visualization_msgs::Marker EllipsoidsToMarkers::vectorMarker(int id, const Eigen::Vector3d& vector, const Eigen::Vector3d& position  )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoids_axis";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x =0.01;
    marker.scale.y =0.03;
    marker.scale.z =0.01;

    marker.color.a= 1.0;
    marker.color.r = id % 3 == 0 ? 1. : 0.;
    marker.color.g = id % 3 == 1 ? 1. : 0.;
    marker.color.b = id % 3 == 2 ? 1. : 0.;

    geometry_msgs::Point start, end;
    start.x = position[0];
    start.y = position[1];
    start.z = position[2];
    end.x = start.x + vector[0];
    end.y = start.y + vector[1];
    end.z = start.z + vector[2];

    marker.points.push_back(start);
    marker.points.push_back(end);
//    print str(marker);
    return marker;
}

//used to paint the autovectors
visualization_msgs::Marker EllipsoidsToMarkers::ellipsoidMarker( int& id, const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance, std::vector<visualization_msgs::Marker>& eigen_vectors )
{
    // painting the gaussian ellipsoid marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoids";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = mean[0];
    marker.pose.position.y = mean[1];
    marker.pose.position.z = mean[2];

    const double std_factor = 2.0;

    // getting the distribution eigen vectors and values
    Eigen::EigenSolver<Eigen::MatrixXd> es( covariance );
    Eigen::VectorXcd eig_val = es.eigenvalues();
    Eigen::MatrixXcd V = es.eigenvectors();

    // building the rotation matrix
    Eigen::Vector3d eig_x = V.col(0).real().normalized();
    Eigen::Vector3d eig_y = V.col(1).real().normalized();
    Eigen::Vector3d eig_z = eig_x.cross( eig_y );

    // painting the eigen vectors

    visualization_msgs::Marker marker2;
    Eigen::Vector3d v_eig;

    v_eig = std::sqrt( eig_val[0].real() ) * std_factor * eig_x;
    marker2 = vectorMarker( 3*id + 0, v_eig, mean );
    eigen_vectors.push_back( marker2 );

    v_eig = std::sqrt( eig_val[1].real() ) * std_factor * eig_y;
    marker2 = vectorMarker( 3*id + 1, v_eig, mean );
    eigen_vectors.push_back( marker2 );

    v_eig = std::sqrt( eig_val[2].real() ) * std_factor * eig_z;
    marker2 = vectorMarker( 3*id + 2, v_eig, mean );
    eigen_vectors.push_back( marker2 );

    /**
    Eigen::Matrix3d m;
    m.col(0) = eig_x;
    m.col(1) = eig_y;
    m.col(2) = eig_z;
    Eigen::Quaterniond q( m );
    **/

    tf::Matrix3x3 m( eig_x[0], eig_y[0], eig_z[0],
                     eig_x[1], eig_y[1], eig_z[1],
                     eig_x[2], eig_y[2], eig_z[2] );

    // cout << " V.real() * V.real().transpose() : " <<  endl << V.real() * V.real().transpose() << endl;

    // tfScalar yaw, pitch, roll;
    // m.getEulerYPR( yaw, pitch, roll );

    tf::Quaternion q;
    // q.setRPY( roll, pitch, yaw );
    m.getRotation( q );

    tf::Transform transform;
    transform.setOrigin( tf::Vector3( mean[0], mean[1], mean[2] ) );
    transform.setRotation( q );

    std::ostringstream ostr; //output string stream
    ostr << id; //use the string stream just like cout,
    br_.sendTransform( tf::StampedTransform(transform, ros::Time::now(), frame_id_, "ellipsoid_" + ostr.str() ) );

    // painting the Gaussian Ellipsoid Marker
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 2. * std::sqrt( eig_val[0].real() ) * std_factor;
    marker.scale.y = 2. * std::sqrt( eig_val[1].real() ) * std_factor;
    marker.scale.z = 2. * std::sqrt( eig_val[2].real() ) * std_factor;

    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.3;

    return marker;
}

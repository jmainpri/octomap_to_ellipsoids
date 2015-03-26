#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include "kmeans.h"

using std::cout;
using std::endl;

double
randf(double m)
{
    return m * rand() / (RAND_MAX - 1.);
}

inline double
dist2(const point_t& a, const point_t& b)
{
    return ( a.x - b.x ).squaredNorm();
}

inline int
nearest(const point_t& pt, point cent, int n_cluster, double& min_d)
{
    int min_i = pt.group;
    min_d = HUGE_VAL;

    for( int i = 0; i < n_cluster; i++ ) {
        double d = dist2( cent[i], pt );
        if (min_d > d) {
            min_d = d;
            min_i = i;
        }
    }

    return min_i;
}

void kpp(point pts, int len, point cent, int n_cent)
{
    int i, j;
    int n_cluster;
    double sum;
    double *d = new double[len];

    cent[0] = pts[ rand() % len ];
    for (n_cluster = 1; n_cluster < n_cent; n_cluster++)
    {
        sum = 0;
        for( j = 0; j < len; j++ ) {
            nearest( pts[j], cent, n_cluster, d[j] ) ;
            sum += d[j];
        }
        sum = randf(sum);
        for( j = 0; j < len; j++ ) {
            sum -= d[j];
            if ( sum > 0 ) continue;
            cent[n_cluster] = pts[j];
            break;
        }
    }
    for (j = 0; j < len; j++){
        double dist = 0.;
        pts[j].group = nearest( pts[j], cent, n_cluster, dist );
    }

    delete[] d;
}

point lloyd(point pts, int len, int n_cluster)
{
    cout << "start loyd" << endl;

    int i, j, min_i;
    int changed;
    int dim = pts[0].x.size();

    point cent = new point_t[n_cluster];

    /* assign init grouping randomly */
//    for( j=0; j<len; j++ )
//    {
//        pts[j].group = j % n_cluster;
//        if( std::isnan(pts[j].group) ){
//            cout << "group is NAN" << endl;
//        }
//    }

    /* or call k++ init */
    kpp( pts, len, cent, n_cluster );

    cout << "start loop" << endl;

    do {
        /* group element for centroids are used as counters */
        for( i=0; i<n_cluster; i++ ) {
            cent[i].group = 0;
            cent[i].x = Eigen::VectorXd::Zero(dim);
        }

        for( j=0; j<len; j++) {
            int id = pts[j].group;
            cent[id].group++;
            cent[id].x += pts[j].x;
        }
        for( i=0; i<n_cluster; i++ )
        {
            if( cent[i].group > 0 )
                cent[i].x /= double( cent[i].group );
            else
                cent[i].x = Eigen::VectorXd::Zero(dim);
        }

        changed = 0;

        /* find closest centroid of each point */
        for( j = 0; j < len; j++ ) {
            double dist;
            min_i = nearest( pts[j], cent, n_cluster, dist);
            if( min_i != pts[j].group ) {
                changed++;
                pts[j].group = min_i;
            }
        }

        // cout << "changed : " << changed << endl;

    } while ( double(changed) > ( double(len) * 0.001 )); /* stop when 99.9% of points are good */

    for( i = 0; i < n_cluster; i++ )
    {
        if( cent[i].group > 0 ){
            cent[i].group = i;
        }
        else{
             cent[i].group = -1;
        }
    }

    return cent;
}

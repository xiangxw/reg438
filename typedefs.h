#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal NormalT;
typedef pcl::FPFHSignature33 FeatureT;
typedef Eigen::Matrix4f Matrix;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef pcl::PointCloud<NormalT>::Ptr NormalCloudPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr NormalCloudConstPtr;
typedef pcl::PointCloud<FeatureT> FeatureCloud;
typedef pcl::PointCloud<FeatureT>::Ptr FeatureCloudPtr;
typedef pcl::PointCloud<FeatureT>::ConstPtr FeatureCloudConstPtr;

#endif // end TYPEDEFS_H

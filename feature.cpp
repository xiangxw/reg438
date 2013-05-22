#include <pcl/features/integral_image_normal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include "feature.h"

NormalCloudPtr compute_normals(const PointCloudConstPtr &cloud, float radius)
{
	pcl::IntegralImageNormalEstimation<PointT, NormalT> ne;
	NormalCloudPtr cloud_normal(new NormalCloud);
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*cloud_normal);

	return cloud_normal;
}

PointCloudPtr compute_keypoints(const PointCloudPtr &cloud,
		float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{
	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;

	sift_detect.setSearchMethod(pcl::search::Search<PointT>::Ptr(new pcl::search::OrganizedNeighbor<PointT>));
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	sift_detect.setInputCloud(cloud);
	pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
	sift_detect.compute(keypoints_temp);
	PointCloudPtr keypoints(new PointCloud);
	pcl::copyPointCloud(keypoints_temp, *keypoints);

	return keypoints;
}

FeatureCloudPtr compute_feature_descriptors(const PointCloudConstPtr &cloud,
		const NormalCloudConstPtr &normals, const PointCloudConstPtr &keypoints,
		float radius)
{
	pcl::FPFHEstimation<PointT, NormalT, FeatureT> fpfh;
	pcl::search::Search<PointT>::Ptr search(new pcl::search::OrganizedNeighbor<PointT>);
	FeatureCloudPtr fpfhs(new FeatureCloud);

	fpfh.setSearchMethod(search);
	fpfh.setRadiusSearch(radius);
	fpfh.setSearchSurface(cloud);
	fpfh.setInputNormals(normals);
	fpfh.setInputCloud(keypoints);
	fpfh.compute(*fpfhs);

	return fpfhs;
}

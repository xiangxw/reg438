#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include "align.h"

Matrix initial_align(const PointCloudConstPtr &keypoints_source, const FeatureCloudConstPtr &descriptors_source,
		const PointCloudConstPtr &keypoints_target, const FeatureCloudConstPtr &descriptors_target,
		float min_sample_distance, double max_correspondence_distance, int nr_iterations)
{
	pcl::SampleConsensusInitialAlignment<PointT, PointT, FeatureT> sac;
	PointCloud cloud;

	sac.setMinSampleDistance(min_sample_distance);
	sac.setMaxCorrespondenceDistance(max_correspondence_distance);
	sac.setMaximumIterations(nr_iterations);

	sac.setInputCloud(keypoints_source);
	sac.setSourceFeatures(descriptors_source);
	sac.setInputTarget(keypoints_target);
	sac.setTargetFeatures(descriptors_target);
	sac.align(cloud);
	return sac.getFinalTransformation();
}

Matrix refine_align (const PointCloudConstPtr &source_points, const PointCloudConstPtr &target_points,
		const Matrix &initial_alignment, double max_correspondence_distance,
		double outlier_rejection_threshold, double transformation_epsilon, int max_iterations)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloudPtr source_points_transformed (new PointCloud);
	PointCloud registration_output;

	icp.setMaxCorrespondenceDistance (max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
	icp.setTransformationEpsilon (transformation_epsilon);
	icp.setMaximumIterations (max_iterations);

	pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

	icp.setInputCloud (source_points_transformed);
	icp.setInputTarget (target_points);

	icp.align (registration_output);

	return (icp.getFinalTransformation () * initial_alignment);
}

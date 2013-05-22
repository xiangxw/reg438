#ifndef ALIGN_H
#define ALIGN_H

#include "typedefs.h"

Matrix initial_align(const PointCloudConstPtr &keypoints_source, const FeatureCloudConstPtr &descriptors_source,
		const PointCloudConstPtr &keypoints_target, const FeatureCloudConstPtr &descriptors_target,
		float min_sample_distance, double max_correspondence_distance, int nr_iterations);
Matrix refine_align (const PointCloudConstPtr &source_points, const PointCloudConstPtr &target_points,
		const Matrix &initial_alignment, double max_correspondence_distance,
		double outlier_rejection_threshold, double transformation_epsilon, int max_iterations);

#endif // end ALIGN_H

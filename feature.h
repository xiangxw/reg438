#ifndef FEATURE_H
#define FEATURE_H

#include "typedefs.h"

NormalCloudPtr compute_normals(const PointCloudConstPtr &cloud, float radius);
PointCloudPtr compute_keypoints(const PointCloudPtr &cloud,
		float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);
FeatureCloudPtr compute_feature_descriptors(const PointCloudConstPtr &cloud,
		const NormalCloudConstPtr &normals, const PointCloudConstPtr &keypoints,
		float radius);

#endif // end FEATURE_H

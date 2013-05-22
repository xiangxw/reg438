#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include "feature.h"
#include "align.h"

struct Arg {
	// -n
	float normal_radius;

	// -k
	float min_scale;
	int nr_octaves;
	int nr_scales_per_octave;
	float min_contrast;

	// -f
	float feature_radius;

	// -i
	float min_sample_distance;
	double max_correspondence_distance;
	int nr_iterations;

	// -r
	double max_correspondence_distance_icp;
	double outlier_rejection_threshold;
	double transformation_epsilon;
	int max_iterations;

	// --no-downsampling
	bool downsampling;
	float downsampling_leafx;
	float downsampling_leafy;
	float downsampling_leafz;

	// --no-refine
	bool refine_align;

	// --show-normals
	bool show_normals;

	// --show-keypoints
	bool show_keypoints;

	// input files
	std::vector<std::string> files;
};

void parse_cmd(int argc, char **argv, Arg *arg);
void help();

int main (int argc, char **argv)
{
	const char *result = "result.pcd";
	Arg arg;
	std::vector<int> index;

	PointCloudPtr cloud_source(new PointCloud);
	NormalCloudPtr normals_source(new NormalCloud);
	PointCloudPtr keypoints_source(new PointCloud);
	FeatureCloudPtr features_source(new FeatureCloud);

	PointCloudPtr cloud_target(new PointCloud);
	NormalCloudPtr normals_target(new NormalCloud);
	PointCloudPtr keypoints_target(new PointCloud);
	FeatureCloudPtr features_target(new FeatureCloud);

	PointCloudPtr cloud_registered(new PointCloud);
	PointCloudPtr cloud_result(new PointCloud);
	Matrix global_transform = Matrix::Identity(), pair_transform;

	// parse command
	parse_cmd(argc, argv, &arg);

	// load first cloud as target
	std::stringstream filename;
	filename << "data/" << arg.files[0] << ".pcd";
	pcl::io::loadPCDFile(filename.str(), *cloud_target);
	pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, index);
	*cloud_result = *cloud_target;

	// register all clouds
	for (std::vector<std::string>::const_iterator i = arg.files.cbegin() + 1;
			i != arg.files.cend(); ++i) {
		// load pcd file
		std::stringstream filename;
		filename << "data/" << *i << ".pcd";
		if (!boost::filesystem::exists(filename.str())) {
			pcl::console::print_warn("input file %s not exists\n", filename.str().c_str());
			continue;
		}
		pcl::io::loadPCDFile(filename.str(), *cloud_source);
		pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, index);

		pcl::console::print_info("---------- start cloud %s ----------\n", filename.str().c_str());

		// source normals
		pcl::console::print_info("start compute source normals, normal radius: %f\n", arg.normal_radius);
		normals_source = compute_normals(cloud_source, arg.normal_radius);
		// source keypoints
		pcl::console::print_info("start compute source keypoints, args: %f, %d, %d, %f\n",
				arg.min_scale, arg.nr_octaves, arg.nr_scales_per_octave, arg.min_contrast);
		keypoints_source = compute_keypoints(cloud_source,
				arg.min_scale, arg.nr_octaves, arg.nr_scales_per_octave, arg.min_contrast);
		// source descriptors
		pcl::console::print_info("start compute source features, feature radius: %f\n", arg.feature_radius);
		features_source = compute_feature_descriptors(cloud_source,
				normals_source, keypoints_source, arg.feature_radius);

		// target normals
		pcl::console::print_info("start compute target normals, normal radius: %f\n", arg.normal_radius);
		normals_target = compute_normals(cloud_target, arg.normal_radius);
		// target keypoints
		pcl::console::print_info("start compute target keypoints, args: %f, %d, %d, %f\n",
				arg.min_scale, arg.nr_octaves, arg.nr_scales_per_octave, arg.min_contrast);
		keypoints_target = compute_keypoints(cloud_target,
				arg.min_scale, arg.nr_octaves, arg.nr_scales_per_octave, arg.min_contrast);
		// target descriptors
		pcl::console::print_info("start compute target features, feature radius: %f\n", arg.feature_radius);
		features_target = compute_feature_descriptors(cloud_target,
				normals_target, keypoints_target, arg.feature_radius);

		// visualize
		if (arg.show_normals || arg.show_keypoints) {
			pcl::visualization::PCLVisualizer viewer_source("Source Cloud");
			pcl::visualization::PCLVisualizer viewer_target("Target Cloud");
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> hander_source(cloud_source);
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> hander_target(cloud_target);

			// visualize source
			viewer_source.addPointCloud(cloud_source, hander_source, "cloud_source");
			viewer_source.resetCameraViewpoint("cloud_source");
			if (arg.show_normals) {
				viewer_source.addPointCloudNormals<PointT, NormalT>(
						cloud_source, normals_source, 100, 0.02, "normals_source");
			}
			if (arg.show_keypoints) {
				pcl::visualization::PointCloudColorHandlerCustom<PointT> red(keypoints_source, 255, 0, 0);
				viewer_source.addPointCloud(keypoints_source, red, "keypoints_source");
				viewer_source.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints_source");
			}

			// visualize target
			viewer_target.addPointCloud(cloud_target, hander_target, "cloud_target");
			viewer_target.resetCameraViewpoint("cloud_target");
			if (arg.show_normals) {
				viewer_target.addPointCloudNormals<PointT, NormalT>(
						cloud_target, normals_target, 100, 0.02, "normals_target");
			}
			if (arg.show_keypoints) {
				pcl::visualization::PointCloudColorHandlerCustom<PointT> red(keypoints_target, 255, 0, 0);
				viewer_target.addPointCloud(keypoints_target, red, "keypoints_target");
				viewer_target.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints_target");
			}

			while (!viewer_source.wasStopped() && !viewer_target.wasStopped()) {
				viewer_source.spin();
				viewer_target.spin();
			}
			continue;
		}

		// initial align
		pcl::console::print_info("start initial align, args: %f, %f, %d\n",
			arg.min_sample_distance, arg.max_correspondence_distance, arg.nr_iterations);
		pair_transform = initial_align(keypoints_source, features_source, keypoints_target, features_target,
				arg.min_sample_distance, arg.max_correspondence_distance, arg.nr_iterations);

		// refine align
		if (arg.refine_align) {
			pcl::console::print_info("start refine align, args: %f, %f, %f, %d\n",
				arg.max_correspondence_distance_icp, arg.outlier_rejection_threshold,
				arg.transformation_epsilon, arg.max_iterations);
			pair_transform = refine_align(cloud_source, cloud_target,
					pair_transform, arg.max_correspondence_distance_icp,
					arg.outlier_rejection_threshold, arg.transformation_epsilon, arg.max_iterations);
		} else {
			pcl::console::print_info("no refine alignment\n");
		}

		// tranform
		global_transform = pair_transform * global_transform;
		pcl::transformPointCloud(*cloud_source, *cloud_registered, global_transform);
		*cloud_result += *cloud_registered;
		cloud_target = cloud_source;

		// downsampling
		if (arg.downsampling) {
			pcl::console::print_info("start downsampling...\n");
			pcl::VoxelGrid<PointT> grid;
			grid.setInputCloud(cloud_result);
			grid.setLeafSize(0.001f, 0.001f, 0.001f);
			grid.filter(*cloud_result);
		} else {
			pcl::console::print_info("no downsampling\n");
		}

		pcl::console::print_info("---------- end cloud %s ----------\n", filename.str().c_str());
	}

	// save result to file
	pcl::io::savePCDFileBinary(result, *cloud_result);
	pcl::console::print_info("Press any key to continue...\n");
	std::cin.ignore();

	return 0;
}

void parse_cmd(int argc, char **argv, Arg *arg)
{
	std::string params_string;
	std::vector<std::string> tokens;
	bool detect_normals;
	bool detect_keypoints;
	bool detect_features;
	bool detect_initial_align;
	bool detect_refine_align;
	bool detect_downsampling;
	bool detect_input_files;

	if (argc < 2) {
		pcl::console::print_error("command input error!\n");
		help();
		exit(1);
	}
	
	// normal radius
	detect_normals = (pcl::console::parse_argument(argc, argv, "-n", arg->normal_radius) > 0);
	if (!detect_normals) {
		arg->normal_radius = 0.025f;
	}

	// keypoints
	detect_keypoints = (pcl::console::parse_argument(argc, argv, "-k", params_string) > 0);
	if (detect_keypoints) {
		boost::split(tokens, params_string, boost::is_any_of(","), boost::token_compress_on);
		if (tokens.size() != 4) {
			pcl::console::print_error("command input error: invalid keypoints arguments!\n");
			help();
			exit(1);
		}
		arg->min_scale = (float)atof(tokens[0].c_str());
		arg->nr_octaves = atoi(tokens[1].c_str());
		arg->nr_scales_per_octave = atoi(tokens[2].c_str());
		arg->min_contrast = (float)atof(tokens[3].c_str());
	} else {
		arg->min_scale = 0.005f;
		arg->nr_octaves = 10;
		arg->nr_scales_per_octave = 8;
		arg->min_contrast = 1.5f;
	}

	// features
	detect_features = (pcl::console::parse_argument(argc, argv, "-f", arg->feature_radius) > 0);
	if (!detect_features) {
		arg->feature_radius = 0.05f;
	}

	// initial align
	detect_initial_align = (pcl::console::parse_argument(argc, argv, "-i", params_string) > 0);
	if (detect_initial_align) {
		boost::split(tokens, params_string, boost::is_any_of(","), boost::token_compress_on);
		if (tokens.size() != 3) {
			pcl::console::print_error("command input error: invalid initial align arguments!\n");
			help();
			exit(1);
		}
		arg->min_sample_distance = (float)atof(tokens[0].c_str());
		arg->max_correspondence_distance = atof(tokens[1].c_str());
		arg->nr_iterations = atoi(tokens[2].c_str());
	} else {
		arg->min_sample_distance = 0.025f;
		arg->max_correspondence_distance = 0.01;
		arg->nr_iterations = 500;
	}

	// refine align
	detect_refine_align = (pcl::console::parse_argument(argc, argv, "-r", params_string) > 0);
	if (detect_refine_align) {
		boost::split(tokens, params_string, boost::is_any_of(","), boost::token_compress_on);
		if (tokens.size() != 4) {
			pcl::console::print_error("command input error: invalid refine align arguments!\n");
			help();
			exit(1);
		}
		arg->max_correspondence_distance_icp = atof (tokens[0].c_str ());
		arg->outlier_rejection_threshold = atof (tokens[1].c_str ());
		arg->transformation_epsilon = atof (tokens[2].c_str ());
		arg->max_iterations = atoi (tokens[3].c_str ());
	} else {
		arg->max_correspondence_distance_icp = 0.05;
		arg->outlier_rejection_threshold = 0.05;
		arg->transformation_epsilon = 0.0;
		arg->max_iterations = 100;
	}

	// downsampling
	arg->downsampling = (pcl::console::find_argument(argc, argv, "--no-downsampling") < 0);
	detect_downsampling = (pcl::console::parse_argument(argc, argv, "-d", params_string) > 0);
	if (detect_downsampling) {
		boost::split(tokens, params_string, boost::is_any_of(","), boost::token_compress_on);
		if (tokens.size() != 3) {
			pcl::console::print_error("command input error: invalid downsampling arguments!\n");
			help();
			exit(1);
		}
		arg->downsampling_leafx = (float)atof(tokens[0].c_str());
		arg->downsampling_leafy = (float)atof(tokens[1].c_str());
		arg->downsampling_leafz = (float)atof(tokens[2].c_str());
	} else {
		arg->downsampling_leafx = 0.001f;
		arg->downsampling_leafy = 0.001f;
		arg->downsampling_leafz = 0.001f;
	}

	// refine align
	arg->refine_align = (pcl::console::find_argument(argc, argv, "--no-refine") < 0);

	// show normals
	arg->show_normals = (pcl::console::find_argument(argc, argv, "--show-normals") > 0);

	// show keypoints
	arg->show_keypoints = (pcl::console::find_argument(argc, argv, "--show-keypoints") > 0);

	// input files
	boost::split(tokens, argv[1], boost::is_any_of(","), boost::token_compress_on);
	if (tokens.size() >= 2) {
		for (std::vector<std::string>::const_iterator i = tokens.cbegin();
				i != tokens.cend(); ++i) {
			arg->files.push_back(*i);
		}
		detect_input_files = true;
	} else {
		boost::split(tokens, argv[1], boost::is_any_of("-"), boost::token_compress_on);
		if (tokens.size() == 2) {
			int start, end;
			start = atoi(tokens[0].c_str());
			end = atoi(tokens[1].c_str());
			if (end > start) {
				for (int i = start; i <= end; ++i) {
					std::stringstream stream;
					stream << i;
					arg->files.push_back(stream.str());
				}
				detect_input_files = true;
			} else {
				detect_input_files = false;
			}
		} else {
			detect_input_files = false;
		}
	}
	if (!detect_input_files) {
		pcl::console::print_error("command input error: invalid input pcd files!\n");
		help();
		exit(1);
	}
}

void help()
{
	pcl::console::print_info("Usage: register_with_feature_all files [options]\n");
	pcl::console::print_info("start ................................. start index of input pcd file\n");
	pcl::console::print_info("end ................................... end index of input pcd file\n");
	pcl::console::print_info("[options]:\n");
	pcl::console::print_info("	-n normal_radius .................... Normal Radius\n");
	pcl::console::print_info("	-k min_scale,nr_octaves,nr_scales_per_octave,min_contrast ......... Compute Keypoints\n");
	pcl::console::print_info("	-f feature_radius ................... Feature Descriptors Radius\n");
	pcl::console::print_info("	-i min_sample_distance,max_correspondence_distance,nr_iterations .. SampleConsensusInitialAlignment argument\n");
	pcl::console::print_info("	-r max_correspondence_distance,outlier_rejection_threshold,transformation_epsilon,max_iterations .. ICP refine align argument\n");
	pcl::console::print_info("	-d leafx,leafy,leafz ................ Downsampling leaf size\n");
	pcl::console::print_info("	--no-downsampling ................... No downsampling\n");
	pcl::console::print_info("	--no-refine ......................... No refine alignment");
	pcl::console::print_info("	--show-normals ...................... Show normals");
	pcl::console::print_info("	--show-keypoints .................... Show keypoints");
}

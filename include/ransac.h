#ifndef POINT_CLOUD_PLANE_DETECTION_RANSAC_H
#define POINT_CLOUD_PLANE_DETECTION_RANSAC_H

#include <opencv2/opencv.hpp>

bool total_least_squares_plane_estimate(cv::Vec4f &model, const cv::Mat &input, const int *sample, int sample_num);

int get_inliers(bool *inliers, const cv::Vec4f &model, const cv::Mat &pts, float thr, int best_inls = 0);

bool VoxelGrid(cv::Mat &sampling_pts, cv::Mat &pts, float length, float width, float height);

void get_planes(cv::Mat &labels, std::vector<cv::Vec4f> &planes, cv::InputArray &points3d,
                float thr, int max_iterations, int desired_num_planes = 1, float grid_size = -1,
                cv::Vec3f *normal = nullptr, double normal_diff_thr = 0.06);

#endif //POINT_CLOUD_PLANE_DETECTION_RANSAC_H

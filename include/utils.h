
#ifndef POINT_CLOUD_PLANE_DETECTION_UTILS_H
#define POINT_CLOUD_PLANE_DETECTION_UTILS_H

#include <fstream>
#include <opencv2/opencv.hpp>

bool save_points_label(const std::string &file_path, cv::InputArray &labels, bool sync_io = false);

bool save_point_cloud_ply(const std::string &file_path, cv::InputArray &pts, bool sync_io = false);

std::string get_plane_expression_str(cv::Vec4f model);

bool read_point_cloud_ply_to_mat(cv::Mat &output, const std::string &file_path, bool sync_io = false);

void point_cloud_generator(float size, int point_num, int noise_num, std::vector<cv::Vec4f> models, cv::Mat &point_cloud);

#endif //POINT_CLOUD_PLANE_DETECTION_UTILS_H

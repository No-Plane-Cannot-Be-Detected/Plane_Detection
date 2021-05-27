#include <fstream>
#include<opencv2/opencv.hpp>
#include "ransac.h"
#include "utils.h"

#ifndef INFO
#define INFO 1
#endif

using namespace std;

/*
* command syntax
*/
void usage() {
    printf("Usage:  Point-Cloud-Plane-Detection desired_num_planes thr grid_size max_iters test_file_path normal\n"
           "\tdesired_num_planes\t\t Number of detected planes \n"
           "\tthr\t\t Distance threshold from point to plane\n"
           "\tgrid_size\t\t The size of the grid used for downsampling\n"
           "\tmax_iters\t\t Maximum iterations of RANSAC for each plane detection \n"
           "\ttest_file_path\t\t Path of test point cloud file \n"
           "\tnormal\t\t Normal vector constraint \n");
}

int main(int argc, char *argv[]) {

    if (argc < 8) {
        usage();
        return 1;
    }

    int desired_num_planes = stoi(argv[1]);
    float thr = stof(argv[2]);
    float grid_size = stof(argv[3]);
    int max_iters = stoi(argv[4]);
    string test_file_path = argv[5];
    float nor1 = stof(argv[6]), nor2 = stof(argv[7]), nor3 = stof(argv[8]);


    cv::Mat point_cloud;
    cv::Mat labels;
    std::vector<cv::Vec4f> planes;

#ifdef INFO
    clock_t start_read_data = clock();
    printf("Start reading point cloud data...\n");
#endif


    if (read_point_cloud_ply_to_mat(point_cloud, test_file_path)) {


#ifdef INFO
        printf("Successfully read point cloud data, point cloud size %d, time cost %f s\n",
               point_cloud.rows, ((float) (clock() - start_read_data)) / CLOCKS_PER_SEC);
#endif

        cv::Vec3f normal(nor1, nor2, nor3);
        cv::Vec3f *normal_ptr;
        if (nor1 == 0 && nor2 == 0 && nor3 == 0) {
            normal_ptr = nullptr;
        } else {
            normal_ptr = &normal;
        }
        get_planes(labels, planes, point_cloud, thr, max_iters, desired_num_planes, grid_size, normal_ptr);
        char label_path[256];
        sprintf(label_path, "%s-thr_%4f-iter_%d-grid_size_%4f-planes-%d-label.txt",
                test_file_path.c_str(), thr, max_iters, grid_size, desired_num_planes);
        save_points_label(label_path, labels);
#ifdef INFO
        printf("save labels Successful, path: %s\n", label_path);
#endif
    }
}

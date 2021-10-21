#include "utils.h"

/**
 * Get the plane equation string ax + by + cz + d = 0
 *
 * @param model [a, b, c, d]
 * @return  Plane equation string
 */
std::string get_plane_expression_str(cv::Vec4f model) {
    double hom1 = sqrt(model[0] * model[0] + model[1] * model[1] + model[2] * model[2] + model[3] * model[3]);
    //hom1 = 1;
    char buf[64];
    sprintf(buf, "%fx + %fy + %fz + %f = 0", model[0] / hom1,
            model[1] / hom1, model[1] / hom1, model[1] / hom1);
    return buf;
}

/**
 * Save point cloud label
 *
 * @param file_path  Save path
 * @param labels  Mat for storing label
 * @param sync_io  IO synchronization
 * @return  true or false
 */
bool save_points_label(const std::string &file_path, cv::InputArray &labels, bool sync_io) {
    cv::Mat labels_m = labels.getMat();

    int size = labels_m.rows;
    if (size == 0) {
        return false;
    }

    std::ios::sync_with_stdio(sync_io);
    std::ofstream ofs(file_path);

    if (!ofs.is_open()) {
        std::cerr << "ofstream open file error!\n";
        return false;
    }
    ofs.clear();

    const int *myptr = (int *) labels_m.data;

    for (int i = 0; i < size; ++i) {
        ofs << myptr[i] << "\n";
    }

    ofs.flush();
    ofs.close();
    std::ios::sync_with_stdio(true);
    return true;
}

/**
 * Save point cloud label
 *
 * @param file_path  Save path
 * @param pts   Point cloud
 * @param sync_io  IO synchronization
 * @return  true or false
 */
bool save_point_cloud_ply(const std::string &file_path, cv::InputArray &pts, bool sync_io) {
    cv::Mat pts_m = pts.getMat();
    int size = pts_m.rows;
    if (size == 0) {
        return false;
    }

    std::ios::sync_with_stdio(sync_io);
    std::ofstream ofs(file_path);

    if (!ofs.is_open()) {
        std::cerr << "ofstream open file error!\n";
        return false;
    }
    ofs.clear();

    std::string head = "ply\n"
                       "format ascii 1.0\n"
                       "comment Created by Where is my plane\n"
                       "element vertex " + std::to_string(size) + "\n"
                                                                  "property float x\n"
                                                                  "property float y\n"
                                                                  "property float z\n"
                                                                  "end_header\n";
    ofs << head;

    float *myptr = (float *) pts_m.data;

//    ofs << std::fixed << std::setprecision(2);

    for (int i = 0; i < size; ++i) {
        ofs << *(myptr++) << " " << *(myptr++) << " " << *(myptr++) << "\n";
    }

    ofs.flush();
    ofs.close();
    std::ios::sync_with_stdio(true);
    return true;
}

/**
 *  Get point cloud data
 *
 * @param output  Point cloud (output)
 * @param file_path  Save path
 * @param sync_io  IO synchronization
 * @return  true or false
 */
bool read_point_cloud_ply_to_mat(cv::Mat &output, const std::string &file_path, bool sync_io) {
    std::ios::sync_with_stdio(sync_io);

    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        std::cerr << "ifstream open file error!\n";
        return false;
    }

    std::string head;
    int size = 0;
    while (head != "end_header") {
        if (head == "vertex") {
            ifs >> size;
        }
        ifs >> head;
    }

    if (size < 0) {
        std::cerr << "File read exception\n";
        return false;
    }

    output = cv::Mat(size, 3, CV_32F);

    float *myptr = (float *) output.data;
    size *= 3;
    for (int i = 0; i < size; ++i) {
        ifs >> myptr[i];
    }
    std::ios::sync_with_stdio(true);
    return true;
}

/**
 * Model used to generate point cloud plane

 * @param size  Represents the range of the space cube, that is, the positive and negative range of the randomly generated plane
 * @param point_num  Indicates the number of generated points
 * @param noise_num  Indicates the number of additional noise points
 * @param models  Represents an array of all plane equations. Before passing in the parameters, you need to add a model equation of the form ax+by+cz+d=0 through models.push(Vec4f(a,b,c,d)) , The quantity does not need to be excessive
 * @param point_cloud  Means the generated point cloud
 */
void
point_cloud_generator(float size, int point_num, int noise_num, std::vector<cv::Vec4f> models, cv::Mat &point_cloud) {
    float *myptr = (float *) point_cloud.data;
    cv::RNG rng((int) std::time(nullptr));

    int ran_num;
    int res_num = point_num;
    int models_size = (int) models.size();
    int per_num = point_num / models_size;
    int idx = 0;
    cv::Vec4f model;
    float x, y, z;
    for (int i = 0; i < models_size; i++) {
        ran_num = per_num - rng.uniform(-per_num / 2, per_num / 2);
        if (i == models_size - 1) {
            ran_num = res_num;
        }
        res_num -= ran_num;
        model = models.at(i);
        for (int j = 0; j < ran_num; j++) {
            x = rng.uniform(-size, size);
            y = rng.uniform(-size, size);
            z = -(model[0] * x + model[1] * y + model[3]) / model[2];
            if (z > size || z < -size) {
                j--;
                continue;
            }
            myptr[idx++] = x;
            myptr[idx++] = y;
            myptr[idx++] = z;
        }
    }

    for (int i = 0; i < noise_num; i++) {
        myptr[idx++] = rng.uniform(-size, size);
        myptr[idx++] = rng.uniform(-size, size);
        myptr[idx++] = rng.uniform(-size, size);
    }
}

#include <iostream>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "ransac.h"

#ifndef INFO
#define INFO 1
#endif


int get_plane(cv::Vec4f &best_model, bool *inliers, const cv::Mat &pts, float thr, int max_iterations,
              cv::Vec3f *expect_normal, double normal_diff_thr);

bool check_same_plane(cv::Vec4f &p1, cv::Vec4f &p2, double thr);

inline bool check_same_normal(cv::Vec4f &actual_plane, cv::Vec3f &expect_normal, double thr);

/**
 * 获取多个平面
 *
 * @param labels  点属于某个平面的标签, n × 1 矩阵, n 等于输入点云的大小 (输出)
 * @param planes  保存平面方程的vector, 方程表示为 ax + by + cz + d = 0 (输出)
 * @param points3d  输入的点云数据
 * @param thr  阈值
 * @param max_iterations 最大迭代次数
 * @param desired_num_planes  目标平面的数量
 * @param grid_size  降采样方格大小, 如果 小于等于 0，代表不进行降采样
 * @param normal 法向量约束，为 nullptr 代表不使用约束， 否则检测的平面法向量满足该约束
 */
void get_planes(cv::Mat &labels, std::vector<cv::Vec4f> &planes, cv::InputArray &points3d,
                float thr, int max_iterations, int desired_num_planes, float grid_size, cv::Vec3f *normal
                , double normal_diff_thr) {
#ifdef INFO
    clock_t start, end, begin_time = clock();
    printf("Begin fit plane, parameter: desired_num_planes: %d, threshold: %f, max_iterations: %d, grid_size: %f\n",
           desired_num_planes, thr, max_iterations, grid_size);
#endif

    using namespace std;
    cv::Mat points3d_ = points3d.getMat();
    if (points3d.isVector()) {
        points3d_ = cv::Mat((int) points3d_.total(), 3, CV_32F, points3d_.data);
    } else {
        if (points3d_.channels() != 1)
            points3d_ = points3d_.reshape(1, (int) points3d_.total()); // 转换成单通道
        if (points3d_.rows < points3d_.cols)
            transpose(points3d_, points3d_);
        CV_CheckEQ(points3d_.cols, 3, "Invalid dimension of point");
        if (points3d_.type() != CV_32F)
            points3d_.convertTo(points3d_, CV_32F); // 使用 float 存储数据
    }


    std::vector<cv::Vec4f> planes_; // 初次找出的平面

    {
        cv::Mat pts3d_plane_fit; // 每次用于找平面的点云



        if (grid_size > 0) {
#ifdef INFO
            float duration;
            start = clock();
#endif

            VoxelGrid(pts3d_plane_fit, points3d_, grid_size, grid_size, grid_size);

#ifdef INFO
            end = clock();
            duration = ((float) (end - start)) / CLOCKS_PER_SEC;
            printf("Sampling is completed, origin point cloud size %d, after sampling %d, time cost %f s \n",
                   points3d_.rows, pts3d_plane_fit.rows, duration);
#endif

        } else {
#ifdef INFO
            printf("Skip down sampling...\n");
#endif
            pts3d_plane_fit = points3d_;
        }


        bool *inliers_ = new bool[pts3d_plane_fit.rows]; // 标记点是否为内点


#ifdef INFO
        printf("-----------------------------------------------------------------------------------------------\n");
        printf(" No. \t\t\t\t Plane \t\t\t\t\tinliers num \t time cost (s) \n");
#endif


        for (int num_planes = 1; num_planes <= desired_num_planes; ++num_planes) {
            cv::Vec4f model_;


#ifdef INFO
            start = clock();
#endif


            int inliers_num = get_plane(model_, inliers_, pts3d_plane_fit, thr, max_iterations, normal, normal_diff_thr);
            if (inliers_num == 0) break;


#ifdef INFO
            printf(" %d \t %fx + %fy + %fz + %f = 0\t\t %d \t\t %f \n", num_planes, model_[0], model_[1],
                   model_[2], model_[3], inliers_num, ((float) (clock() - start)) / CLOCKS_PER_SEC);
#endif


            planes_.emplace_back(model_);
            if (num_planes == desired_num_planes) break;

            const int pts3d_size = pts3d_plane_fit.rows;
            cv::Mat tmp = pts3d_plane_fit.clone();
            pts3d_plane_fit = cv::Mat(pts3d_size - inliers_num, 3, CV_32F);

            const float *tmp_ptr = (float *) tmp.data;
            float *fit_ptr = (float *) pts3d_plane_fit.data;

            for (int c = 0, p = 0; p < pts3d_size; ++p) {
                if (!inliers_[p]) {
                    // 如果不是已知平面的内点，加入下一轮迭代用于找新的平面
                    int i = 3 * c, j = 3 * p;
                    fit_ptr[i] = tmp_ptr[j];
                    fit_ptr[i + 1] = tmp_ptr[j + 1];
                    fit_ptr[i + 2] = tmp_ptr[j + 2];
                    ++c;
                }
            }
        }
        delete[] inliers_;
    }


#ifdef INFO
    printf("-----------------------------------------------------------------------------------------------\n");
    printf("Start optimizing the plane model\n");
    clock_t opt_time_start = clock();
    printf("-----------------------------------------------------------------------------------------------\n");
    printf(" No. \t\t\t\t Plane \t\t\t\t\tinliers num \t time cost (s) \n");
#endif


    //  根据得到的平面模型，在原点云数据上进行局部优化并打标记label
    int max_lo_inliers = 300, max_lo_iters = 3;
    int pts_size = points3d_.rows;
    labels = cv::Mat::zeros(pts_size, 1, CV_32S);

    // 保持点的索引数组与原始点相对应
    int *orig_pts_idx = new int[pts_size];
    for (int i = 0; i < pts_size; ++i) orig_pts_idx[i] = i;

    bool *inliers = new bool[pts_size];
    cv::Vec4f lo_model, best_model;

    // 存储平面内点数量，下标从 1 开始 按照降序排列
    vector<int> plane_inls_num = {0};

    int *labels_ptr = (int *) labels.data;
    int *inlier_sample = new int[max_lo_inliers];

    int planes_cnt = (int) planes_.size();
    for (int plane_num = 1; plane_num <= planes_cnt; ++plane_num) {


#ifdef INFO
        start = clock();
#endif


        best_model = planes_[plane_num - 1];
        pts_size = points3d_.rows;
        std::vector<int> random_pool(pts_size);
        for (int p = 0; p < pts_size; ++p) random_pool[p] = p;

        int best_inls = get_inliers(inliers, best_model, points3d_, thr);
        int lo_inls = 0;
        for (int lo_iter = 0; lo_iter < max_lo_iters; ++lo_iter) {
            cv::randShuffle(random_pool);
            int sample_cnt = 0;
            for (int p : random_pool) {
                if (inliers[p]) {
                    inlier_sample[sample_cnt] = p;
                    ++sample_cnt;
                    if (sample_cnt >= max_lo_inliers) break;
                }
            }

            if (!total_least_squares_plane_estimate(lo_model, points3d_, inlier_sample, sample_cnt))
                continue;

            if (normal != nullptr)
            {
                if (!check_same_normal(lo_model, *normal, normal_diff_thr))
                    continue;
            }

            lo_inls = get_inliers(inliers, lo_model, points3d_, thr, best_inls);
            if (best_inls < lo_inls) {
                best_model = lo_model;
                best_inls = lo_inls;
            } else if (best_inls == lo_inls) {
                break;
            }
        }

        if (best_inls >= lo_inls) best_inls = get_inliers(inliers, best_model, points3d_, thr);

        int e = 0;
        while (best_inls < plane_inls_num[e]) ++e;
        plane_inls_num.insert(plane_inls_num.begin() + e, best_inls);

        planes.insert(planes.begin() + e, best_model);


#ifdef INFO
        printf(" %d \t %fx + %fy + %fz + %f = 0 \t\t %d \t\t %f \n", plane_num, best_model[0], best_model[1],
               best_model[2], best_model[3], best_inls, ((float) (clock() - start)) / CLOCKS_PER_SEC);
#endif


        cv::Mat tmp = points3d_;
        const int pts3d_size = tmp.rows;
        if (plane_num == planes_cnt) {
            for (int c = 0, p = 0; p < pts3d_size; ++p) {
                if (inliers[p])
                    labels_ptr[orig_pts_idx[p]] = plane_num;
            }
            break;
        }

        points3d_ = cv::Mat(pts3d_size - best_inls, 3, CV_32F);

        const float *tmp_ptr = (float *) tmp.data;
        float *pts3d_ptr_ = (float *) points3d_.data;
        for (int c = 0, p = 0; p < pts3d_size; ++p) {
            if (!inliers[p]) {
                // 如果点不在找到的平面内, 将其添加到下一次运行
                orig_pts_idx[c] = orig_pts_idx[p];
                int i = 3 * c, j = 3 * p;
                pts3d_ptr_[i] = tmp_ptr[j];
                pts3d_ptr_[i + 1] = tmp_ptr[j + 1];
                pts3d_ptr_[i + 2] = tmp_ptr[j + 2];
                ++c;
            } else {
                labels_ptr[orig_pts_idx[p]] = plane_num; // 否则标记此点
            }
        }
    }


#ifdef INFO
    printf("-----------------------------------------------------------------------------------------------\n");
    printf("Optimization time cost: %f s\n", ((float) (clock() - opt_time_start)) / CLOCKS_PER_SEC);
    printf("Total time of plane fitting: %f s\n", ((float) (clock() - begin_time)) / CLOCKS_PER_SEC);
#endif


    delete[] orig_pts_idx;
    delete[] inliers;
    delete[] inlier_sample;
}

/**
 * 体素滤波降采样
 *
 * @param sampling_pts  采样后的点云 (输出)
 * @param pts 原始点云
 * @param length 方格长度
 * @param width  方格宽度
 * @param height  方格高度
 * @return
 */
bool VoxelGrid(cv::Mat &sampling_pts, cv::Mat &pts, float length, float width, float height) {
    const int size = pts.rows;
    using namespace std;
    float *myptr = (float *) pts.data;
    float x_min, x_max, y_min, y_max, z_min, z_max;
    x_max = x_min = myptr[0];
    y_max = y_min = myptr[1];
    z_max = z_min = myptr[2];

    float x, y, z;
    for (int i = 1; i < size; ++i) {
        int ii = 3 * i;
        x = myptr[ii];
        y = myptr[ii + 1];
        z = myptr[ii + 2];

        if (x_min > x) x_min = x;
        if (x_max < x) x_max = x;

        if (y_min > y) y_min = y;
        if (y_max < y) y_max = y;

        if (z_min > z) z_min = z;
        if (z_max < z) z_max = z;
    }   // find the minimum and maximum of xyz

    unordered_map<string, vector<int>> grids;

    int init_size = size * 0.02;
    grids.reserve(init_size);

    char buff[64];
    for (int i = 0; i < size; ++i) {
        int ii = 3 * i;
        int hx = (int) ((myptr[ii] - x_min) / length);
        int hy = (int) ((myptr[ii + 1] - y_min) / width);
        int hz = (int) ((myptr[ii + 2] - z_min) / height);
        sprintf(buff, "%d,%d,%d", hx, hy, hz);
        //string str = string(buff);
        //string str = to_string(hx) + ',' + to_string(hy) + ',' + to_string(hz);
        if (grids[buff].empty())
            grids[buff] = {i};
        else
            grids[buff].push_back(i);
    }

    sampling_pts = cv::Mat((int) grids.size(), 3, CV_32F);


    float *sampling_ptr = sampling_pts.ptr<float>();

    int label_id = 0;
    for (auto &grid : grids) {
        int cluster_size = (int) grid.second.size();
        float sumx = 0, sumy = 0, sumz = 0;
        float **block = new float *[cluster_size];
        for (int j = 0; j < cluster_size; ++j) {
            block[j] = new float[3];
            float *pts_ptr = pts.ptr<float>(grid.second[j]);
            block[j][0] = *pts_ptr;
            ++pts_ptr;
            block[j][1] = *pts_ptr;
            ++pts_ptr;
            block[j][2] = *pts_ptr;
            sumx += block[j][0];
            sumy += block[j][1];
            sumz += block[j][2];
        }
        float x_center = sumx / cluster_size, y_center = sumy / cluster_size, z_center = sumz / cluster_size;
        float x_sample = block[0][0], y_sample = block[0][1], z_sample = block[0][2];
        float min_dist = (x_sample - x_center) * (x_sample - x_center) +
                         (y_sample - y_center) * (y_sample - y_center) +
                         (z_sample - z_center) * (z_sample - z_center);
        for (int j = 1; j < cluster_size; ++j) {
            float tmp_dist = (block[j][0] - x_center) * (block[j][0] - x_center) +
                             (block[j][1] - y_center) * (block[j][1] - y_center) +
                             (block[j][2] - z_center) * (block[j][2] - z_center);
            if (tmp_dist < min_dist) {
                min_dist = tmp_dist;
                x_sample = block[j][0];
                y_sample = block[j][1];
                z_sample = block[j][2];
            }
        }

        for (int _ = 0; _ < cluster_size; ++_) {
            delete[] block[_];
        }
        delete[]block;

        *sampling_ptr = x_sample;
        ++sampling_ptr;
        *sampling_ptr = y_sample;
        ++sampling_ptr;
        *sampling_ptr = z_sample;
        ++sampling_ptr;


        ++label_id;
    }

    return true;
}


/**
 * 选取部分点 拟合一个平面
 *
 * @param model 拟合的平面模型结果 （输出）  ax + by + cz + d = 0
 *
 * @param input 输入的点云
 * @param sample 用于拟合平面的点 在 input 中的数据下标，前 sample_num 个有效
 * @param sample_num 用于拟合平面的点的个数
 * @return 拟合结果是否有效
 */
bool total_least_squares_plane_estimate(cv::Vec4f &model, const cv::Mat &input, const int *sample, int sample_num) {
    const float *pts_ptr = (float *) input.data;

    // 判断三点共线
    if (3 == sample_num) {
        int id1 = 3 * sample[0], id2 = 3 * sample[1], id3 = 3 * sample[2];
        float x1 = pts_ptr[id1], y1 = pts_ptr[id1 + 1], z1 = pts_ptr[id1 + 2];
        float x2 = pts_ptr[id2], y2 = pts_ptr[id2 + 1], z2 = pts_ptr[id2 + 2];
        float x3 = pts_ptr[id3], y3 = pts_ptr[id3 + 1], z3 = pts_ptr[id3 + 2];
        cv::Vec3f ba(x1 - x2, y1 - y2, z1 - z2);
        cv::Vec3f ca(x1 - x3, y1 - y3, z1 - z3);
        float ba_dot_ca = fabs(ca.dot(ba));
        if (fabs(ba_dot_ca * ba_dot_ca - ba.dot(ba) * ca.dot(ca)) < 0.0001) {
            return false;
        }
    }
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < sample_num; ++i) {
        int ii = 3 * sample[i];
        sum_x += pts_ptr[ii];
        sum_y += pts_ptr[ii + 1];
        sum_z += pts_ptr[ii + 2];
    }

    const float mean_x = sum_x / sample_num, mean_y = sum_y / sample_num, mean_z = sum_z / sample_num;

    cv::Mat pd_mat;
    {
        cv::Mat U(sample_num, 3, CV_32F);
        cv::Mat UT(3, sample_num, CV_32F);
        float *U_ptr = (float *) U.data;
        float *UT_ptr = (float *) UT.data;
        int d_size = sample_num + sample_num;
        for (int i = 0; i < sample_num; ++i) {
            int ii = 3 * sample[i], j = 3 * i;
            UT_ptr[i] = U_ptr[j] = pts_ptr[ii] - mean_x;
            UT_ptr[i + sample_num] = U_ptr[j + 1] = pts_ptr[ii + 1] - mean_y;
            UT_ptr[i + d_size] = U_ptr[j + 2] = pts_ptr[ii + 2] - mean_z;
        }

        pd_mat = UT * U;
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvectors(3, 3, CV_32F);
    cv::eigen(pd_mat, eigenvalues, eigenvectors);
    const float *eig_ptr = (float *) eigenvectors.data;

    float a = eig_ptr[6], b = eig_ptr[7], c = eig_ptr[8];
    if (std::isinf(a) || std::isinf(b) || std::isinf(c) || (a == 0 && b == 0 && c == 0)) {
        std::cerr << "tls estimate plane fail." << std::endl;
        return false;
    }

    model = cv::Vec4f(a, b, c, -a * mean_x - b * mean_y - c * mean_z);
    return true;
}


/**
 * 获取平面内点
 *
 * @param inliers 标记是否为输入平面的内点 （输出）
 *
 * @param model 平面模型
 * @param pts 点云
 * @param thr 阈值，点到平面的距离小于阈值则认为该点属于该平面
 * @param best_inls 最好的模型的内点数量，如果内点数量没有机会大于该值，则终止计算
 * @return 内点数量
 */
int get_inliers(bool *inliers, const cv::Vec4f &model, const cv::Mat &pts, float thr, int best_inls) {
    const int pts_size = pts.rows;
    const float *pts_ptr = (float *) pts.data;
    float a = model(0), b = model(1), c = model(2), d = model(3), hom = sqrt(a * a + b * b + c * c);
    a = a / hom, b = b / hom, c = c / hom, d = d / hom;

    int num_inliers = 0;

    std::fill(inliers, inliers + pts_size, false);
    // 根据统计预估前 2/3 的点计算是必须的，无法剪枝
    int cut = pts_size * 2 / 3;
    for (int p = 0; p < cut; ++p) {
        int pp = 3 * p;
        if (fabs(a * pts_ptr[pp] + b * pts_ptr[pp + 1] + c * pts_ptr[pp + 2] + d) < thr) {
            inliers[p] = true;
            ++num_inliers;
        }
    }
    // prune
    for (int p = cut; p < pts_size; ++p) {
        int pp = 3 * p;
        if (fabs(a * pts_ptr[pp] + b * pts_ptr[pp + 1] + c * pts_ptr[pp + 2] + d) < thr) {
            inliers[p] = true;
            ++num_inliers;
        }
        // 如果未计算的点全都是内点，该模型都无法比最好的模型好，那么终止计算
        if (num_inliers + pts_size - p < best_inls) break;
    }
    return num_inliers;
}


/**
 * 获得一个平面
 *
 * @param best_model 最好的平面模型（输出）
 * @param inliers 标记是否为平面模型的内点 （输出）
 *
 * @param pts 点云
 * @param thr 阈值
 * @param max_iterations 最大迭代次数
 * @return 内点数量
 */
int
get_plane(cv::Vec4f &best_model, bool *inliers, const cv::Mat &pts, float thr,
          int max_iterations, cv::Vec3f *normal, double normal_diff_thr) {
    using namespace std;
    const int pts_size = pts.rows, min_sample_size = 3, max_lo_inliers = 20, max_lo_iters = 10;
    if (pts_size < 3) return 0;

    cv::Vec4f model, lo_model;
    std::vector<int> random_pool(pts_size);
    for (int p = 0; p < pts_size; ++p) random_pool[p] = p;

    cv::RNG rng;
    int *min_sample = new int[min_sample_size];
    int *inlier_sample = new int[max_lo_inliers];
    int best_inls = 0, num_inliers = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 从点云中随机选取一些点 拟合平面
        for (int i = 0; i < min_sample_size; ++i) min_sample[i] = rng.uniform(0, pts_size);

        if (!total_least_squares_plane_estimate(model, pts, min_sample, min_sample_size)) continue;

        if(normal != nullptr){
            if(!check_same_normal(model, *normal, normal_diff_thr)) continue;
        }

        num_inliers = get_inliers(inliers, model, pts, thr, best_inls);

        if (num_inliers > best_inls) {

            // 保存至今为止最好的模型
            best_model = model;
            best_inls = num_inliers;

            // 进行局部优化 Local Optimization
            for (int lo_iter = 0; lo_iter < max_lo_iters; ++lo_iter) {
                cv::randShuffle(random_pool);

                // 从内点中随机选取一些点 拟合平面
                int sample_cnt = 0;
                for (int p : random_pool) {
                    if (inliers[p]) {
                        inlier_sample[sample_cnt] = p;
                        ++sample_cnt;
                        if (sample_cnt >= max_lo_inliers) break;
                    }
                }

                if (!total_least_squares_plane_estimate(lo_model, pts, inlier_sample, sample_cnt))
                    continue;

                if (normal != nullptr) {
                    if (!check_same_normal(lo_model, *normal, normal_diff_thr)) continue;
                }

                num_inliers = get_inliers(inliers, lo_model, pts, thr, best_inls);

                if (best_inls < num_inliers) {
                    best_model = lo_model;
                    best_inls = num_inliers;
                } else if (best_inls == num_inliers) {
                    break;
                }
            }

            const double max_hyp = 3 * log(1 - 0.95) / log(1 - pow(float(best_inls) / pts_size, min_sample_size));
//            printf("best_inls = %d  best_inls/pts_size = %f   max_hyp = %f\n", best_inls, 1.0f * best_inls / pts_size,
//                   max_hyp);
            if (!std::isinf(max_hyp) && max_hyp < max_iterations) {
                max_iterations = static_cast<int>(max_hyp);
//                printf("update max_iterations --> %d \n", max_iterations);
            }
        }
    }

    delete[] min_sample;
    delete[] inlier_sample;
    // 更新 best_model 的内点信息 inliers
    if (best_inls != 0 && best_inls >= num_inliers) best_inls = get_inliers(inliers, best_model, pts, thr);
    return best_inls;
}

/**
 * 检查两个平面是否为同一个平面
 *
 * @param p1 平面1
 * @param p2 平面2
 * @return  两个平面很接近则返回 true，否则 false
 */
bool check_same_plane(cv::Vec4f &p1, cv::Vec4f &p2, double thr) {
    double hom1 = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2] + p1[3] * p1[3]);
    double hom2 = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2] + p2[3] * p2[3]);
    double p1a = p1[0] / hom1, p1b = p1[1] / hom1, p1c = p1[2] / hom1, p1d = p1[3] / hom1;
    double p2a = p2[0] / hom2, p2b = p2[1] / hom2, p2c = p2[2] / hom2, p2d = p2[3] / hom2;
    return (p1a - p2a) * (p1a - p2a) + (p1b - p2b) * (p1b - p2b) + (p1c - p2c) * (p1c - p2c) + (p1d - p2d) * (p1d - p2d)
           < thr; // 0.0000001
}

/**
 *
 * @param actual_plane
 * @param expect_normal
 * @param thr
 * @return
 */
bool check_same_normal(cv::Vec4f &actual_plane, cv::Vec3f &expect_normal, double thr) {
    double dot = (actual_plane[0] * expect_normal[0] + actual_plane[1] * expect_normal[1] +
                  actual_plane[2] * expect_normal[2]);
    double sqr_modulu_a = (actual_plane[0] * actual_plane[0] + actual_plane[1] * actual_plane[1] +
                         actual_plane[2] * actual_plane[2]);
    double sqr_modulu_b = (expect_normal[0] * expect_normal[0] + expect_normal[1] * expect_normal[1] +
                         expect_normal[2] * expect_normal[2]);
    //double cos_sqr = dot * dot / (sqr_mold_a * sqr_mold_b);
    //return (cos_sqr - 1) * (cos_sqr - 1) < thr;
    double cos_sqr = dot * dot;
    double sqr_modulu_ab = (sqr_modulu_a * sqr_modulu_b);
    return (cos_sqr - sqr_modulu_ab) * (cos_sqr - sqr_modulu_ab) <= thr * sqr_modulu_ab;
}

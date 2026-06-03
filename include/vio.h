/* 
视觉头文件
*/

#pragma once

#include "utils/types.h"
#include "visual_point.h"
#include "voxel_map.h"
#include "feature.h"
#include "opencv2/imgproc.hpp"
#include "pcl/filter/voxel_grid.h"
#include <cstddef>
#include <set>
#include <unordered_map>
#include <vikit/math_utils.h>
#include <vikit/robust_utils.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

//局部稀疏地图（它保存的是当前帧视觉匹配需要用到的一小部分地图点）视觉更新过程中临时使用的稀疏视觉子地图容器,每一帧 VIO/LIVO 视觉匹配时，从已有地图里挑选出来的一批视觉点和相关中间结果。
struct SubSparseMap
{
    //这个一般保存 传播位姿下的误差。
    vector<float> propa_errors;
    
    //保存当前视觉匹配的误差。
    vector<float> errors;

    //它保存的是 经过投影变换后的图像 patch。
    vector<vector<float>> wrap_patch;

    //保存每个视觉点在图像金字塔中的搜索层级。
    vector<int> search_levels;

    //它保存当前子地图中的视觉点指针。
    vector<VisualPoint *> voxel_points;

    //保存每个视觉点参考图像的逆曝光时间。
    vector<double> inv_expo_list;

    //这个保存从 LiDAR voxel map 中新增出来的点。
    vector<pointWithVar> add_from_voxel_map;


    //给稀疏地图预分配
    SubSparseMap()
    {
        propa_errors.reserve(SIZE_LARGE);
        errors.reserve(SIZE_LARGE);
        wrap_patch.reserve(SIZE_LARGE);
        search_levels.reserve(SIZE_LARGE);
        voxel_points.reserve(SIZE_LARGE);
        inv_expo_list.reserve(SIZE_LARGE);
        add_from_voxel_map.reserve(SIZE_LARGE);
    };  

    //清空子图的临时数据
    void reset()
    {
        propa_errors.clear();
        errors.clear();
        wrap_patch.clear();
        search_levels.clear();
        voxel_points.clear();
        inv_expo_list.clear();
        add_from_voxel_map.clear();
    }
}

class warp
{
    public:
       //当前帧 patch 到参考帧 patch 的二维仿射变换矩阵。
       Matrix2d A_cur_ref;
       //金字塔搜索层级
       int search_level;

       //构造wrap
       warp(int level, Matrix2d wrap_matrix) : search_level(level) , A_cur_ref(wrap_matrix) {};
       ~wrap(){};
}

//这个类像是一个容器，用来保存某个 voxel 里面的视觉地图点。
class VOXEL_POINTS
{
    public:

      //保存视觉点指针
       std::vector<VisualPoint *> voxel_points;

       //视觉点数量
       int count;

       //构造
       VOXEL_POINTS(int num) : count(num) {};
       ~VOXEL_POINTS()
       {
        //当 VOXEL_POINTS 对象销毁时，把里面保存的所有 VisualPoint* 都释放掉。
        for (VisualPoint* vp : voxel_points)
        {
            if(vp != nullptr)
            {
                delete vp;
                vp = nullptr;
            }
        }
       }
}

//视觉前端 / 视觉更新管理器，负责相机模型、图像尺寸、相机-雷达-IMU 外参、视觉点子地图、patch 匹配、NCC、曝光估计等视觉相关处理。
class VIOManager
{
  public:

    //图像网格大小。视觉前端常把图像划分成很多网格，用来均匀选点，避免特征点集中在某个区域。
    int grid_size;

    //是通用相机模型指针，负责投影、反投影、判断像素是否在图像内
    vk::AbstractCamera *cam;

    //针孔相机模型指针。如果当前相机是针孔模型，就可以用它访问更具体的内参和方法
    vk::pinhole_camera *pinhole_cam;

    //指向当前优化后的系统状态
    StatesGroup *state; 

    //指向 IMU 传播得到的预测状态
    StatesGroup *state_propagat;

    //外参和坐标变换矩阵，旋转矩阵和平移向量
    M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
    V3D pli, pci, pcl, pcw;

    //相机内参
    double fx, fy, cx, cy;

    //记录每个图像网格里已有多少点，用来保持点分布均匀
    vector<int> grid_num;

    //可能记录每个网格对应的地图点索引
    vector<int> map_index;

    //标记某个点是否靠近图像边界。patch 匹配需要点周围一小块图像，如果太靠边，patch 会越界。
    vector<int> border_flag;

    //用于标记某个点是否已经更新、是否参与当前帧视觉更新
    vector<int>update_flag;

    //保存地图点到相机的距离，常用于深度排序、近远点筛选
    vector<float> map_dist;

    //保存当前点的光度/扫描评分值，用于筛选点或者计算误差
    vector<float> scan_value;

    //patch 临时缓存，保存图像小块灰度值
    vector<float> patch_buffer;

    //功能开关
    bool noraml_en, inverse_composition_en, exposure_estiamte, raycast_en, has_ref_patch_cache;

    //是否使用 NCC 归一化互相关做 patch 匹配
    bool ncc_en = false, colmap_output_en;

    //是图像宽高，图像网格数量，总网格数量
    int width, height, grid_n_width, grid_n_height, length;

    //图像缩放比例
    double image_resize_factor;


    //使用第几层图像金字塔做 patch。patch 边长，patch 总像素数，patch 半径，边界安全距离
    int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, wrap_len;

    //视觉优化或 patch 对齐的最大迭代次数，前处理的视觉点数量
    int max_iterations, total_points;

    //图像点测量协方差，可以理解为像素观测噪声，外点阈值，用来剔除误差太大的视觉匹配点，NCC 阈值
    double img_point_cov, outlier_threshold, ncc_thre;

    //创建稀疏子地图visual_submap 存当前帧可能可见的视觉地图点、patch 误差、搜索层级、曝光信息等。
    SubSparseMap *visual_submap;

    //这个保存沿相机射线采样得到的三维点。
    std::vector<std::vector<V3D>> rays_with_sample_points;

    //计算雅可比耗时和ekf更新耗时
    double compute_jacobian_time, update_ekf_time;

    //平均耗时
    double ave_total = 0;
    
    //帧数和绘图标识
    int frame_count = 0;
    bool plot_flag;

    //ekf中间矩阵和增益
    Matrix<double,DIM_STATE, DIM_STATE> G ,H_T_H;
    MatrixXd K,H_sub_inv;

    //输出相机数据和colmap
    ofstream fout_camera, fout_colmap;

    //地图哈希表相关,feat_map 管视觉地图点在哪里，sub_feat_map 管当前帧哪些 voxel 已经被取过，warp_map 管每个点的 patch 如何从参考帧变换到当前帧。

    //  voxel 索引 -> 这个 voxel 里的视觉点集合，也就是用一个 voxel 的坐标索引 VOXEL_LOCATION，快速找到这个 voxel 里保存的 VisualPoint。
    unordered_map<VOXEL_LOCATION ,VOXEL_POINTS *> feat_map;

   //  voxel 索引 -> 当前子地图中的索引/标记，用于当前帧构建局部视觉子地图时，防止重复加入同一个 voxel，或者记录某个 voxel 在子地图中的编号。
    unordered_map<VOXEL_LOCATION, int>sub_feat_map;

    //  点或 patch 的索引 -> 对应的 Warp 信息. 保存视觉点对应的 patch warp 变换和搜索层级，用于 patch 匹配。
    unordered_map<int, warp *> warp_map;

    //从视觉 voxel map 中检索出来的视觉点
    vector<VisualPoint *> retrieve_voxel_points;

    //准备从 LiDAR voxel map 添加到视觉地图中的点
    vector<pointWithVar> append_voxel_points;

    //当前新图像帧指针。
    FramePtr new_frame_;

    //图像副本，用于处理或画图，RGB/BGR 彩色图像，可能用于给点云染色，调试图像，用于显示 patch、投影点、匹配结果
    cv::Mat img_cp, img_rgb, img_test; 


    //cell 是地图空间中的一个小区域，CellType 用来标记这个区域是已有地图区域、当前点云区域，还是未知区域。这样程序就可以快速判断这个区域该用于匹配、建图，还是跳过
    enum CellType
    {
        TYPE_MAP = 1,     //表示这个 cell 来自地图
        TYPR_POINTCLOUD,   //表示这个 cell 来自当前点云
        TYPE_UNKNOW     //表示未知类型
    };

    VIOManager();
    ~VIOManager();

    //    使用 inverse compositional 逆组合方法更新状态
    void updateStateInverse(cv::Mat img, int level);
    //    普通正向组合方式更新视觉状态
    void updateState(cv::Mat img, int level);
    
    //  当前图像帧处理入口   当前图像， 带协方差点，  体素地图，  图像时间
    void processFrame(cv::Mat &img, Vector<pointWithVar>&pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map. double img_time);

    //从已有地图中检索当前图像可能看到的稀疏视觉点。plane_map 是体素平面地图。函数会根据当前相机位姿，把地图点投影到图像里，判断是否在视野内，然后加入当前帧的 visual_submap。
    void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg,  const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

    //根据当前图像和 LiDAR 点生成新的 VisualPoint。
    void generateVisualMapPoints(cv:mat img, vector<pointWithVar> &pg);

    //IMU 到 LiDAR 外参
    void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);

    //LiDAR 到 Camera 外参
    void setLidarToCameraExtrinsic(vector<double> &R, vector<double> ;P);

    //初始化视觉模块
    void initializeVIO();

    //作用是从图像中提取一个 patch 小块。
    void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);

    //计算相机投影模型的雅可比
    void computeProjectionJacobian(V3D p, MD(2,3) &J);

    //计算雅可比并 EKF 更新
    void computeJacobianAndUpdateEKF(cv::Mat img);

    //网格重置
    void resetGrid();

    //作用是更新已有 VisualPoint 的状态。
    void updateVisualMapPoints(cv::Mat img);

    //计算参考帧 patch 到当前帧 patch 的二维仿射变换矩阵
    void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_refm const SE3 &T_cur_ref,
                            const int level_red;
                            const int patch_pyrimid_level, const int halfpatch_size, Matrix2d &A_cur_ref );

    //根据 A_cur_ref，从参考图像中采样出 warp 后的 patch
    void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level, 
                    const int pyramid_level, const int halfpatch_size, float *patch);
    
    //把一个新的 VisualPoint 插入视觉点体素哈希地图。
    void insertPointIntoVoxelMap(VisualPoint *pt_new);

    //画出当前跟踪到的视觉点
    void plotTrackedPoint();

    //根据当前系统状态更新 new_frame_ 的位姿
    void updateFrameState(statesGroup state);

    //将参考 patch 根据当前位姿投影到当前帧图像。
    void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

    //为视觉点选择或更新参考 patch  
    void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

    //作用是提前计算参考 patch，提高后续匹配速度。
    void precomputeReferencePatches(int level);

    //作用是把当前视觉地图、相机位姿、3D 点、图像观测输出成 COLMAP 可读格式。
    void dumpDataForColmap();

    //两个 patch 的归一化互相关
    double calculateNCC(float *ref_patch_, float *cur_patch, int patch_size);

    //warp 矩阵判断应该在哪一层图像金字塔上搜索。
    int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);

    //作用是在非整数像素位置 pc 处取图像颜色/灰度。
    V3F getInterpolatedPixel(cv::Mat img, V2D pc);

}

typedef std::shared_ptr<VIOManager> VIOManagerPtr;
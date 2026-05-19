#pragma once

#include "common_lib.h"
#include "utils/types.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>   
#include <unistd.h>
#include <unordered_map>
#include <visualzation_msgs/Marker.h>
#include <visualzation_msgs/MarkArray>

#include VOXELMAP_HASH_P 116101
#include VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

//体素地图配置
typedef struct VoxelMapConfig
{
    //最大体素尺寸
    double max_voxel_size_;

    //八叉树最大层
    int max_layer_;

    //最大迭代次数
    int max_interations_;

    //每层初始化所需要的点数，每层初始化点数不同是为了适应不同层数的体素大小，层数越大体素越小，所需点数也越少
    std::vector<int>layer_init_num_;

    //节点最多容纳的点数，超过这个数就继续分割
    int max_points_num;

    //平面判断的阈值
    double planner_threshold;


    //lidar测量误差参数，点云协方差计算需要
    //激光束方向
    double beam_err_;
    //深度方向
    double dept_err_;

    //
    double sogma_num;

    //是否发布平面地图
    bool is_pub_plane_map_;

    //地图滑动的位移阈值
    double sliding_thresh;

    //是否开启地图滑动
    bool map_sliding_en;
    
    //局部地图半尺寸
    int half_map_size;
} VoxelMapConfig;


//点到平面的残差数据载体，当前帧点找到对应平面后，计算点到平面的距离和残差，保存到这个结构体中
typedef  struct PointToPlane
{
    //机体坐标下点云
    Eigen::Vector3d point_b_;
    
    //世界坐标下点云
    Eigen::Vector3d point_w_;

    //平面法向量
    Eigen::Vector3d normal_;

    //平面中心
    Eigen::Vector3d center;

    //平面参数不确定性
    Eigen::Matrix<double, 6,6> plane_var_;

    //当前lidar点的不确定性协方差
    M3D body_cov_;

    //平面八叉树层数
    int layer_;

    //平面方程参数
    double d_;

    //平面相关参数
    double eigen_value;

    //残差是否有效
    bool is_valid;

    //点到平面的距离
    float dis_to_plane_;

}PointToPlane;


//体素地图中拟合出来的平面
typedef struct Voxelplane
{
    //平面中心，一般是点集的均值
    Eigen::Vector3d center_;

    //平面法向量
    Eigen::Vector3d normal_;

    //法向量y轴
    Eigen::Vector3d y_normal_;

    //法向量X轴
    Eigen::Vector3d x_normal_;

    //体素内点云分布的协方差
    Eigen::Matrix3d covariance;

    //平面的协方差
    Eigen::Matrix<double, 6, 6> plane_var_;

    //平面内点到平面中心的最大半径
    float radius_ = 0;

    //协方差矩阵的特征值，反映点云分布的形状，特征值越小越像平面
    float min_eigen_value_ = 1;
    float mid_eigen_value_ = 1;
    float max_engen_value_ = 1;

    //平面方程参数
    float d_ = 0;

    //拟合平面的点云数
    int points_size = 0;
    
    //是否判定为平面
    bool is_plane_ = false;

    //平面序号
    bool id_ = 0;

    //平面是否更新
    bool is_update_ = false;

    //体素平面初始化
    Voxelplane()
    {

        //体素内点云分布的协方差，判断一个点是不是在一个平面上，协方差越小越像平面
        plane_var_ = Eigen::Matrix<double, 6,6>::Zero();

        //平面协方差初始化为0
        covariance_ =Eigen::Matrix3d::Zero();

        //平面中心初始化为0
        center_ = Eigen::Vector3d::Zero();

        //法向量初始化为0
        normal_ = Eigen::Vector3d::Zero();
    }
} Voxelplane;


//地图地图的哈希索引，一个体素在地图的离散索引
class VOXEL_LOCATION
{
    public:

      //将体素空间离散成体素索引
      int64_t x,y,z; 

      //索引键值
      VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0 : x(vc) , y(vy) , z(vz)) {};

      //判断索引是否一致
      bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z )};
}

//降采样结构
struct DS_POINT
{
    ///xyz坐标
    float xyz[3];
    //强度
    float intensity;
    //积累原始多少个点
    int count = 0;
}

//计算机体坐标下点云的协方差矩阵，输入为点云在机体坐标下的坐标，激光束方向误差和深度误差，输出为点云的协方差矩阵
void calcBodyCov(const Eigen::Vector3d &pb, const float &range_inc , const float degree_inc, Eigen::Matrix3d &cov);


//八叉树节点
class VoxelOctoTree
{
    public:

    //八叉树节点构造函数
    VoxelOctoTree() = default;

    //当前八叉树节点暂存的点
    std::vector<pointWithVar> temp_points_;

    //当前八叉树节点拟合的平面
    VoxelPlane *plane_ptr_;

    //当前八叉树层级
    int layer_;


    //八叉树状态，0是终点，1是继续分割
    int octo_state_;

    //八叉树八个子节点指针
    VoxelOctotree *leaves_[8];

    //体素中心坐标
    double voxel_center_[3];

    //八叉树初始化层数
    std::vector<int> layer_init_num_;

    //体素边长的一半的一半
    float quater_length_;

    //平面阈值
    float planer_threshold;

    //当前层级所需的最少点数
    int points_size_threshold;

    //更新点大小阈值
    int update_size_threshold;

    //当前节点所需要最多的点数
    int max_points_num;

    //最大层数
    int max_layer_;

    //新增点数
    int new_points_num_ = 0;

    //是否初始化完成
    bool init_octo_;

    //是否更新
    bool update_enable_;

    VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
        : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num), planer_threshold_(planer_threshold),

    {
        //暂存点云清空
        temp_points_.clear();

        //八叉树状态初始化为0，初始化叶子节点
        octo_state_ = 0;

        //新增点数初始化为0
        new_points_ = 0;


        update_size_threshold_ = 5;


        init_octo_ = false;

        //更新标志初始化为true
        update_enable_ = true;

        //子结点指针初始化为nullptr
        for(int i = 0; i<8; i++)
        {
            leaves_[i] = nullptr;
        }

        //给当前节点分配一个平面指针
        plane_ptr_ = new VoxelPlane;
    }


    ~VoxelOctoTree()
    {
        for(int i = 0; i < 8; i++)
        {
            if(leaves_[i] != nullptr)
            {
                delete leaves_[i];
            }
        }
        plane_ptr_ = nullptr;
    }

    //用一批点初始化平面
    void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);

    //初始化八叉树
    void init_octo_tree();

    //切分八叉树
    void cut_octo_tree();

    //更新八叉树
    void updateOctoTree(const pointWithVat &pv);

    //世界坐标系点找到对应平面
    VoxelOctoTree *find_correspond(Eigen::Vector3d pw);

    //插入点云到八叉树中
    VoxelOctoTree *insert(const pointWithVar &pv)

}

//加载体素地图配置参数
void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

//地图管理器
class VoxelMapManager
{
    public:

    //地图管理器构造
    voxelMapManager() = default;

    //地图配置
    VoxelMapConfig config_setting_;
    
    //当前帧数
    int current_frame_id_ = 0;

    //发布体素地图
    ros::Publisher voxel_map_pub_;

    //体素地图数据结构，使用哈希表存储体素索引和对应的八叉树指针
    std::unordered_map<VOXEL_LOCATION, voxelOctoTree *> voxel_map_;

    //去畸变点云
    PointCloudXYZI::Ptr feats_undistort_;

    //降采样点云
    PointCloudXYZI::Ptr feats_down_body_;

    //世界坐标系下的降采样点云
    PointCloudXYZI::Ptr feats_down_world_;

    //外参
    M3D extR_;
    V3D extT_;

    //建立残差时间和EKF时间
    float build_residual_time, ekf_time;

    //平均残差时间和EKF时间
    float ave_build_residual_time = 0.0;
    float ave_ekf_time = 0;、

    //扫描线索引，当前点云帧的扫描线索引，主要用于点云去畸变和体素地图更新时判断点云顺序
    int scan_count = 0;

    //状态
    StatesGroup state_;

    //上一次位置
    V3D position_last_;

    //上一次滑动位置
    V3D last_slide_position = { 0,0,0};

    //ros四元素的姿态变量
    geometry_msgs::Quaternion geoQuat_;

    int feats_down_size_;
    int effect_feat_num_;

    //每个点的反对称矩阵，点云协方差在EKF里需要用到
    std::vector<M3D> cross_mat_list_;

    //每个点的协方差矩阵，点云协方差在EKF里需要用到
    std::vector<M3D> body_cov_list_;

    //带协方差的点云数据载体
    std::vector<pointWithVar> pv_list;

    //点到平面的残差列表
    std::vector<PointToPlane> ptpl_list;


    //根据 体素地图配置  和八叉树节点    初始化地图管理器
    VoxelMapManager(VoxelMapConfig &config_setting, std::undordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map) : config_setting_(config_setting), voxel_map_(voxel_map)
    {

        //当前帧数初始化为0
        current_frame_id_ = 0;

        feats_undistort_.reset(new PointCloudXYZI());
        feats_down_body_.reset(new PointCloudXYZI());
        feats_down_world_.reset(new PointCloudXYZI());
    }


    //状态估计
    void StateEstimation(StateGroup &state_propagat);

    //坐标系转换
    void TransfromLidar(const Eigen::Matrix3d &rot, const Eigen::Vector3d t , const PointCloudXYZI::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);


    //构建体素地图
    void buildVoxelMap();

    //根据输入点 input_point 所在的体素位置，生成一个 RGB 颜色，用于体素平面地图可视化。
    V3F RGBFromVovel(const V3D &input_point);

    //当前点插入体素地图并更新八叉树
    void UpdateVoxelMap(const std::vector<pointwithvar> &pv_list , std::vector<PointToPlane> &ptpl_list);
    
    //构建单个点到平面的残差
    void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob);


    //发布体素平面地图
    void pubVoxelMap();
    
    //局部地图滑动
    void mapSliding();

    //清除地图范围外的内存
    void clearMemOutOfMap(const int& x_max, const int& x_min, const int& y_max, const int& y_min, const int& z_max, const int& z_min);

    private:


    // 从八叉树里递归收集平面
    void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<voxelPlane> &plane_list);
    
    //把单个 VoxelPlane 转成 RViz Marker
    void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha);

    //根据平面局部坐标轴计算 Marker 姿态四元数
    void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

    //根据数值生成 RGB 颜色
    void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
    
}

typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;
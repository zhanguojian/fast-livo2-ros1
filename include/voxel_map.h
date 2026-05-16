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

    //每层初始化所需要的点数
    std::vector<int>layer_init_num_;

    //
    int max_points_num;

    //平面判断的阈值
    double planner_threshold;

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


//点到平面的残差数据载体
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

    //
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
    //平面中心
    Eigen::Vector3d center_;

    //平面法向量
    Eigen::Vector3d normal_;

    //法向量y轴
    Eigen::Vector3d y_normal_;

    //法向量X轴
    Eigen::Vector3d x_normal_;

    //平面协方差
    Eigen::Matrix3d covariance;

    //平面方程协方差
    Eigen::Matrix<double, 6, 6> plane_var_;

    //平面内点到平面中心的最大半径
    float radius_ = 0;

    //
    float min_eigen_value_ = 1;
    float mid_eigen_value_ = 1;
    float max_engen_value_ = 1;

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

        //平面方程协方差初始化为0
        plane_var_ = Eigen::Matrix<double, 6,6>::Zero();

        //平面协方差初始化为0
        covariance_ =Eigen::Matrix3d::Zero();

        //平面中心初始化为0
        center_ = Eigen::Vector3d::Zero();

        //法向量初始化为0
        normal_ = Eigen::Vector3d::Zero();
    }
} Voxelplane;


//哈希体素
class VOXEL_LOCATION
{
    public:

      //将体素空间离散成体素索引
      int64_t x,y,z;

      //索引键值
      VOXEL_LOCATION(intt64_t vx = 0, int64_t vy = 0, int64_t vz = 0 : x(vc) , y(vy) , z(vz)) {};

      //判断索引是否一致
      bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && other.z )};
}

//降采样结构
struct DS_POINT
{
    ///xyz坐标
    float xyz[3];
    //强度
    float intensity;
    //积累原始点
    int count = 0;
}
/*
各个点云类型的别名定义，和imu信息，方便后续修改和使用

*/

#pragma once
#include<Eigen/Eigen>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZRGBA PointTypeRGBA;

typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<PointTypeRGB> PointCloudRGB;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
//Eigen::aligendPallocator是Eigen库提供的一个内存分配器，用于确保分配的内存满足特定的对齐要求，以提高性能。
typedef std::vector<PointType,Eigen::aligned_allocator<PointType>> PointVector;

//float和double的别名，方便后续修改
typedef Eigen::Vector2f V2F;
typedef Eigen::Vector3f V3F;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3f M3F;
typedef Eigen::Matrix3d M3D;

//M(a,b)表示一个a行b列的矩阵，元素类型为double ,矩阵中a，b带括号是防止参数表达式出现问题，保险做法
#define MD(a,b) Eigen::Matrix<double,(a),(b)> 
#define MF(a,b) Eigen::Matrix<float,(a),(b)>

//VF(a)表示一个a行1列的向量，元素类型为float
#define VF(a) Eigen::Matrix<float,(a),1>
#define VD(a) Eigen::Matrix<double,(a),1>

//某个imu时间段内的位姿信息，对应lidar预积分状态
struct Pose6D
{
    double offset_time;   //    imu的测量时间相对于liadr的第一个点云的时间偏移，单位为秒
    double acc[3];     //加速度，单位为m/s^2
    double gyr[3];    //陀螺仪，单位为rad/s
    double vel[3];    //速度，单位为m/s
    double pos[3];   //位置，单位为m
    double rot[9];   //旋转矩阵，单位为无量纲
};
/*
imu处理模块：
IMU 初始化
IMU 状态传播
LiDAR 点云运动畸变去除
LiDAR-IMU 外参设置
IMU 噪声参数设置
是否启用 IMU / 重力 / bias / 曝光估计
*/

#pragma once

#include <fstream>    //文件读写，iostream
#include <Eigen/Eigen>

#include "common_lib.h"   //项目自己的公共头文件
#include "utils/types.h"
#include <condition_variable>   //线程同步变量条件
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>


//声明一个序列比较函数 {return (x.curvature < y.curvature);};
const bool time_list(PointTypev &x, PointType &y);

class ImuProcess
{

    public:;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW    //加快矩阵运算，使用内存对齐

      ImuProcess();
      ~ImuProcess();

      //重置和指定时间戳将次imu作为上一帧imu数据
      void Reset();
      void Reset(double start_timestamp, const sensor_msgs::ImuConsterPtr &lastimu);

      //设置雷达到imu的外参。1设置平移和旋转，2设置平移，3使用变换矩阵设置
      void set_extrinsic(const V3D &transl, const M3D &rot);
      void set_extrinsic(const V3D &transl);
      void set_extrinsic(const MD(4,4) & T);

      //设置imu噪声参数
      void set_gyr_cov_scale(const V3D &scaler);   //设置陀螺仪测量噪声
      void set_acc_cov_scale(const V3D &scaler);   //加速度计测量噪声
      void set_gyr_bias_cov(const V3D &b_g);        //陀螺仪游走噪声
      void set_acc_bias_cov(const V3D &b_a);         //加速度计游走噪声
      
      void set_inv_expo_cov(const double &inv_expo);     //曝光噪声

      void set_imu_init_frame_num(const int &num);  //设置初始化帧数

      //禁用估计功能
      void disable_imu();                   
      void disable_gravity_est();
      void disable_bias_est();
      void disable_exposure_est();
      
      //主要处理函数
      void process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
      void UndistorPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

      ofstream fout_imu;    //用于输出保存日志
      double IMU_mean_acc_norm;  //IMU 初始化阶段估计得到的平均加速度模长，静止时测量主要是重力是否正常
      V3D unbaised_gyr;    //去偏差后的角速度

      //协方差参数
      V3D cov_acc;    //加速度计测量噪声
      V3D cov_gyr;    //陀螺仪测量噪声
      V3D cov_bias_gyr;    //陀螺仪游走噪声
      V3D cov_bias_acc;    //加速度计游走噪声

      double cov_inv_expo;  //曝光参数噪声
      double frist_lidar_time;   //第一帧lidar时间

      bool imu_time_init = false;    //imu是否初始化
      bool imu_need_init = true;    //imu是否需要初始化
      
      //常用矩阵和向量，lidar类型
      M3D Eye3d;
      V3D Zero3d;
      int lidar_type;

    private:
      
      //使用N帧数量初始化
      void IMU_init(const LidarMeasureGroup &meas, StatesGroup &state, int &N);

      //这时无法根据 IMU 做精确状态传播和点云去畸变，只能做一个简单处理。
      void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);


    
      PointCloudXYZI pcl_wait_proc;                 //等待处理的点云
      sensor_msgs::ImuConsterPtr last_imu;          //上一帧imu
      PointCloudXYZI::Ptr cur_pcl_un_;              //当前去畸变的点云指针

      vector<Pose6D> IMUpose;                       //保存liadr一帧时间内imu积分得到的位姿

      M3D Lid_rot_to_IMU;                           //lidar到imu坐标系下的外参
      V3D Lid_offset_to_IMU;                        //lidar到imu下的补偿

      V3D mean_acc;                                //初始化累计的加速度
      V3D mean_gyr;                                //初始化累计的角速度

      V3D angvel_last;                             //上一次imu的角速度
      V3D acc_s_last;                              //上一次imu的加速度

      double last_prop_end_time;                   //上一次传播结束状态
      double time_last_scan;                       //上一帧雷达的扫描时间

      int init_iter_num = 1, MAX_INI_COUNT = 20;    //当前累计初始化数 ，  最大初始化数

      bool b_first_frame = true;                    //是否第一帧
      bool imu_en = true;                         //是否启用imu
      bool gravity_est_en = true;                 //是否启用重力估计
      bool ba_bg_est_en = true;                   //是否估计加速度陀螺仪噪声
      bool exposure_estimate_en = true;           //曝光估计
}
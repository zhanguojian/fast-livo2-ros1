/*
LIVMapper = 系统主控类
负责：
1. 读取参数
2. 初始化模块
3. 接收 LiDAR / IMU / Image 数据
4. 同步多传感器数据
5. 调用 IMU 去畸变
6. 调用 LIO / VIO 状态估计
7. 更新地图
8. 发布点云、里程计、路径、图像等结果
*/

# pragma once

#include "IMU_Processing.h"
#include "utils/types.h"
#include "vio.h"
#include "preprocess.h"
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/path.h>
#include <unordered_map>
#include <vikit/camera_loader.h>

class LIVMapper
{
    public:
      LIVMapper(ros::NodeHandle &nh):
      ~LIVMapper();

      //初始化
      void initializeSubscribersAndPublisher(ros::NodeHandle &nh, image_transport::image_transport &it);
      void initializeComponents();
      void initializeFile();

      //运行主函数
      void run();
      void gravityAlignment();
      void handleFirstFrame();
      void stateEstimationAndMapping();
      void handleVio();
      void handleLio();
      void savePCD();
      void processImu();

      //imu和点云处理
      bool sync_packages(LidarMeasureGroup &meas);
      void prop_imu_once(StatesGroup &imu_pro_state, const double dt, V3D acc_avr, V3D angvel_avr);
      void imu_prop_callback(const ros::TimerEvent &e);
      void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::ptr &input_cloud, PointCloudXYZI::ptr &trans_cloud);
      void pointBodyToWorld(const PointType &pi, PointType &po);
      void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);

      //点云和imu回调处理
      void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
      void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msgin);
      void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
      void imu_cbk(const sensor_msgs::ImageConstPtr &msg_in);
      
      //ros发布
      void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
      void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
      void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
      void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
      void publish_odometry(const ros::Publisher &pubOdometryAftMapped);
      void publish_mavros(const ros::Publisher &mavros_pose_publsiher);
      void publish_path(const ros::Publsher &pubPath);
      
      void readParameters(ros::NodeHandle &nh);

      template<typename T> void set_posestamp(T &out);
      template<typename T> void pointBodyToWorld(const Eigen::Matrix<T,3,1> &pi, Eigen::Matrix<T, 3,1> &po);
      template<typename T> void Eigen::Matrix<T,3,1> pointBodyToWorld(const Eigen::Matrix<T, 3,1>  &pi)
 

      cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
      
      //多线程同步变量
      std::mutex mtx_buffer, mtx_buffer_imu_prop;
      std::condition_variable sig_buffer;

      SLAM_MODE slam_mode;
      std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;

      //传感器话题与外参
      string root_dir;
      string lid_topic,imu_topic,seq_name,img_topic;
      V3D extT;
      M3D exrR;


      /*
        last_timestamp_lidar  检查 LiDAR 时间是否回退
        last_timestamp_imu    检查 IMU 时间是否回退
        last_timestamp_img    检查图像时间是否回退
        imu_time_offset       IMU 时间偏移补偿
        lidar_time_offset     LiDAR 时间偏移补偿
        img_time_offset       图像时间偏移补偿
      
      */
      double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
      double imu_time_offset  = 0.0;
      double lidar_time_offser = 0.0;

      /*
        pcd_save_en              是否保存点云
        img_save_en              是否保存图像
        pub_effect_point_en      是否发布有效匹配点
        pose_output_en           是否输出轨迹文件
        imu_en                   是否启用 IMU
        gravity_est_en           是否估计重力
        ba_bg_est_en             是否估计 IMU bias
        dense_map_en             是否生成稠密地图
        img_en                   是否启用图像
        normal_en                是否使用法向
        exposure_estimate_en     是否估计曝光
        raycast_en               是否启用 raycast
        lidar_en                 是否启用 LiDAR
      */
      int feats_down_size = 0, max_iterations = 0;

      double res_mean_last = 0.05;
      double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
      double blind_rgb_points = 0.0;
      double filter_size_surf_min = 0;
      double filter_size_pcd = 0;
      double _first_lidar_time = 0.0;
      double match_time = 0, solve_time = 0, solve_const_H_time = 0;

      bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
      deque<sensor_msgs::Imu> prop_imu_buffer;
      sensor_msg::Imu newest_imu;
      double lasest_ekf_time;
      nav_msgs::Odometry imu_prop_odom;
      ros::Publisher pubImuPropOdem;


      bool gravity_align_en = false, gravity_align_finished = false;
      bool dense_map_flag = false;

      int img_en = 1, imu_int_frame = 3,
      bool normal_en = true;
      bool exposure_estimate_en = false;
      double exposure_time_init = 0.0;
      bool inverse_composition_en = false;
      bools raycast_en = false;
      int lidar_en = 1;
      bool is_first_frame = false;
      int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
      double outlier_threshold;
      double plot_time;
      int frame_cnt;
      double img_time_offset = 0;




      //传感器数据缓存区
      deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
      deque<double> lid_header_time_buffer;
      deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
      deque<cv::Mat> img_buffer;
      deque<double> img_time_buffer;

      vector<pointwithVar> pv_list;
      vector<double> extrinR;
      vector<double> extrinT;
      vector<double>cameraextrinT;
      vector<double>cameraextrinR;
      double IMG_POINT_COV;

    /* visual_sub_map   视觉子地图
    feats_undistort             去畸变后的 LiDAR 特征点
    feats_down_body             body 系下的降采样点
    feats_down_world            world 系下的降采样点
    pcl_w_wait_pub              等待发布的世界系点云
    pcl_wait_pub                等待发布的当前点云
    pcl_wait_save               等待保存的 RGB 点云
    pcl_wait_save_intensity     等待保存的 intensity 点云
    */
      PointCloudXYZI::ptr visual_sub_map;
      PointCloudXYZI::ptr feats_undistort;
      PointCloudXYZI::ptr feats_down_body;
      PointCloudXYZI::ptr feats_down_world;
      PointCloudXYZI::ptr pcl_w_wait_pub;
      PointCloudXYZI::ptr pcl_wait_pub;
      PointCloudXYZRGB::ptr pcl_wait_save;
      PointCloudXYZI::ptr pcl_waitt_save_intensity;

      ofstream fout_pre, fout_out, fout_pcd_pos, fout_point;

      //定义体素滤波器对象
      pcl::VoxelGrid<PointType> downSizeFilterSurf;

      V3D euler_cur;

      LidarMeasureGroup LidarMeasureGroup;
      StatesGroup _state;
      StatesGroup state_propagat;

      nav_msgs::Path path;
      nav_msgs::Odometry odomAftMapped;
      geometry_msgs::Quaternion  geoQuat;
      geometry_msgs::set_posestamp msg_body_pose;

      //主要模块指针  LiDAR 预处理模块，IMU 处理模块，体素地图管理模块，视觉模块
      PreprocessPtr p_pre;
      PreprocessPtr P_imu;
      VoxelMapManagePtr voxelmap_manage;
      VIOManagerPtr vio_manager;


      //ros发布完整点云。法向，视觉子地图，有效匹配点，地图点云，里程计，轨迹，图像
      ros::Publisher plane_pub;
      ros::Publisher voxel_pub;
      ros::publisher pubLaserCloudFullRes;
      ros::publisher pubNormal;
      ros::Publisher pubSubVisualMap;
      ros::Publisher pubLaserCloudEffect;
      ros::Publisher pubLaserCloudMap;
      ros::Publisher pubOdometryAftMapped;
      ros::Publisher pubpath;
      ros::Publisher pubLaserCloudDyn;
      ros::Publisher pubLaserCloudDynRmed;
      ros::Publisher pubLaserCloudDynDbg;
      image_transport::Publisher pubImage;
      ros::publisher mavros_pose_publsiher;
      ros::Timer imu_prop_timer;

      //订阅点云，imu，图像
      ros::Subscriber sub_pcl;
      ros::Subscriber sub_imu;
      ros::Subscriber sub_img;

      int frame_num = 0;
      double aver_time_consu = 0;
      double aver_time_icp = 0;
      double aver_time_map_inre = 0;
      bool colmap_output_en = false;

}
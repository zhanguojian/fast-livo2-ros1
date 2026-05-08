/*LIVMapper主要做了以下几个事。
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

#include "LIVMapper.h"
#include <cinttypes>
#include <pthread.h>
#include <random>

//LIVMapper的初始化
LIVMapper::LIVMapper(ros::NodeHandle &nh) : extT(0,0,0) , extR(M3D::Identity())
{
    //初始化外参
    extrinT.assign(3,0.0);
    extrinR.assign(9,0.0);
    cameraextrinT.assign(3,0.0);
    camerarxtrinR.assign(9,0.0);

    //初始化imu和lidar模块
    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());

    //加载参数
    readParameters(nh);

    //加载体素地图配置
    VoxelMapConfig voxel_config;
    loadVoxelConfig(nh, voxel_config);

    //点云指针重置
    visual_sub_map.reset(new PointCloudXYZI());
    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());

    pcl_w_wait_pub.reset(new PointCloudXYZI());
    Pcl_wait_pub.reset(new PointCloudXYZI());
    pcl_wait_save.reset(new PointCloudXYZRGB());
    pcl_waitt_save_intensity.reset(new PointCloudXYZI());

    //初始化地图管理器
    voxelmap_manage.reset(new voxelManager(voxel_config, voxel_map));

    //初始化视觉管理器
    vio_manager.reset(new VIOManager());

    root_dir = ROOT_DIR;


    //初始化文件路经
    initializeFile();

    //初始化组件模块
    initializeComponents();
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {};

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
    //common
    nh.param<string>("common/lid_topic",lid_topic, "/livox/lidar")
    nh.param<string>("common/imu_topic",imu_topic, "/livox/imu")
    nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
    nh.param<int>("common/img_en", img_en, 1);
    nh.param<int>("common/lidar_en", lidar_en, 1)
    nh.param<string>("common/img_topic",img_topic,"left_camera/image");

    //vio
    nh.param<bool>("vio/normal_en",normal_en,true);
    nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
    nh.Param<int>("vio/max_iterations", max_iterations, 5);
    nh.Param<double>("vio/img_ppoint_cov", IMG_POINT_COV, 100);
    nh.param<bool>("vio/raycast_en",raycast_en, fasle);
 
    //--------------------待完成



}

void LIVMapper::initializeComponents()
{
    downSizeFilterSurf.setLeafSize(filter_size_surf_min,filter_size_surf_min,filter_size_surf_min);

    //将lidar到imu的平移和旋转赋值
    extT << VEC_FROM_ARRLY(extrinT);
    extR << MAT_FROM_ARRAY(extrinR);

    //传入体素地图中设置外部参数，确保地图构建正确
    voxelmap_manage->extT_ << VEC_FROM_ARRLY(extrinT);
    voxelmap_manage->extR_ << MAT_GROM_ARRAY(extrinR);

    if(!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("camera model not correctly ");

    //设置vio管理器的参数
    vio_manager->grid_size = grid_size;
    vio_manager->patch_size = patch_size;
    vio_manager->outlier_threshold = outlier_threshold;
    vio_manager->setImuToLidarExtrinsic(ExtT,ExtR);
    vio_manager->setLidarToCameraExtrinsic(cameraextrinR,cameraextrinT);
    vio_manager->state = &_state;
    vio_manager->state_propagat = &state_propagat;
    vio_manager->max_iterations = max_iterations;
    vio_manager->img_point_cov = IMG_POINT_COV;
    vio_manager->normal_en = normal_en;
    vio_manager->inverse_composition_en = inverse_composition_en;
    vio_manager->raycast_en = raycast_en;
    vio_manager->grid_n_width = grid_n_width;
    vio_manager->grid_n_height = grid_n_height;
    vio_manager->patch_pyrimid_level = patch_pyrimid_level;
    vio_manager->exposure_estimate_en = exposure_estimate_en;
    vio_manager->colmap_output_en = colmap_output_en;

    vio_manager->initializeVIO();

    //设置imu模块的初始化参数
    p_imu->set_extrinsic(extT,extR);
    p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_inv_expo_cov(inv_expo_cov);
    p_imu->set_gyr_bias_cov(0.0001,0.0001,0.0001);
    p_imu->set_acc_bias_cov(0.0001,0.0001,0.0001);
    p_imu->set_imu_init_frame_num(imu_init_frame);

    if(!imu_en) p_imu->disable();
    if(!gravity) p_imu->disable_gravity_est();
    if(!ba_bg_est_en) p_imu->disable_bias_est();
    if(!exposure_estimate_en) p_imu->disable_exposure_est()

    slam_mode = (img_en && lidar_en) ? LIVO : imu_en ? LIVO :  ONY_LIO : ONLY_LO;


}

void LIVM::initializeSubscribersAndPublisher(ros::NodeHandle &nh, image_transport::image_transport &it)
{
    sub_pcl = p_pre->lidar_type == AVIV ?     
                nh.Subscriber(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this):
                nh.Subscriber(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
            

    sub_imu = nh.Subscriber(imu_topic, 200000, &LIVMapper::imu_cbk,this);
    sub_img = nh.Subscriber(img_topic, 200000, &LIVMapper::img_cbk,this);

    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered",100);
    pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
    pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before" , 100);
    pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_map",100);
    pubOdometryAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_map_to_init", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path",10);

    plane_pub = nh.advertise<visualization_msgs::Maker>("/planner_normal",1);
    voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);

    pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
    pubLaserCloudDynRmed = nh.advertise<sensor_msgs::Pointcloud2>("/dyn_obj_removed", 100);
    pubLaserCloudDynbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbj_hist", 100);

    mavros_pose_publsiher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    pubImage = it.advertise("/rgb_img", 1);

    //imu高频位姿递推与发布
    pubImgPropOdom = nh.advertise<nav_msgs::Odometry>(".LIVO2/imu_propagate", 10000);
    imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);

    voxelmap_manage->voxel_map_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);

} 

void LIVMapper::run()
{
    ros::Rate rate(5000);  //主控制器的循环频率
    while(ros::ok())
    {
       ros::spinOnce();   //触发传感器回调，把 LiDAR、IMU、Image 数据放入对应 buffer。
       if(!sync_packages(LidarMeasures))
       {
        rate.sleep();
        continue;
       }
       
       handleFirstFrame();

       processImu();

       stateEstimationAndMapping();

    }

    savePCD();
}

bool LIVM::sync_packages(LidarMeasureGroup &meas)
{ 
    if(lid_raw_data_buffer.empty() && lidar_en) return false;
    if(img_buffer.empty()&& img_en) return false;
    if(imu_buffer.empty()&& imu_em) return false;

    switch (slam_mode_)
    {
        case LVIO:
        {

            EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
            switch (last_lio_vio_flag)
            {
            case WAIT:
            case VIO:
            {
                double img_caputure_time = img_time_buffer().front + exposure_time_init;
                
                if(meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();

                double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.buffer.back().curvature / double(1000);
                double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

                if(img_caputure_time < meas.last_lio_update_time + 0.00001)
                {
                    img_buffer.pop_front();
                    img_time_buffer.pop_front();
                    ROS_ERROR("Data Cut");
                    return false;

                }

                if(img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
                {
                    return fasle;
                }

            }
            case LIO:

            default:
            {

            }

            }
            break;
        }


        case ONLY_LIO:
        {


        }

        case ONLY_LO:
        {

        }
        default:
        {

        }

    }

    ROS_ERROR("out sync");
}
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
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstddef>
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
    initializeFiles();

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


//初始化参数
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


//初始化订阅和发布器
void LIVM::initializeSubscribersAndPublisher(ros::NodeHandle &nh, image_transport::image_transport &it)
{

    //lidar点云订阅
    sub_pcl = p_pre->lidar_type == AVIV ?     
                nh.Subscriber(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this):
                nh.Subscriber(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
            

    //imu数据订阅
    sub_imu = nh.Subscriber(imu_topic, 200000, &LIVMapper::imu_cbk,this);

    //img数据订阅
    sub_img = nh.Subscriber(img_topic, 200000, &LIVMapper::img_cbk,this);

    //发布配准后点云
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered",100);

    //发布视觉法向量点
    pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);

    //发布优化前视觉子地图
    pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before" , 100);

    //发布有效点云
    pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);

    //发布全局地图
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_map",100);

    //发布优化后的里程计信息
    pubOdometryAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_map_to_init", 10);

    //发布路经
    pubPath = nh.advertise<nav_msgs::Path>("/path",10);

    //发布平面法线
    plane_pub = nh.advertise<visualization_msgs::Maker>("/planner_normal",1);

    //发布体素结构
    voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);

    //发布动态物体
    pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);

    //发布移除障碍物后的点云
    pubLaserCloudDynRmed = nh.advertise<sensor_msgs::Pointcloud2>("/dyn_obj_removed", 100);

    //发布调试点云
    pubLaserCloudDynbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbj_hist", 100);

    mavros_pose_publsiher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    //发布rgb图像
    pubImage = it.advertise("/rgb_img", 1);

    //imu高频位姿递推与发布
    pubImgPropOdom = nh.advertise<nav_msgs::Odometry>(".LIVO2/imu_propagate", 10000);
    imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);

    //体素平面订阅
    voxelmap_manage->voxel_map_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);

} 

//slam控制器主流程
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
       
       //处理第一帧
       handleFirstFrame();

       //处理imu
       processImu();

       //状态估计和建图
       stateEstimationAndMapping();

    }

    //保存pcd建图
    savePCD();
}

//点云重组
bool LIVM::sync_packages(LidarMeasureGroup &meas)
{ 
    //判断缓存是否异常
    if(lid_raw_data_buffer.empty() && lidar_en) return false;
    if(img_buffer.empty()&& img_en) return false;
    if(imu_buffer.empty()&& imu_em) return false;

    //选择slam模式
    switch (slam_mode_)
    {
        //激光视觉模式
        case LVIO:
        {

            //把处理完的同步更新为上次处理同步记号
            EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;

            //选择上次标记处理同步
            switch (last_lio_vio_flg)
            {
                
            case WAIT:

            //上次标记为vio,这次执行lio，储存imu和点云
            case VIO:  
            {
                //以图像为基点同步时间
                double img_caputure_time = img_time_buffer().front + exposure_time_init;
                
                //如果上次更新小于0,将上次lio时间更新为雷达开始时间
                if(meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();

                //雷达最新时间为最后帧时间加点云偏移时间
                double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.buffer.back().curvature / double(1000);
                //imu最新时间为最后一个imu时间
                double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

                //如果图像时间小于上次lio更新时间则图像过时
                if(img_caputure_time < meas.last_lio_update_time + 0.00001)
                {
                    img_buffer.pop_front();
                    img_time_buffer.pop_front();
                    ROS_ERROR("Data Cut");
                    return false;

                }

                //如果相机时间大于雷达和imu的最新时间则说明相机时间提前
                if(img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
                {
                    return fasle;
                }

                //创建imu和img容器
                struct MeasureGroup m;

                //清空旧缓存
                m.imu.clear();

                //将图像时间赋值为这次lio时间
                m.lio_time = img_capture_time;

                //缓存上锁保护
                mtx_buffer.lock();

                while(!imu_buffer.empty())
                {
                
                    //如果imu开始大于这次lio时间则跳出
                   if(!imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;

                   //如果imu时间大于上次lio时间则说明正常
                   if(imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

                   //删除缓存中已经存入的点
                   imu_buffer.pop_front();
                }

                //解锁释放资源和唤醒
                mtx_buffer.unlock();
                sig_buffer.notify_all();
                
                //将同步中分为下一次的点云转换为当前处理点云
                *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
                
                //将下一个点云存储清空
                PointCloudXYZI().swap(*meas.pcl_proc_next);

                //记录lidar缓存中点云帧数
                int lid_frame_num  = lid_raw_data_buffer.size();
                //预设最大点云数
                int max_size = meas.pcl_proc_cur -> size() + 24000 * lid_frame_num;
                //预分配当前和下一次点云数
                meas.pcl_proc_cur->reserve(max_size);
                meas.pcl_proc_next->reserve(max_size);

                //当点云缓存不为空
                while (!lid_raw_data_buffer.empty())
                {

                    //如果开始点云时间大于图像时间则说明点云异常
                    if(lid_header_time_buffer.front() > img_capture_time ) break;

                    //取出开始点云
                    auto pcl(lid_raw_data_buffer.front()->points);

                    //取出开始点云时间
                    double frame_header_time(lid_header_time_buffer.front());

                    //计算最大偏移时间即开始点云到这次图像为基点的lio的时间
                    float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

                    //遍历点云
                    for(int i = 0; i < pcl.size; i++)
                    {

                    //取出点云在临时容器中
                    auto pt = pcl[i];
                    //重组的点云小于这次lio时间偏移则重组为这次lio
                    if (pcl[i].curvature < max_offs_time_ms)
                    {
                        pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
                        meas.pcl_proc_cur->point.push(pt);
                    }

                    //否则为下次lio点云
                    else
                    {
                        pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
                        meas.pcl_proc_next->points.push_bac(pt);
                    }

                    //删除已经处理的点云和时间
                    lid_raw_data_buffer.pop_front();
                    lid_header_time_buffer.pop_front();

                    }
                }

                //将处理的measure容器放入lidar大容器
                meas.measure.push_back(m);

                //标记这次为lio
                meas.lio_vio_flg = LIO;

                return true;


            }


            case LIO: //上次执行完lio后这次执行vio
            {
                //记录图像时间作为基点
                double img_capture_time = img_time_buffer.front() + exposure_time_init;
                //标记为vio处理
                meas.lio_vio_flg = VIO;

                //清空measure历史
                meas.measure.clear();

                //记录开始imu时间
                double imu_time = imu_buffer.front()->header.stamp.toSec();

                //创建img容器存储img
                struct MeasureGroup m;

                //将当前图像时间设置为VIO时间
                m.vio_time = img_capture_time;
                //将当前lio时间设置为上次lio时间
                m.lio_time = meas.last_lio_update_time;
                //存入img
                m.img = img_buffer.front();

                //img上锁删除释放
                mtx_buffer.lock();
                img_buffer.lock();
                img_time_buffer.pop_front();
                mtx_buffer.unlock();
                sig_buffer.notify_all();

                //将m存入lidar大容器
                meas.measures.push_back(m);
                lidar_pushed  = false;
                return true;
                
            }



            default:
            {
                return false;

            }

            }
            break;
        }


        case ONLY_LIO:
        {

            //检查上次lio时间，如果是开始时间则将其设置为点云最开始时间
            if (meas.last_lio_update_time < 0.0 ) meas.last_lio_update_time = lid_header_time_buffer.front();

            //处理点云标识，存储在容器
            if (!lidar_pushed)
            {
                //取出点云放入容器
                meas.lidar = lid_raw_data_buffer.front();
                //如果点云数下无点云则抛弃
                if(meas.lidar->points.size() <= 1) return false;

                //容器中开始时间设置为点云最开始时间
                meas.lidar_frame_beg_time = lid_header_time_buffer.front();

                //最后时间则设置为帧开始时间加最后一个点云的偏移时间
                meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);

                //将容器中的点云设置为当前处理点云
                meas.pcl_proc_cur = meas.lidar;
                lidar_pushed = true;
            }

            //如果imu的时间，最后时间小于雷达最后时间则等待覆盖
            if(imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
            {
                return false;
            }
        
            //imu存储容器
            struct MeasureGroup m;

            //容器历史清除
            m.imu.clear();
            //lio时间设置为雷达帧最后时间
            m.lio_time = meas.lidar_frame_end_time;
            mtx_buffer.lock();

            while(!imu_buffer.empty())
            {
                if(imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
                m.imu.posh_back(imu_buffer.front());
                imu_buffer.pop_front();
            }

            //将取出的lidar点云和时间清除
            lid_raw_data_buffer.pop_front();
            lid_header_time_buffer.pop_front();

            mtx_buffer.unlock();
            sig_buffer.notify_all();

            //标记和存入lidar容器
            meas.lio_vio_flg = LIO;
            meas.MeasureGroup.push_back(m);

            lidar_pushed = false;
            return true;

            break;

        }

        case ONLY_LO:
        {

        }
        default:
        {
            return false;

        }

    }

    ROS_ERROR("out sync");
}

//处理第一帧
void LIVMapper::handleFirstFrame()
{
    if(!is_first_frame)
    {
        //记录第一帧时间给imu处理模块
        _first_lidar_time = LidarMeasures.last_lio_update_time;
        p_imu->frist_lidar_time = _first_lidar_time;

        is_first_frame = true;
        count << "FIRST LIDAR FRAME" << endl;
    }
}


//在 IMU 初始化完成后，把当前地图/状态坐标系旋转到“重力对齐坐标系”下，使重力方向变成全局 z 轴负方向 (0, 0, -1)。
void LIVMapper::gravityAlignment()
{
    //重力对齐条件
    if (!p->imu_need_init && !gravity_align_finished)
    {
        std::cout << "Gravity Alignment Start" << endl;

        //目标重力方向和当前重力方向
        V3D ez(0,0,-1), gz(_state.gravity); 

        //估计一个从gz到ez旋转的四元数，gz是起始向量，ez是目标向量。FromTwoVectors() 直接返回一个新的四元数
        Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz,ez);   

        //将四元数转换为旋转矩阵
        M3D G_R_I0 = G_q_I0.toRotationMatrix();     

        //将当前状态下的旋转，位置，速度，重力都应用对齐的旋转
        _state.pos_end = G_R_I0 * _state.pos_end;
        _state.rot_end = G_R_I0 * _state.rot_end;
        _state.vel_eng = G_R_I0 * _state.vel_end;
        _state.gravity = G_R_I0 * _state.gravity;

        //重力对齐完成标识
        gravity_align_finished = true;
        std::cout<< "Gravity Alignment Finished" << std::endl;
        
    }
}

//处理 IMU 数据，完成状态预测、点云去畸变，然后根据配置决定是否做重力对齐，最后把处理后的状态和点云传给后续 voxel map 模块。
void LIVMapper::processImu()
{
    //imu模块进行递推和去畸变
    p_imu->Process2(LidarMeasureGroup, _state, feats_undistort);

    //重力对齐
    if(gravity_align_en) gravityAlignment();

    //递推状态更新
    state_propagat = _state;

    //体素地图状态和去畸变点云更新
    voxelmap_manage->state_ = _state;
    voxelmap_manage->feats_undistort = feats_undistort;

}


//基于imu测量的一次递推状态传播
void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr , V3D angvel_avr)
{
    double mean_acc_norm = p_imu->IMU_mean_acc_norm;
    acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
    angvel_avr -= imu_prop_state.bias_g;

    //自姿态递推，有角速度递推得到旋转位姿矩阵
    M3D Exp_f = Exp(angvel_avr, dt);

    //旋转位更新
    imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

    //加速度转换.rot_end * acc_var 把平均加速度转换到世界坐标下， 再加上重力则得到世界坐标系下的真实加速度
    V3D acc_imu = imu_pro_state.rot_end * acc_avr + V3D(imu_pro_state.gravity[0], imu_pro_state.gravity[1], imu_pro_state.gravity[2]);

    //位置递推
    imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt ;

    //速度递推
    imu_pro_state.vel_end = imu_pro_state,vel_end + acc_imu *dt;

}


//定时器回调处理并构建里程计信息发布
void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{

    //确保imu模块已经初始化，且有新数据和完成一次状态更新
    if(p_imu->ime_need_init || !new_imu || !ekf_finish_once )
    {
        return;
    }

    //已经传播递推的imu数据上锁
    mtx_buffer_imu_prop.lock(); 

    //标记当前imu数据已被处理，等待下一次数据更新
    new_imu = false;

    //启用imu传播和传播缓存不为空
    if (imu_prop_enable && !prop_imu_buffer.empty())
    {
        static double last_t_from_lidar_end_time = 0;

        //状态更新标识
        if(state_update_flg)
        {
            //获取最新的状态
            imu_propagate = lastest_ekf_state;

            //如果递推容器不为空且递推的时间戳小于最新的递推时间
            while(!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < lastest_ekf_time)
            {
                //删除过时的递推
                prop_imu_buffer.pop_front();
            } 

            //重置时间基准
            last_t_from_lidar_end_time = 0;

            //缓存中递推imu缓存容器
            for(int i = 0, i < prop_imu_buffer.size(); i++ )
            {
                //计算当前 IMU 相对 EKF 的时间，上次递推时间和这次imu时间戳相减，
                double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - lastest_ekf_state;

                //得到当前 IMU 与上一次传播 IMU 之间的时间差
                double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;

                //提取加速度和角速度
                V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].accleration.y, prop_imu_buffer[i].accleration.z);
                V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z)

                //递推
                prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);

                //使用上一次的递推时间做时间基准更新
                last_t_from_lidar_end_time = t_from_lidatr_end_time;
            }

            //状态更新标识
            state_update_flg = false;

        }
        //不状态更新
        else
        {
            //获取加速度和角速度
            V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, neast_imu.linear_acceleration.z);
            V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);

            //与上面相同得到递推的时间
            double t_from_lidar_end_time = newest_imu.header.toSec() - lastest_ekf_time;
            double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
            prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
            last_t_from_lidar_end_time = t_from_end_time;
        }


        //从传播器中提取最新的位姿和速度信息
        V3D posi, vel_i;
        Eigen::Quaterniond q;
        posi = imu_propagate.pos_end;
        vel_i = imu_propagate.vel_end;
        q = Eigen::Quaterniond(imu_propagate.rot_end);

        //构建里程计消息并发布
        imu_prop_odom.header.frame_id = "world";
        imu_prop_odom.header.stamp = newest_imu.header.stamp;
        imu_prop_odom.pose.pose.position.x = posi.x;
        imu_prop_odom.pose.pose.position.y = posi.y;
        imu_prop_odom.pose.pose.position.z = posi.z;
        imu_prop_odom.pose.pose.orientation.w = q.w();
        imu_prop_odom.pose.pose.orientation.x = q.x();
        imu_prop_odom.pose.pose.orientation.y = q.y();
        imu_prop_odom.pose.pose.orientation.z = q.z();

    }
    mtx_buffer_imu_prop.lock();

}


//主 LIO 流程用 imu_buffer 做低频但准确的 LiDAR-IMU 融合更新；高频 IMU propagation 用 prop_imu_buffer 在两次 EKF 更新之间做快速状态预测
void LIVMapper::imu_cbk()
{
    //imu未启用则直接返回不接收
    if(!imu_en)  return;

    //如果上一帧lidar的时间戳小于0则返回
    if(last_timp_lidar < 0.0)  return;

    //接收imu传感器数据
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(msg_in));

    //imu时间偏移参数矫正
    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);

    //记录imu的时间戳
    double timestamp = msg->header.stamp.toSec();

    //如果上一帧lidar和当前时间戳时间大于0.5且没有开启驱动修正
    if(fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
    {
        ROS_WARN("IMU and LIDAR not synced! delta time : %lf, .\n", last_timestamp_lidat - timestamp);
    }

    //若开启时间对齐则重新设置imu的时间对齐
    if(ros_driver_fix_en) timestamp += std::round(last_timestamp_lidat - timestamp);
    msg->header.stamp = ros::Time().fromSec(timestamp);

    //缓存上锁
    mtx_buffer.lock();

    //当前imu的时间戳小于上一帧的时间戳
    if(last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
    {
        //缓存解锁和唤醒
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        ROS_ERROR("imu loop back, offset: %lf \n" ,last_timestamp_imu - timestamp);
        return;
    }

    //上一帧imu时间戳更新
    last_timestamp_imu = timestamp;

    //将imu传感器数据放入缓存
    imu_buffer.push_back(msg);

    //内存解锁
    mtx_buffer.unlock()

    //imu递推功能开启
    if(imu_prop_enable)
    {
        //imu递推缓存上锁
        mtx_buffer_imu_prop.lock();

        //imu递推功能开启
        if(imu_prop_enable && !p_imu->imu_need_init)
        {
            //将imu传感器数据放入递推缓存容器中
            prop_imu_buffer.push_back(*msg);

        }
        
        //当前新imu更新和解锁
        newest_imu = *msg;
        new_imu = true;
        mtx_buffer_imu_prop.unlock();
    }

    //唤醒
    sig_buffer.notify_all();

}


//lidar转换为世界坐标系，lidar->imu->世界坐标系
void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
    //创建一个临时指针容器将旧点云释放
    PointCloudXYZI().swap(*trans_cloud);

    //将清空的点云根据输出预处理的点云重新预分配内存
    trans_cloud->reserve(input_cloud->size());

    //遍历点云
    for(size_t i , i<input_cloud->size(); i++)
    {
        //取出当前点
        pcl::PointXYZINormal p_c = input_cloud->points[i];

        //将点转换为向量形式
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);

        //将点云位姿进行转换，先转到imu在转到世界坐标系
        p = (rot * (extR * p + extT) + t);

        PointType pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.std::push_back(pi);
    }

}

//状态估计与建图控制器
void LIVMapper::stateEstimationAndMapping()
{
    switch(LidarMeasures.lio_vio_flg)
    {
        case VIO:
          handleVIO();
          break;
        case LIO:
  //      case LO:
          
    }
}


//lio激光模块处理
void LIVMapper::handleLIO()
{
    //从旋转矩阵中选取欧拉角
    euler_cur = RotMtoEuler(_state.rot_end);

    //排除空指针或者空点云
    if(pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
    {
        std::cout<< "[ LIO ] NO points" << std::endl;
        return
    };

    //计算该模块运行时间
    double t0 = omp_get_wtime();

    //对去畸变点云进行下采样
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);

    //计算下采样时间
    double t_down = omp_get_wtime();

    //下采样后的点云数
    feats_down_size = feats_down_body->points.size();

    //将下采样后的点云注册进体素地图中
    voxelmap_manage->feats_down_body_ = feats_down_body;

    //将下采样后的地图转换到世界坐标系下
    transfromLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);

    //将转换世界坐标系点云和大小注册进体素地图模块
    voxelmap_manage->feats_down_world_ = feats_down_world;
    voxelmap_manage->feats_down_size = feats_down_size;

    //如果地图没有初始化构建则进行初始化构建
    if(!lidar_map_inited)
    {
        lidar_map_inited = true;
        voxelmap_manage->buildVoxelMap();

    }



}


//视觉模块处理
void LIVMapper::handleVIO();
{
    //从旋转矩阵中提取欧拉角
    euler_cur = RotMtoEuler(_state.rot_end);

    //排除空指针或者点云
    if(pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
    {
        std::cout <<  "[ vio ] NO point" << std::endl;
        return;
    }

    //打印当前原始点云数
    std::cout << "[ VIO ] Raw feature num " << pcl_w_wait_pub->points.size() << std::endl;

    //从第一帧 LiDAR 到最近 LIO/VIO 标志时间的相对时间，减去 plot_time 后，是否小于一个由 frame_cnt 决定的时间阈值
    if(fabs(LidarMeasures.last_lio_vio_flg - _firsr_lidar_time) - plot_time < (frame_cnt / 2 * 0.1) )
    {
        vio_manager->plot_flag  = true;
    }
    else
    {
        vio_manager->plot_flag = false;
    }

    //vio视觉模块处理
    vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manage->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);





}

//标准点云回调处理
void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    //如果没有开启lidar则返回
    if(!lidar_en) return;

    //缓存加锁保护
    mtx_buffer.lock();

    //取出当前lidar时间加lidar偏移时间
    double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;

    //如果当前时间小于上一帧的时间则清空lidar缓存
    if (cur_head_time < last_timestamp_lidat)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lid_raw_data_buffer.clear();
    }

    //创建点云指针容器
    PointCloudXYZI:: Ptr ptr(new PointCloudXYZI());

    //原始点云接收预处理
    P_pre->process(msg, ptr);

    //检查点云是否为空
    if(!ptr || ptr->empty())
    {
        ROS_ERROR("RECEIVED an empty point cloud")
        mtx_buffer.unlock();
        return;
    }

    //存储点云
    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(cur_head_time);

    //更新最近的lidar时间
    last_timestamp_lidar = cur_head_time;

    //内存解锁和唤醒
    mtx_buffer.unlock();
    sig_buffer.notity_all();

}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
    //如果lidar未启用则直接返回
    if(!lidar_en) return;

    //内存上锁保护
    mtx_buffer.lock();
    
    //创建点云接收容器
    livox_ros_driver::Custom::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in))

    //如果最新的imu和lidar时间大于1s且imu缓存不为空
    if( abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0  &&  !imu_buffer.empty())
    {
        double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
        printf("\033[95mself sync imu and lidar , hard time lag is %.10lf \n\033[0m ", timediff_imu_wrt_lidar - 0.100);
    }

    //记录当前的lidar点云时间
    double cur_head_time = msg->header.stamp.toSec();
    ROS_INFO("get LIDAR, its header time : %.6f" , cur_head_time);

    //如果当前点云时间小于上一帧时间回退，则清空缓存
    if( cur_head_time < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lid_raw_data_buffer.clear();
    }

    //创建点云接收容器
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    //点云预处理
    p_pre->process(msg, ptr);

    //将点云放入容器
    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(cur_head_time);

    //更新最新点云时间
    last_timestamp_lidar = cur_head_time;

    //内存解锁和唤醒主处理线程
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    

}


//从ros中获取img数据转成普通图像格式
cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv::Mat img; 

    //bgr8 里的 8 表示每个 B/G/R 颜色通道是 8 位，像素值范围是 0 到 255，对应 OpenCV 的 CV_8UC3 图像格式。
    img = cv_bridge::tocvCopy(img_msg, "bgr8"->image);   
    return;
}

//图像数据回调
void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
    //如果相机未启用则退出
    if(!img_en ) return;
    
    //创建一个图像容器
    sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));

    //取出图像时间加偏移时间
    double msg_header_time = msg->header.stamp.toSec() + img_time_offset;

    //避免处理同一帧图像数据
    if(abs(msg_header_time - last_timestamp_img) < 0.001) return;

    //打印当前图像时间
    ROS_INFO("get image, its header time :  %.06f" , msg_header_time );

    //等待lidar初始化
    if(last_timestamp_lidar < 0) return;

    //时间回溯检查
    if(msg_header_time < last_timestamp_img) 
    {
        ROS_ERROR("image loop");
        return;
    }


    //内存上锁
    mtx_buffer.lock();

    //正确的图像时间
    double img_time_corret = msg_header_time;

    //最小帧间隔处理，新图像数据时间差异过小可能是图像异常或者重复帧
    if(img_time_corret - last_timestamp_img < 0.02)
    {
        ROS_WARN("Image need Jump: %.6f", img_time_corret);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    //将图像数据放进容器
    cv::Mat img_cur = getImageFromMsg(msg);
    img_buffer.push_back(img_cur);
    img_time_buffer.push_back(img_time_corret);

    //更新图像时间
    last_timestamp_img = img_time_corret;

    //内存解锁和唤醒
    mtx_buffer.unlock();
    sig_buffer.notify_all();


}


//发布彩色图像数据，使用ros的图像发布器
void LIVMapper::publish_img_rgb(const image_transport::publisher &pubImage, VIOManagerPtr vio_manager)
{
    //cv::Mat浅拷贝数据
    cv::Mat img_rgb = vio_manager->img_cp;

    //创建消息封装对象
    cv_bridge::Cvimage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "camera_init"
    out_msg.encoding = sensor_msgs::image_encoding::bgr8;
    out_msg.image = img_rgb;

    //发布图像
    pubImage.publish(out_msg.toIamgeMsg());
}

//发布当前帧世界坐标系点云；如果开启图像 img_en，就把点云投影到相机图像上取颜色，发布彩色点云；同时根据配置保存 PCD 地图
void LIVMapper::publish_frame_world(const ros::publisher &pubLaserCloudFullRes, VIOManager vio_manager)
{
    // 如果转到世界坐标系下点云为空则退出发布
    if(pcl_w_wait_pub->empty) return;

    //创建彩色点云容器
    PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB);

    //如果开启图像则对点云进行染色
    if(img_en)
    {
        //多帧点云累积
        static int pub_num = 1;
        *pvl_wait_pub += pcl_w_wait_pub;

        //控制发布间隔，只有累积次数达到设定值才彩色点云生成
        if(pub_num == pub_scan_num)
        {

            pub_num = 1;
            //获取累积点云数
            size_t size = pcl_wait_pub->points.size();

            //预分配点云空间
            laserCloudWorldRGB->reserve(size);

            //获取当前图像
            cv:Mat img_rgb = vio::manager->img_rgb;

            //遍历点云
            for(size_t i = 0; i < size; i++)
            {
                //创建彩色点云容器
                PointTypeRGB pointRGB;

                //取出世界点云坐标点
                pointRGB.x = pcl_wait_pub->points[i].x;
                pointRGB.y = pcl_wait_pub->points[i].y;
                pointRGB.z = pcl_wait_pub->points[i].z;

                //转换为向量形式
                V3D p_w(pcl_wait_pub->points[i].x,pcl_wait_pub->points[i].y, pcl_wait_pub[i].z);、

                //转换为相机坐标系
                V3D pf(vio_manager->new_frame->w2f(p_w));

                //世界点投影到像素坐标
                V2D pc(vio_manager->new_frame->w2f(p_w));

                //判断像素点是否在图像范围内
                if(vio->manage->new_frame->cam->isInFrame(pc.cast<int>(),3))
                {
                    //从图像中取出颜色
                    V3F pixel = vio_manager->getInterpolatedPixel(img_rgb,pc);

                    //给点云赋色
                    pointRGB.r = pixel[2];
                    pointRGB.g = pixel[1];
                    pointRGB.b = pixel[0];

                    //过滤相机太近的盲点
                    if(pf.norm() > blinf_rgb_points)
                    {
                        laserCloudWorldRGB->push_back(pointRGB);
                    }
                }


            }
        }
        //否则继续增加点云帧
        else
        {
            pub_num++;
        }
    }

    //
    if(img_en)
    {
        //发布彩色点云
        pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    }
    else
    {
        //发布普通点云
        pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    }

    //设置发布点云的时间和坐标系
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    
    //发布点云
    pubLaserCloudFullRes.publish(laserCloudmsg);


    //pcd地图保存模块
    if(pcd_save_en)
    {
        //获取去畸变点云数
        int size = feats_undistort->size();

        //创建世界坐标点云
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size,1));

        //累积帧数
        static int scan_wait_num = 0;

        //根据图像保存不同点云
        if(img_en)
        {
            //保存彩色点云
            *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
            //保存带强度的普通点云
            *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }

        //点云帧累积
        scan_wait_num++;

        //满足有点云数据，保存时间间隔大于0,累积帧数大于设定间隔
        if((pcl_wait_save->size()>0 || pcl_wait_save_intensity->size() >0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            //生成pcd路经
            pcd_index++;
            string add_points_dir(string(ROOT_DIR) + "log/PCD/") + to_string(pcd_index) + string(".pcd");

            //写入pcd工具
            pcl::PCDWriter_pcd_writer;

            //pcd写入
            if(pcd_save_en)
            {
                cout << "current scan save to /pcd/" << all_points_dir << endl;

                //如果开启相机则保存彩色点云
                if(img_en)
                {
                    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
                    PointCloudXYZRG().swap(*pcl_wait_save);
                }
                else
                {
                    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
                    PointCloudXYZI().swap(*pcl_wait_save_intensity);
                }

                //获取位姿
                Eigen::Quaterniond q(_state.rot_end);

                //
                fout_pcd_pos << _state.pos_end[0] << "" << _state.pos_end[1] << " " << _state.pos_end[2] << " "
                             << q.w()  << " "  << q.x() <<  " "  << q.y() << " " << q.z() << endl;

                //重置保存计数
                scan_wait_num = 0;
            }
        }

    }

    //清空临时点云缓存
    if(laserCloudWorldRGB->size() > 0) poinrCloudXYZI().swap(*pcl_wait_pub);
    PointCloudXYZI().swap(*pcl_w_wait_pub);

 }


 //初始化输出文件的函数
 void LIVMapper::initializeFiles()
 {  
    //保存pcd和colmap开启
    if(pcd_save_en && colmap_output_en)
    {
        //定义sh文件路经
        const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";

        //给sh文件赋予权限
        std::string chmodCommand = "chmod +x" + folderPath;

        //执行赋予权限指令
        int chmodRet = systen(chmodCommand.c_str());

        //执行失败返回值不为0
        if(chmodRet != 0)
        {
            std::cerr << "failed to set execute permission for the script"  << std::endl;
            return;
        }

        //执行sh指令
        int ececutionRet = system(folderPath.c_str());

        //执行失败返回值不为0
        if(execution != 0)
        {
        std::cerr << " failed to execute the script" << std::endl;
        return;
        }
    }
    
    //commap开启保存路经
    if(colmap_output_en)
    {
        fout_points.open(string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
    }

    //pcd开启保存路经
    if(pcd_save_interval > 0) 
    {
        fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
    }

    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
    fout_pre.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
 }


 //发布视觉子地图
 void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
 {
    //从视觉子地图获取点云数据
    PointCloudXYZI::Ptr pubLaserCloudFullRes(visual_sub_map)

    //获取点云数
    int size = laserCloudFullRes->points.size();

    //无点云退出
    if(size == 0) return;

    //创建临时点云对象指针将点云赋值，建议使用共享时针或者move
    PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
    *sub_pcl_visual_map_pub = *laserCloudFullRes

    //
    if(1)
    {
        //创建ros传感器容器
        sensor_msgs::PointCloud2 laserCloudmsg;

        //将点云转换为ros消息
        pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);

        //时间戳和坐标系
        laserCloudmsg.header.stamp = ros::Time::now();
        laserCloudmsg.header.frame_id = "camera_init";

        //发布
        pubSubVisualMap.publish(laserCloudmsg);
    }
 }

 //发布有效点云数据
 void LIVMapper::publish_effect_world(const ros::publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
 {
    //获取有效点云数
    int effct_feat_num = ptpl_list.size();

    //
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num,1));

    //点云填充
    for(int i =0 , i < effect_feat_num; i++)
    {
        laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
        laserCloufWorld->points[i].y = ptpl_list[i].point_w_[1];
        laserCloudWorld->points[1].z = ptpl_list[i].point_w_[2];
    }

    //创建ros发布点云
    sensor_msgs::PointCloud2 laserCloudFullRes3;

    //点云转换
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);

    //时间和坐标系填充
    laserCloudFullRes3.header.stamp = ros::Time::now();
    laserCloudFullRes3.header.frame_id = "camera_init";
    
    //发布
    pubLaserCloudEffect.publish(laserCloudFullRes3);
 }



 //将当前状态的位置和位姿转换为ros消息的pose结构里
 template <typename T> void LIVMapper::set_posestamp(T &out)
 {
    out.position.x = _state.pos_end(0);
    out.position.y = _state.pos_end(1);
    out.position.z = _state.pos_end(2);
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
 }


 //发布aft_mapped 是主 LIO/VIO/EKF 更新后的低频高精度里程计
 void LIVMapper::publish_odometry(cosnt ros::Publisher &pubOdometryAftMapped)
{
    //设置消息头
    pubOdometryAftMapped.header.framed_id = "camera_id";
    pubOdometryAftMapped.child_frame_id = "aft_mapped";
    pubOdometryAftMapped.header.stamp = ros::Time::now();

    //填充位姿
    set_posestamp(pubOdometryAftMapped.pose.pose);

    //创建tf广播
    static tf::TransformBroadcaster br;

    //创建tf变换
    tf::Transform transfrom;

    //设置旋转
    tf::Quaternion q;

    //设置平移
    transform().setOrigin(tf::Vector(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));

    //设置旋转
    q.setw(geoQuat.w);
    q.setw(geoQuat.x);
    q.setw(geoQuat.y);
    q.setw(geoQuat.z);
    transform(q);

    //发布tf
    br.sendTransform( tf::StampedTransform(transform, pubOdometryAftMapped.header.stamp, "camera_inint" , "aft_mapped"));


    //发布里程计消息
    pubOdometryAftMapped.publish(odomAftMapped);


}

//发布路径信息 (publish_path) 更新并发布机器人的运动路径
void LIVMapper::publish_path(const ros::publish pubpath)
{
    //设置发布位姿
    set_posestamp(msg_body_pose.pose);

    //设置消息头
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";

    //将当前位姿添加到path的轨迹组
    path.poses.push_back(msg_body_pose);
    
    //发布
    pubPath.publish(path);
}

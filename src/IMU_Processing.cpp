/*
imu初始化
*/

#include"IMU_Processing.h"


//时间序列比较
const bool time_list(PointType &x, PointType &y)
{
    return (x.curvature < y.curvature);
}

//imu数据的初始化
ImuProcess::ImuProcess() : Eye3d(M3D::Identity()), Zero3d(0,0,0), b_first_frame(true), imu_need_init(true)
{
    init_iter_num = 1;                 //初始化迭代次数为1

    cov_acc = V3D(0.1, 0.1, 0.1);      //加速度计噪声
    cov_gyr = V3D(0.1, 0.1, 0.1);      //陀螺仪噪声
    cov_bias_gyr = V3D(0.1, 0.1, 0.1);  //陀螺仪偏置噪声
    cov_bias_acc = V3D(0.1, 0.1, 0.1);   //加速度计偏置噪声

    cov_inv_expo = 0.2;                   //控制曝光估计的权重

    mean_acc = V3D(0, 0, -1.0);            //静止时加速度初始值假设重力方向为下
    mean_gyr = V3D(0,0,0);                //静止时陀螺仪角速度初始值则为0

    angvel_last = Zero3d;                 //上一次的角速度初始化为0
    acc_s_last = Zero3d;                  //上一次的加速度初始化为0

    Lid_offset_to_IMU = Zero3d;           //lidar到imu的补偿初始化为0
    Lid_rot_to_IMU = Zero3d;              //lidar到imu的偏移外参初始化为0

    last_imu.reset(new sensor_msgs::Imu());  //对上一次的imu进行重置
    cur_pcl_un_.reset(new PointCloudXYZI);   //重置新的点云对象
}

ImuProcess::~ImuProcess() { };

void ImuProcess::Process2(LidarMeasureGroup &lidar_meas, StatesGroup &start, PointCloudXYZI::Ptr cur_pcl_un_)
{
    double t1, t2, t3;
    t1 = omp_get_wtime();

    //如果lidar序列中的点云为空，触发断言失败报错，提示数据流是否正常
    ROS::ASSERT( lidar_meas.lidar != nullptr );

    //无imu状态下直接复制当前点云，使用上一帧状态简单预测，不做严格的 IMU 去畸变
    if(!imu_en)
    {
        Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
        return;
    }

    //取最后一个measure包
    MeasureGroup meas = lidar_meas.measure.back();

    if(imu_need_init)
    {
        if(meas.imu.empty)
        {
            return;
        };

        imu_init(meas, stat, init_iter_num);

        imu_need_init = true;

        last_imu = meas.imu.back();

        if(init_iter_num > MAX_INI_COUNT)
        {
            imu_need_init = false;

            //打印初始化重力向量，重力模长，测量偏置协方差
            ROS_INFO( 
            "imu Initial : gravity: %.4f %.4f %.4f %.4f; acc covarience:" 
            "%.8f %.8f %.8f; gyr covarience: %.8f %.8f %.8f \n" ,
            stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(), cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2] 
            )

            //打印游走误差
            ROS_INFO(
            "IMU Initials : ba covarience: %.8f %.8f %.8f; bg covarience:"
            "%.8f %.8f %.8f",
            cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2];
            )
        }
    }

    UndistorPcl(lidar_meas, stat, *cur_pcl_un_)

}

void ImuProcess::Reset()
{
    ROS_WARN("Reset IMUProcess");
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    init_iter_num = 1;

    IMUpose.clear();
    last_imu.reset(new sensor_msgs::Imu());
    cur_pcl_un_.reset(new PointCloudXYZI());
}

ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N )
{
    ROS_INFO("IMU Initializing : %.1f %%", double(N) / MAX_INI_COUNT * 100);

    //定义加速度角速度临时变量
    V3D cur_acc, cur_gyr;

    if(b_first_frame)
    {
        //重置历史状态，防止污染环境
        Reset();
        N = 1;   //从当前帧开始

        //获取第一帧数据
        b_first_frame = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;

        //第一帧数据设置为平均加速度角速度
        mean_acc << imu_acc.x ,imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x ,gyr_acc.y, gyr_acc.z;
    }

    //提取序列中初始化的imu求平均值
    for(const auto &imu : meas.imu)
    {

        //取出imu的角速度和加速度
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;

        //将其转换为向量形式
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << hyr_acc.x, gyr_acc.y, gyr_acc.z;

        //求平均
        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        N++;
    }

    //求平均加速度模长
    IMU_mean_acc_norm = mean_acc.norm();  

    //使用模长初始化重力
    state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
    state_inout.rot_end = Eye3d;    //旋转初始化为单位矩阵
    state_inout.bias_g = Zero3d;    //初始化陀螺仪

    last_imu = meas.imu.back();   //保存最后一条imu数据，为了下次imu积分能接上

}
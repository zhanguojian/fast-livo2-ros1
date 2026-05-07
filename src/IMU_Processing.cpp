/*
imu初始化
*/

#include"IMU_Processing.h"
#include <iterator>
#include <type_traits>


//时间序列比较
const bool time_list(PointType &x, PointType &y)
{
    return (x.curvature < y.curvature);
}

//imu模块的初始化
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

        //imu初始化，init_iter_num是开始帧，一般是1
        imu_init(meas, stat, init_iter_num);

        //保持初始化标识
        imu_need_init = true;

        //保存最后一帧imu，保持与上次imu递推状态连续
        last_imu = meas.imu.back();

        //当初始化帧数大于最大初始化帧数退出初始化
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

    //去畸变处理
    UndistorPcl(lidar_meas, stat, *cur_pcl_un_)

}


//imu初始化
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

//imu状态清楚，防止历史数据污染当前初始化
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

//禁用imu
void ImuProcess::disable_imu()
{
    count << "IMU disabled " << endl;
    imu_en = false;
    imu_need_init = false;
}

//禁用重力在线估计功能
void ImuProcess::disable_gravity_est()
{
    count <<"online Gravity Estiamtion Disable !" <<endl;
    gravity_est_en = false;
}

//禁用imu的噪声偏差估计
void ImuProcess::disable_bias_est()
{
    count <<" bias estimation Disable !" << endl;
    ba_bg_est_en = false;
}

//禁用在线曝光估计
void ImuProcess::disable_exposure_est()
{
    count <<"online time offset estimation disable !" << endl;
    exposure_estimate_en = false;
}

//旋转矩阵设置imu到lidar的外参，    matrix.block<行数, 列数>(起始行, 起始列)
void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
    Lid_offset_to_IMU = T.block<3,1>(0,3);
    Lid_rot_to_IMU = T.block<3,3>(0,0);
}

//设置外参平移和旋转
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU = rot;
}

//设置外参平移
void ImuProcess::set_extrinsic(const V3D &transl )
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU.Identity();
}

//设置角速度协方差因子
void ImuProcess::set_gyr_cov_scale(const V3D &scaler)
{
    cov_gyr = scaler;
}

//加速度协方差因子
void ImuProcess::set_acc_cov_scale(const V3D &scaler)
{
    cov_acc = scaler;
}

//角速度偏差协方差
void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
    cov_bias_gyr = b_g;
}

//加速度偏置协方差
void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
    cov_bias_acc = b_a;
}

//逆曝光时间协方差
void ImuProcess::set_inv_expo_cov(const double &inv_expo)
{
    cov_inv_expo = inv_expo;
}

//imu初始化帧数
void ImuProcess::set_imu_init_frame_num(const int &num)
{
    MAX_INI_COUNT = num;
}


//imu异常时抛弃imu时用原点云暂时递推
void ImuProcess::Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
    //点云
    pcl_out = *(meas.lidar);

    //点云开始采样时间
    const double &pcl_beg_time = meas.lidar_frame_beg_time;
 
    //按帧中点云偏移时间排序
    sort(pcl_out.point.begin(),  pcl_out.point.end(), time_list);

    //点云结束时间
    const double &pcl_end_time = pcl_beg_time + pcl_out.point.back().curvature / double(1000);

    //这一帧结束采样的时间
    meas.last_lio_update_time = pcl_end_time;

    //最后一个点相对帧开始的时间差
    const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

    //状态
    MD(DIM_STATE,DIM_STATE) F_x, cov_w;

    double dt = 0;

    if(b_first_frame)
    {
        dt = 0.1;
        b_first_frame = false;
    }
    else
    {
     dt = pcl_beg_time - time_last_scan;
    }

    //更新上一帧开始时间
    time_last_scan = pcl_beg_time;

    //计算旋转增量
    M3D Exp_f = Exp(state_inout.bias_g, dt);

    //状态传播
    F_x.setIdentity();            //状态转移矩阵初始化为单位阵
    cov_w.setZero();              //状态协方差初始化为零矩阵

    //下一时刻姿态误差 对 上一时刻姿态误差 的传播关系。
    F_x.block<3,3>(0,0) = Exp(state_inout.bias_g, -dt);
    F_X.block<3,3>(0,10) = Eye3d * dt;
    F_x.block<3,3>(3,7) = Eye3d * dt;

    //diagonal（）表示取对角线
    cov_w.block<3,3>(10,10).diagonal() = cov_gyr * dt * dt;
    cov_w.block<3,3>(7,7).diagonal() = cov_acc * dt * dt;

    //协方差更新
    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    //旋转和位置更新
    state_inout.rot_end = state_inout.rot_end * Exp_f;
    state_inout.pos_end = state_inout.pos_end * state_inout.vel_end * dt;

    //去畸变
    if(lidar_type != L515 )
    {
        auto it_pcl = pcl_out.point.end() - 1;
        double dt_j = 0.0;
        for(;it_pcl != pcl_out.point.std::begin(); it_pcl--)
        {
            //确定偏移时间转换为秒
            dt_j = pcl_end_offset_time - it_pcl->curvature/double(1000);

            //j到k时间的相对旋转，负号表示往前递推
            M3D R_jk(Exp(state_inout.bias_g, -dt_j));

            //当前点坐标
            V3D P_j(it_pcl->x,it_pcl->y,it_pcl->z );

            //j到k时间的平移，transpose是坐标系转换到当前坐标系下
            V3D p_jk;
            p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

            //使用平移旋转确定畸变，去畸变点 = 旋转补偿 * 原始点 + 平移补偿
            V3D P_compensate = R_jk * p_j + p_jk;

            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2)
        }
    }


    void ImuProcess::UndistorPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
    {
        double t0 = omp_get_wtime();
        pcl_out.clear();

        //取当前待处理的一组
        MeasureGroup &meas = lidar_meas.measures.back();

        //将imu放进新容器并上一帧最后一个imu放入前当前imu开头递推连续
        auto v_imu = meas.imu;
        v_imu = push_front(last_imu);

        //确定时间
        const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
        const double &imu_end_time = v_imu.back()->header.stamp.toSec();
        const double prop_beg_time = last_prop_end_time;

        //最后时间取决于本次lio还是vio
        const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

        if(lidar_meas.lio_vio_flg == LIO)
        {
            pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->point.size());
            pcl_wait_proc = *(pcl_meas.pcl_proc_cur);
            lidar_meas.lidar_scan_index_now = 0;
            IMUpose.push_back(set_pose6d( 0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
        }

        //初始化传播变量
        V3D acc_imu(acc_s_last) , angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pose_imu(state_inout.pos_end);
        M3D R_imu(state_inout.rot_end);
        
        MD(DIM_STATE, DIM_STATE) F_x, cov_w;
        double dt, dt_all = 0.0;

        //offs_t代表当前积分结束点当对于prop_beg的时间
        double offs_t;

        double tau;
        
        if(!imu_time_init)
        {
            tau = 1.0;
            imu_time_init = true;
        }
        else{
            tau = state_inout.inv_expo_time;
        }

        switch(lidar_meas.lio_vio_flg)
        {
            case LIO;

            case VIO: 

              //dt表示head到tail的积分时间
              dt = 0;
              for (int i = 0; i < v_imu.size() - 1; i++)
              {
                auto head = v_imu[i];
                auto tail = v_imu[i+1];

                //跳过传播前无效数据
                if (tail->header.stamp.toSec() < prop_beg_time ) continue;

                //计算平均角速度
                angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                      0.5 * (head->angular_velocity.z + tail->angular_velocity.z)

                //计算平均加速度
                acc_avr  << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x) + 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y) , 
                      0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z)


                //setw（10）即如果实际数字长度不够 10 个字符，前面会自动补空格，让输出更整齐。  打印imu时间戳，平均加速度，平均角速度    
                fout_imu << setw(10) << head->header.stamp.toSec() - frist_lidar_time <<  " " << angvel_avr.transpose()  << "  " << acc_var.transpose() << endl;

                //去除偏置
                angvel_avr -= state_inout.bias_g;
                acc_var = acc_var * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

                //第一条数据在传播之前
                if(head->header.stamp.toSec() < prop_beg_time)
                {
                    dt = tail->header.stamp.toSec() - last_prop_end_time;
                    offs_t = tail->header.stamp.toSec() - prop_beg_time;
                }
                //当前imu数据在传播区间
                else if (i != v_imu.size() - 2)
                {

                    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
                    offs_t = tail->header.stamp.toSec() - prop_beg_time;
                }
                //当前数据在传播区间且是最后一条imu数据
                else
                {
                    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
                    offs_t = prop_end_time - prop_beg_time;
                }

                dt_all += dt;

                M3D acc_avr_skew;
                M3D Exp_f = Exp(angvel_avr, dt);
                acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

                F_x.setIdentity();
                cov_w.setZero();

                F_x.bolck<3,3>(0,0) = Exp(angvel_var, -dt);
                F_x.block<3,3>(3,7) = Eye3d * dt;

                if(ba_bg_est_en)
                {
                    F_x.block<3,3>(0,10) = -Eye3d * dt;
                    F_x.block<3,3>(7,13) = -R_imu * dt;
                }

                if(gravity_est_en)
                {
                    F_x.block<3,3>(7,16) = Eye3d * dt;
                }
                
                if(exposure_estimate_en)
                {
                    cov_w(6,6) = cov_inv_expo * dt *dt;
                }


              }
        }

    }
    





}
#include "../include/voxel_map.h"
#include <cmath>
#include <vector>


//把 LiDAR 一个点的测距误差和角度误差，传播成该点在三维坐标系下的协方差矩阵。
void calcBodyCov(Eigen::Vector3d &pb, const float &range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{

    //如果Z轴坐标为0,为了避免后续计算中除以0的情况，给Z轴坐标赋一个很小的值
    if (pb[2] == 0)  pb[2] = 0.0001;

    //点云距离
    float range = sqrt(pb[0] * pb [0] + pb[1] * pb[1] + pb[2] * pb[2]);



    //根据距离增量计算距离误差方差，比例系数由 range_inc 参数控制
    float range_var = range_inc * range_inc;

    //根据角度增量计算方向误差方差，假设角度误差与距离成正比，比例系数由 degree_inc 参数控制,direction_var是方向误差的方差矩阵，计算点云方向变化对位置不确定性的影响
    Eigen::Matrix2d direction_var;
    //pow计算一个数的幂既几次方，DEG2RAD是deg角度转rad的宏定义，这里的2维矩阵表示方向误差随距离增加而增大
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);


    //计算点的方向向量，构造一个direction向量表示点云在坐标系的方向
    Eigen::Vector3d direction(pb);
    //单位归一化方向向量，使其长度为1，表示纯粹的方向信息
    direction.normalize();



    //点方向变化对位置不确定性的影响，direction_hat是点云方向的反对称矩阵，表示点云在空间中的旋转关系，
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;



    //建立一个正交基向量和direction垂直，
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();

    //通过叉乘得到另一个与direction垂直的基向量，保证三个基向量之间相互垂直，形成一个局部坐标系，cross是向量叉乘
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();

    //构建二维基底，在激光垂直束方向上z不变XY（base_vector1和2)方向上的扰动的三维坐标
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);



    //构建雅可比矩阵A，角度误差到三维点坐标误差的传播矩阵，描述这个角度误差会让三维点偏移多少；range是点云距离把 角度误差 转成 空间位置误差。，direction_hat是点云方向的反对称矩阵，N是构建的正交基向量矩阵
    Eigen::Matrix<double, 3,2> A = range * direction_hat * N; 

    //距离方向误差 + 角度方向误差
    cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();

}

//从ros服务器加载配置
void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{

    //是否发布
    nh.param<bool>("publish/pub_plane_en" , voxel_config.is_pub_plane_map_, false);
    
    //lio配置参数
    nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
    nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
    nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold, 0.01);

    nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
    nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02);
    nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.005);

    nh.param<double>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
    nh.param<double>("lio/max_points_num", voxel_config.max_points_num_, 50);
    nh.param<double>("lio/max_iterations", voxel_config.max_iterations_, 5);

    //local_map配置参数
    nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
    nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
    nh.param<double>("local_map/sliding_thresh",voxel_config.sliding_thresh, 8);

};

//根据带有方差点云初始化平面
void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
    //初始化平面
    plane->plane_var_ = Eigen::Matrix<double, 6 ,6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->center_ = Eigen::Vector3d::Zero();
    plane->normal_ = Eigen::Vector3d::Zero();

    plane->points_size_ = points.size();
    plane->radius_ = 0;

    //遍历点云
    for(auto pv:points)
    {
        //所有点云协方差l累积
        plane->covariance_ += pv.point_w * pv.point_w.transpose();
        //所有点云总坐标
        plane->center_ += pv.point_w;
    }
    //点云中心
    plane->center_ = plane->center_ / palne->points_size_;

    //Σ=E[xxT]−μμT
    plane->covariance_ = plane->covariance_ / plane->size_ - plane->center_ * plane->center_.transpose();


    //特征值求解器 即 Σ v = λ v
    Eigen::EigenSlover<Eigen::Matrix3d> es(plane->covariance_);

    //求解出三个特征向量的特征向量矩阵
    Eigen::Matrix3cd evecs = es.eigenvectors();

    //求解出三个特征值，cd代表复数
    Eigen::Vector3cd evals = es.eigenvalues();

    //取复数的实数部分
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();

    //对特征值进行排序，找到最小，中间，最大的特征值及其对应的特征向量
    Eigen::Matrix3f::Index evalsMin, evalsMax;

    //找到下标存到索引中
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    
    //得到中间的特征值索引
    int evalsMid = 3 - evalsMin - evalsMax;

    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

    //J_Q = I / N
    Eigen::Matrix3d J_Q;

    //点坐标误差传播到平面中心误差的雅可比矩阵。用于把每个点的不确定性传播到平面中心的不确定性。点数越多对中心的不确定性越小
    J_Q << 1.0 / plane->points_size_, 0,0,0,1.0 / plane->points_size_, 0,0,0,1.0 / plane->points_size_;

    //最小特征值小于阈值则判定可以构成一个平面
    if(evlasReal(evalsMin) < planer_threshold_)
    {
        //遍历所有点云的不确定性对平面参数不确定性的贡献
        for(int i = 0; i < points.size(); i++)
        {
            //雅可比计算,每个点云的扰动是如何影响平面参数,J =
            // [ ∂normal / ∂p_i ]
            // [ ∂center / ∂p_i ]
            Eigen::Matrix<double, 6, 3> J;

            //中间矩阵,计算点坐标扰动对最小特征向量，也就是平面法向量的影响。
            Eigen::Matrix3d F;

            //遍历特征值,当点发生扰动时，法向量 evecMin 的变化只会沿着另外两个特征向量方向偏移。
            for(int m = 0; m < 3; m++)
            {
                if(m ! (int)mvalsMin)
                {
                    //特征向量扰动理论
                    Eigen::Matrix<double, 1, 3> F_m = 
                        (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) * 
                        (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                }else {
                
                    //最新特征值对应的索引行,点坐标扰动对平面参数影响小直接设置为0;
                    Eigen::Matrix<double,1,3> F_m;
                    F_m << 0,0,0;
                    F.row(m) = F_m;
                }
            }
            //J 的上 3 行：点扰动对平面法向量 normal 的影响
            J.block<3,3>(0,0) = evecs.real() * F;

            //J 的下 3 行：点扰动对平面中心 center 的影响
            J.block<3,3>(3,0) = J_Q;

            //平面参数的协方差 plane_var_ 由雅可比矩阵 J 和点的协方差 points[i].var 传播计算得到
            plane->plane_var_ += J * points[i].var * J.transpose();
        }

        //将拟合的平面参数设置

        //最小特征值对应的特征向量 作为平面法向量
        plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1,evalsMin), evecs.real()(2,evalsMin);

        //中间特征值作为平面Y轴法向量
        plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1,evalsMid), evecs.real()(2,evalsMid);

        //最大特征值作为平面X轴法向量
        plane->x_normal_ <<evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2,evalsMax);

        //保存特征值
        plane->min_eigen_value_ = evalsReal(evalsMin);
        plane->mid_eigen_value_ = evalsReal(evalsMid);
        plane->max_eigen_value_ = evalsReal(evalsMax);

        //最大特征值代表点云在最大主方向上的方差。,开方过后就是标准差尺度,用他作为平面半径
        plane->radius_ = sqrt(evalsReal(evalsMax));
        
        //计算平面参数
        plane->d_ = -(plane->norlmal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
        
        //标记有效平面
        plane->is_plane_ = true;

        //标记已经更新
        plane->is_update_ = true;

        //如果平面未初始化
        if(!plane->is_init)
        {
            //给新平面分配一个唯一id
            plane->id = voxel_plane_id;
            voxel_plane_id++;
            plane->is_init_ = true;
        }

    }
    else {
      plane->is_update_ = true;
      plane->is_plane_ = false;
    }

}

//初始户八叉树,点云数超过拟合点云阈值则切分,切分后不是平面则继续切分,是平面则标记平面,如果点云数超过最大点云数则禁止更新并清空点云
void VoxelOctoTree::init_octo_tree()
{
    //节点中点云暂存数超过阈值
    if(temp_points_.size() > points_size_threshold)
    {
        //对当前节点进行拟合
        init_plane(temp_points_, plane_ptr_);

        //拟合平面标记为真
        if(plane_ptr_->is_plane_ == true)
        {
            //八叉树状态标记叶子节点
            octo_state_ = 0;

            //当前平面点数超过最大点数阈值
            if(temp_points_.size() > max_points_num_)
            {
                //禁用更新并清空点云
                update_enable_ = false;
                std::vector<pointWithVar>().swap(temp_points_);
                new_points_ = 0;
            }
        }
        else{

            //将当前节点状态为非平面继续切分
            octo_state_ = 1;
            cut_octo_tree();
        }
        //标记初始化
        init_octo_ = true;
        new_points  = 0;
    }
}


void VoxelOctoTree::cut_octo_tree()
{
    //切分层数大于最大层则标记叶子平面节点退出停止切分
    if(layer_ >= max_layer_)
    {
        //标记叶子节点
        octe_state_ = 0;
        return;
    }

    //遍历点云分配到叶子节点中
    for(size_t i = 0; i < temp_points_.size(); i++)
    {
        //标记在体素中8个叶子节点中的位置
        int xyz[3] = {0,0,0};

        //xyz[0]、xyz[1]、xyz[2] 分别表示点在 x左右、y前后、z上下 方向上的位置，初始值为 0
        if(temp_points_[i].point_w[0] > voxel_center_[0]){ xyz[0] = 1;}
        if(temp_points_[i].point_w[1] > voxel_center_[1]){ xyz[1] = 1;}
        if(temp_points_[i].point_w[2] > voxel_center_[2]){ xyz[2] = 1;}
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        //如果子结点不存在则创建新的子节点
        if(leaves_[leafnum] == nullptr)
        {
            leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1 , layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;

            //子节点中心的位置
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            
        }

        //将当前点加入子节点中
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_++;
    }

    //对新创建的叶子节点遍历进行平面拟合
    for(uint i = 0; i < 8; i++)
    {
        //子节点存在不为空
        if(leaves_[i] != nullptr)
        {
            //子节点大于可以平面拟合数
            if(leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold)
            {
                //子节点点云进行平面拟合
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_)

                //子节点标记为平面
                if(leaves_[i]->plane_ptr_->is_plane_)
                {
                    //标记为叶子节点
                    leaves_[i]->octo_state_ = 0;

                    //子结点超过最大点数阈值
                    if(leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
                    {
                        //子结点不再更新且清空点云
                        leaves_[i]->update_enable_ = false; 
                        std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
                        new_points_ = 0;
                    }
                }
                else{
                    //继续切分
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }

                //标记已初始化
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_ = 0;
            }

        }
    }
}

//当新帧加入时更新节点
void VoxelOctoTree::updateOctoTree(const pointwithvar &pv)
{
    //是否初始化节点
    if(!init_octo_)
    {
        //将新增点数放入临时容器，标记点数更新
        new_points++;
        temp_points_.push(pv);

        //暂存点数大于初始化点数阈值
        if(temp_points_.size() > points_size_threshold)
        {
            //初始化节点
            init_octo_tree();
        }
    }
    else{

        //该节点标记为平面
        if(plane_ptr_->is_plane_)
        {
            //更新为true
            if(update_enable_)
            {
                //新增点数放入临时容器，更新点数标记
                new_points_++;
                temp_points_.push(pv);

                //如果新增点数大于更新阈值
                if(new_points_ >update_size_threshold_)
                {
                    //初始化平面，新增点数标记为0
                    init_plane(temp_points_, plane_ptr_)
                    new_points_ = 0;
                }

                //如果暂存点数超过最大点数
                if(temp_points_.size() >= max_points_num_)
                {
                    //标记更新且清空点云
                    update_enable_ = false;
                    std::vector<pointWithVar>().swap(temp_points_);
                    new_points_ = 0;
                }

            }
        }else
        {
            //如果当前层数小于最大层数
            if(layer_ < max_layer_)
            {
                //确定新点的体素索引
                int xyz[3] = {0, 0, 0};
                if(pv.point_w[0] > voxel_center_[0]){xyz[0] = 1;}
                if(pv.point_w[1] > voxel_center_[1]){xyz[1] = 1;}
                if(pv.point_w[2] > voxel_center_[2]){xyz[2] = 1;}
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

                //如果叶子节点不为空
                if(leaves_[leafnum] != nullptr)
                {
                    //更新该节点
                    leaves_[leafnum]->updateOctoTree(pv);
                }else{
                    //创建新的节点
                    leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num[layer_ + 1], max_points_num_, planer_threshold_);
                    leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->updateOctoTree(pv);
                }
            }
            else
            {
                //如果达到最大层数且允许更新
                if(update_enable_)
                {
                    //新增点数更新却放入临时容器
                    new_points_++;
                    temp_points_.push_back(pv);

                    //新增点大于更新阈值
                    if(new_points_ > update_size_threshold)
                    {
                        //初始化平面
                        init_plane(temp_points_, plane_ptr_);
                        new_points_ = 0;
                    }

                    //如果临时容器点数大于最大点数
                    if(temp_points_.size() > max_points_num)
                    {
                        //禁止更新且清空容器
                        update_enable_ = false;
                        std::vector<pointWithVar>().swap(temp_points_);
                        new_points_ = 0;
                    }
                }
            }
        }
    }
}

//给定一个世界坐标点 pw，从当前八叉树节点开始，沿着八叉树往下找，找到这个点最终对应的叶子节点或平面节点
VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)  //返回一个八叉树节点
{
    //如果节点没有初始化， 该节点是平面，或者层数超过最大层，返回该节点
    if(!init_octo_ || plane_ptr_->is_plane_ || (layer_ > max_layer_))
    {
        return  this;
    }

    //确定体素索引
    int xyz[3] = {0, 0, 0};
    xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    //叶子节点存在则继续调用该函数，否则返回该节点
    return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;

}


//lidar点云转换为目标坐标系
void VoxelMapManager::TransfromLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::ptr &input_cloud, pcl::PointCloud<pcl::PointXYZI>::ptr &trans_cloud)
{
    //清空并预分配输出点云
    pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
    trans_cloud->reserve(input_cloud->size());

    //遍历输入点云
    for(size_t i = 0; i < input_cloud->size(); i++)
    {
        //取出点云并拷贝给法向量点云
        pcl::PointXYZINormal p_c = input_cloud->points[i];

        //将坐标转换为向量坐标
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);

        //坐标系转换
        p = (rot *(extR_ * p + extT_) + t);

        //创建新点云容器并赋值
        pcl::PointXYZI pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;

        //将新点云存入输出点云
        trans_cloud->points.push_back(pi);
    }
}


//
void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{

    //清空和与预分配反对称矩阵和协方差矩阵容器
    cross_mat_list_.clear();
    cross_mat_list_.reserve(feats_down_size_);
    body_cov_list_.clear();
    body_cov_list_.reserve(feats_down_size_);

    //计算每个点的协方差和反对称矩阵放入容器
    for(size_t i = 0; i < feats_down_body_->size(); i++)
    {
        //取出点的坐标并转换为向量格式
        V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z):

        //如果z轴坐标为0赋值避免0的计算
        if(point_this[2] == 0)
        {
            point_this[2] == 0.001;
        }

        //计算协方差
        M3D var;
        calcBodyCov(points_this, config_setting_.dept_err, config_setting_.beam_err, var);

        //存储点的协方差
        body_cov_list_.push_back(var);

        //lidar坐标系转换为imu坐标系
        point_this = extR_ * point_this + extT_;

        //点的反对称矩阵
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        //存储反对称矩阵
        cross_mat_list_.push_back(point_crossmat);
    }

    //清空并预分配pv_list的大小，用来存储带协方差的点
    vector<pointwithvar>().swap(pv_list);
    pv_list.reserve(feats_down_size_);

    int rematch_num = 0;
    MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;

    G.setZero();    //卡尔曼增益矩阵
    H_T_H.setZero();   //测量雅可比的转置与协方差矩阵的乘积
    I_STATE.setZero();   //单位矩阵

    bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
    for(int iterCount = 0; iterCount < config_setting_.max_interations_; iterCount++)
    {
        //总残差的时间初始设置为0
        double total_residual = 0.0;

        //世界地图
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);

        //坐标系转换
        TransfromLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar)

        //旋转部分的协方差矩阵 rot_var 从状态协方差矩阵 state_.cov 中提取出来，表示旋转部分的不确定性
        M3D rot_var = state_.cov.block<3,3>(0,0);

        //平移部分的协方差矩阵 t_var 从状态协方差矩阵 state_.cov 中提取出来，表示平移部分的不确定性
        M3D t_var = state_.cov.bolck<3,3>(3,3);

        //遍历降采样后的点云
        for(size_t i = 0; i < feats_down_body_->size(); i++)
        {

            //取出第i个pv_list的引用
            pointwithvar &pv = pv_list_[i];  

            //机体坐标系下点云转换到pv_list
            pv.point_b << feats_down_body_->point[i].x, feats_down_body_->point[i].y, feats_down_body_->point[i].z;

            //世界坐标系下点云转换到pv_list
            pv.point_w << world_lidar->point[i].x, world_lidar[i].point[i].y, world_lidar[i].point[i].z;

            //取出第 i 个点在 body 系下的协方差矩阵。
            M3D cov = body_cov_list_[i];

            //取出第 i 个点对应的反对称矩阵，也就是叉乘矩阵。
            M3D point_crossmat = cross_mat_list_[i];

            //把点的协方差从 body 系传播到 world 系，并加入位姿不确定性
            cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;

            //把计算后的世界系协方差保存到 pv.var。
            pv.var = cov;

            //同时保存 body 系下原始协方差。
            pv.body_var = body_cov_list_[i];
        }
        ptpl_list_.clear();

        BuildResidualListOMP(pv_list, ptpl_list);

        for(int i = 0; i < ptpl_list_.size(); i++)
        {
            total_residual += fabs(ptpl_list_[i].dis_to_plane_);
        }

        //
        effect_feat_num_ = ptpl_list_.size();
        cout << "[LIO] Raw feature num:" << feats_undistort_->size() << "downsampled feature num:" << feats_down_size_ 
             << "effecture feature num:" << effect_feat_num_ << " average residual:" << total_residual / effect_feat_num_ << endl;


    }

    //
    void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
    {
        //获取最大层数
        int max_layer = config_setting.max_layer_;

        //获取最大体素大小
        double voxel_size = config_setting_.max_voxel_size_;

        //标准差阈值
        double sigma_num = config_setting_.sigma_num_;
        std::mutex mylock();

        //初始化输出结果
        ptpl_list.clear();

        //存储计算得到的残差
        std::vector<PointToPlane> all_ptpl_list(pv_list.size());

        //标记成功计算了残差的点
        std::vector<bool> useful_ptpl(pv_list.size());

        //记录点的索引
        std::vector<size_t> index(pv_list.size());

        //初始化有用点标记
        for(size_t i = 0; i < index.size(); ++i)
        {
            index[i] = i;
            useful_ptpl[i] = false;
        }

        #ifdef MP_EN
         omp_set_num_threads(MP_PROC_NUM);
         #pragma omp parallel for
        #endif

        //遍历每个点云
        for(int i = 0; i < index.size(); i++)
        {
            //取出点云引用
            pointWithVar &pv = pv_list[i];
            float loc_xyz[3];

            //计算体素位置得到体素网格坐标
            for(int j = 0; j < 3; j++)
            {
                loc_xyz[j] = pv.point_w[i] / voxel_size;
                if(loc_xyz[j] < 0) 
                {
                    loc_xyz[j] -= 1.0;
                }
            }

            //构造体素
            VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

            //在体素坐标中查找这个体素
            auto iter = voxel_map_.find(position);


            if(iter != voxel_map_.end())
            {
                //取出对应的八叉树
                VoxelOctoTree *current_octo = iter->second;

                //定义一个点到平面残差对象
                PointToPlane single_ptpl;
                
                //定义一个是否成功标识
                bool is_sucess = false;

                //定义一个概率值
                double prob = 0;

                //构建残差
                build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);

                //如果这个点的残差计算失败，尝试在邻近体素重新计算残差
                if(!is_sucess)
                {
                    //标记体素位置
                    VOXEL_LOCATION near_position = position;

                    if(loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_))
                    {
                        near_position.x = near_position + 1;
                    }
                    else if (loc_xyz[1] < (current_octo->voxel_center_[0] - current_octo->quater_length_))
                    {
                        near_position.x = near_position - 1;
                    }

                    if(loc_xyz[0] > (current_octo->voxel_center_[1] + current_octo->quater_length_))
                    {
                        near_position.y = near_position + 1;
                    }
                    else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_))
                    {
                        near_position.y = near_position - 1;
                    }
  
                    if(loc_xyz[0] > (current_octo->voxel_center_[2] + current_octo->quater_length_))
                    {
                        near_position.z = near_position + 1;
                    }
                    else if (loc_xyz[1] < (current_octo->voxel_center_[2] - current_octo->quater_length_))
                    {
                        near_position.x = near_position - 1;
                    }

                    auto iter_near = voxel_map_.find(near_position);
                    if(iter_near != voxel_map_.end())
                    {
                        build_single_residual(pv,iter_near->second, 0, is_sucess, prob, single_ptpl;)
                    }
                }
                if(is_sucess)
                {
                    mylock.lock();
                    useful_ptpl[i] = true;
                    all_ptpl_list[i] = single_ptpl;
                    mylock.unlock();
                }
                else
                {
                    mylock.lock();
                    useful_ptpl[i] = false;
                    mylock.unlock();
                    
                }
            }
        }
        for(size_t i = 0; i < useful_ptpl.size(); i++)
        {
            ////在并行部分结束后，将 所有成功计算的 PointToPlane 结果 存入 ptpl_list
            if(useful_ptpl[i])
            {
                ptpl_list.push_back(all_ptpl_list[i]);
            }
        }

    }

    //该函数用于计算给单个定点 pv 相对于 当前体素 current_octo 的点到平面（Point-to-Plane）残差，并更新 single_ptpl 结构
    void VoxelMapManager::build_single_residual(pointwithvar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob, PointToPlane &single_ptpl)
    {
        //最大层数
        int max_layer = config_setting.max_layer;

        //标准差权重
        double sigma_num = config_setting.sigma_num_;

        //范围判定因子
        double radius_ = 3;

        //世界坐标系点转为向量形式
        Eigen::Vector3d p_w = pv.point_w;

        //当前八叉树是否是平面
        if(current_octo->plane_ptr_->is_plane_)
        {
            //取出当前平面
            Voxelplane &plane = *current_octo->plane_ptr_;

            //当前点到平面中心差
            Eigen::Vector3d p_world_to_center = p_w - plane.center_;

            //计算点到平面的距离
            float dis_to_plane_ = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);

            //点到平面中心的距离
            float dis_to_center = (plane.center_(0) - p_w(0) )  * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) + (plane.center_(2) - p_w(2)) * ( plane.center_(2) - p_w(2));

            //计算点到平面中心的投影半径，根据勾股定理，点到平面中心和点到平面距离
            float range_dis = sqrt(dis_to_center - dis_to_plane_ * dis_to_plane_);

            //判断投影半径是否在平面允许的范围内， radius_k是放大系数
            if(range_dis <= radius_k * plane.radius_)
            {
                //构造 点到平面残差对平面参数的雅可比矩阵，残差r = n^T (p - q)，J_nq = ∂r / ∂[n, q]
                Eigen::Matrix<double,1,6> J_nq;

                //对法向量 normal 的导数，残差对法向量的敏感程度
                J_nq.block<1,3>(0,0) = p_w - plane.center_;

                //对平面中心 center 的导数
                J_nq.block<1,3>(0,3) = -plane.normal_;

                //点到平面残差由于平面不确定性带来的方差，可以用误差传播：
                double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();

                //如果点到平面的距离小于若干倍标准差，就认为这个点和平面匹配。
                if(dis_to_plane < sigma_num * sqrt(sigma_l))
                {
                    //标记找到一个可以匹配的平面
                    is_sucess = true;

                    //根据点到平面距离和残差方差，计算当前匹配的概率/评分。   标准高斯概率形式 prob ∝ exp(-0.5 * d² / σ²) / sqrt(σ²)
                    double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);

                    // 选择最优平面
                    if(this_prob > prob)
                    
                        //保存当前最优匹配
                        prob = this_prob;

                        //保存平面法向量到点结构
                        pv.normal = plane.normal_;

                        //保存点的机体系协方差  
                        single_ptpl.body_cov_ = pv.body_var;

                        //保存点在 body 系和 world 系下的位置
                        single_ptpl.point_b = pv.point_b;
                        single_ptpl.point_w = pv.point_w;

                        //保存平面参数协方差
                        single_ptpl.plane_var_ = plane.plane_var_;
                        
                        //保存平面法向量和中心
                        single_ptpl.normal_ = plane.normal_;
                        single_ptpl.center_ = plane.center_;

                        //保存平面方程参数 d
                        single_ptpl.d_ = plane.d_;

                        //保存当前平面所在层级
                        single_ptpl.layer_ = current_layer;

                        //保存最终点到平面距离
                        single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
                    }

            }

            return;
        }
        else
        {
            ////递归搜索八叉树的子节点，直到达到 max_layer
            if(current_layer < max_layer)
            {
                for(size_t leafnum = 0; leafnum < 8; leafnum++)
                {
                    if(current_octo->leaves_[leafnum] != nullptr)
                    {
                        VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
                        build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
                    }
                }
            
            } 
            return；   
        }

    }
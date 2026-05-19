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

void VoxelOctoTree::updateOctoTree(const pointwithvar &pv)
{
    if(!init_octo_)
    {
        new_points++;
        temp_points_.push(pv);
        if(temp_points_.size() > points_size_threshold)
        {
            init_octo_tree();
        }
    }
    else(
        
    )
}
/*

*/

#include "../include/preprocess.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <sys/types.h>
#include <type_traits>
#include <utility>
#include <vector>

#define RETURN0 0x00
#define RETURN0AND1 0x01

Preprocess::Preprocess() :   feature_enable(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
    inf_bound = 10;  //用于判断边缘的上限
    N_SCANS = 6;   //激光雷达的线数
    group_size = 8;  //每组点的数量下限
    disA = 0.01;    //每组点最近距离阈值
    disB = 0.1;    //每组点的最远距离
    p2l_ratio = 225  //点到线距离比，用来判断平面点的

    limit_maxmid = 6.25;  //组内头部与中间的距离差比上限
    limit_midmin = 6.25;  
    limit_maxmin = 3.24;

    jump_up_limit = 170.0; //用于判断射线方向与邻点方向的最大夹角
    jump_down_limit =8.0;
    cos160 = 160;          //两个邻点向量之间的最大夹角
    
    edgea = 2;        //距离差倍数，用于判断边缘跳跃点
    edgeb = 0.1 

    smallp_intersect = 172.5;
    smallp_ratio = 1.2;
    given_offset_time = false;

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    cos160 = cos(cos160 / 180 * M_PI);
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {};

void Preprocess::set(bool feat_en , int lid_type, double bld, int pfilt_num)
{
    feature_enable = feat_en;
    lidar_type = lid_type;
    blind = bld;
    point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::Custom::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
    avia_handler(msg);
    *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
    switch (lidar_type)
    {
        case OUST64:
          ouster_handler(const int &msg);
          break;

        case VELO16:
          velodyne_handler(const int &msg);
          break;

        default:
          printf("error Lidar Type")
    }
    *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    //初始化清空容器
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    double t1 = omp_get_wtime(); //用于记录当前时间；

    int plsize = msg->point_num;
    printf("[Preprocess] input point number : %d \n", plsize);

    //与分配内存
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_surf.reserve(plsize);

    for( int i = 0; i < N_SCANS; i++)
    {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    uint valid_num = 0;


    if (feature_enabled)
    {
        for (uint i = 1; i < plsize; i++)
        {
            if ( (msg->point[i].line < N_SCANS) && (msg->point.tag & 0x30) == 0x10)
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->point[i].offset_time / float(1000000);  //微秒转成秒

                //过滤重复点
                bool is_new = false;
                if ((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) 
                 || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)  )
                {
                    pl_buff[msg->points[i].line].push_babk(pl_full[i]); 
                }
            }
        }
        
        //对扫描线点云进行特征提取
        static int count = 0;  
        static double time = 0.0;
        count++;
        double t0 = omp_get_wtime();

        for (int j = 0, j < N_SCANS, j++)
        {
            if (pl_buff[j].size() <= 5) continue;
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            plsize = pl.size();
            vector<orgtype> &types = typess[j];   //给typess取别名types ， 扫描线上点的分类和原始几何信息
            types.clear();
            types.resize(plsize);
            plsize--;
            for(uint i = 0; i < plsize ; i++)
            {
                types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
                vx = pl[i].x - pl.[i + 1].x;
                vy = pl[i].y - pl.[i++ 1].y;
                vz = pl[i].z - pl.[i + 1].z;
            };
           types[plsize].range = pl[plsize].x * pl[plsize] + pl[plsize].y * pl[plsize].y;
           give_feature(PointCloudXYZI &pl, int &types)
        }
         time += omp_get_wtime() - t0;
         printf("Feature extraction time : %lf \n", time / count) ;
    
    }
    else {
        for( uint i = 0; i < plsize ; i++)
        {
            if(msg->point[i].line < N_SCANS)
            {
                valid_num++;

                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].y = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

                if(i == 0)  
                  pl_full[i].curvature = fabs(pl_full[i].curvature) < 1.0 ? pl_full[i].curvature : 0.0;  //离异值判断，时间偏移小于1s

                else
                {
                    pl_full[i].curvature = fabs(pl_full[i].curvature - pl_full[i-1].curvature < 1.0 ?
                     pl_full[i].curvature = : pl_full[i-1].curvature + 0.004166667f;  
                     )
                }
                
                if (valid_num % point_filter_num == 0)
                {
                    if(pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full.y + pl_full.z * pl_full.z >= blind_sqr)
                    {
                        pl_surf.push_babk(pl_full[i]);
                    }
                }
            }
        }
    }

    printf("[ Preprocess ] Output tf point number : %zu \n ",pl_surf.points.size());
}

void Preprocess::give_feature(pcl::PointCloud<PointType &pl, vector<orgtype> &types)
{
    int plsize = pl.size();
    int plsize2;               //去除掉计算平面组后的点云，防止后续平面被之前的影响

    if (plsize == 0)
    {
        printf("something error \n");
        return ;
    }
    uint head = 0;

    while (type[head.range] < blind_sqr)
    {
        head++;
    }
 
    //surf
    plsize2 = (plsize > group_size ) ? ( plsize - group_size) : 0 ;  //group是用于判断平面使用的一组来连续点
    
    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());   //初始化当前平面方向和下一个平面方向，用三维向量表示
    Eigen::vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;    // i_nex平面判断结束索引 i2 当前索引
    uint last_i = 0;       //上一段平面的起点
    uint last_i_nex = 0;   //上一段平面的终点
    int last_state = 0;    //上一轮是否检测到平面
    int plane_type;        //保存 plane_judge的结果


    //沿扫瞄线判断平面
    for (uint i = head; i < plsize2; i++)
    {
        if (type[i].range < blind_sqr) 
        {
            continue;
        };

        i2 = i;

        plane_type = plane_judge( pl, types, i, i_nex, current_direct);

        if(plane_type)
        {
            for (uint j = i, j <= i_nex; j++)   //标记平面段中的点
            {
                if(j != i; i != i_nex;)
                {
                    types[j]ftype = real_plane;
                }
                else{
                    types.[j].ftype = Poss_plane;
                }
            
            }

            if (last_state == 1 && last_direct.norm() > 0.1)
            {
                double mod = last_direct.transpose * curr_direct;
            }
        }

    }
}
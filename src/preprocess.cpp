/*

*/

#include "../include/preprocess.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <ratio>
#include <stdexcept>
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
    disA = 0.01;    //平面点基础测量距离增长阈值系数
    disB = 0.1;    //平面点基础测量长度，用于计算平面每组允许的最远距离
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

        if(plane_type == 1)
        {
            for (uint j = i, j <= i_nex; j++)   //标记平面段中的点
            {

                //中间点判定为平面点
                if(j != i; i != i_nex;)
                {
                    types[j]ftype = real_plane;
                }
                //两端点判定为可能平面点
                else{
                    types.[j].ftype = Poss_plane;
                }
            
            }

            //判断当前平面与上一段平面是否明显转折
            if (last_state == 1 && last_direct.norm() > 0.1)
            {
                //方向点乘看两向量方向像不像，mod = cos(theta)，0.707是夹角45度，负的是135度
                double mod = last_direct.transpose * curr_direct;
                if ( mod > -0.707 && mod < 0.707 )
                {
                    types[i].ftype = Edge_Plane;
                }
                else
                {
                    types[i].ftype = real_plane;
                }
            }

            i = i_nex - 1;   //把当前平面最后点赋给当前点
            last_state = 1;   //当前平面赋给上一平面状态
        }
        else
        {
            i = i_nex;
            last_state = 0;
        }


        //当前平面状态赋值给上平面状态更新
        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;

    }

    /*平面点提取之后，继续从剩余点里找边缘点、跳变点、细小平面点，并最终把点放进 pl_surf 和 pl_corn。
    1. 检测 Edge_Jump / Wire
    2. 补充小平面点 Real_Plane
    3. 根据 ftype 输出 pl_surf 和 pl_corn.

    所以这一段主要处理的是还没有明确分类的普通点。
    */
    plsize2 = plsize > 3 ? plsize - 3 : 0 ;
    for (uint i = head + 3; i < plsize2; i++)
    {

        //过滤盲点和已经判定为平面点
        if (types[i].range < blind_sqr || types[i].ftype[i].ftype >= real_plane)
        {
            continue;
        }

        //过滤上一个点和当前点距离太小点
        if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
        {
            continue;
        }

        //当前点的与原点的向量，  前后点与该点的向量容器
        Eigen::Vector vec_a(pl[i].x , pl[i].y, pl[i].z);
        Eigen::Vector vecs[2];

        for (int j = 0; j < 2; j++)
        {
        
            //用j循环条件设置前后点
            int m = -1;
            if(f ==1)
            {
                m = 1;
            }

            //是在相邻点无效/在盲区时，判断当前点旁边的断裂到底属于“远处无穷跳变”还是“近处盲区跳变”。
            if(types[i + m].range < blind_sqr)
            {
                //超过初始化inf_bound参数阈值判定打到物体边界或者很远地方
                if (types.range > inf_bound)
                {
                    types[i].edj[j] = Nr_inf;
                }else   //近距离盲区
                {
                    types[i].edj[j] = Nr_blind;
                }
                continue;
            }

            vecs[j] = Eigen::vector3d(pl[i + m].x, pl[i+m].y, pl[i+m].z);
            vecs[j] = vecs[j] - vec_a;

            //dot对应公式点乘，norm即模，cosθ=a⋅b​ | ∥a∥∥b∥
            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();

            //余弦值很小，说明下一个点的方向与该点直线180度相反，发生近距离跳变
            if(type[i].angle[j] < jump_up_limit)
            {
                types[i].edg[j] = Nr_180;
            }
            //余弦值很大，说明下一个点的方向与该点相同，发生远距离跳变
            else if(types[i].angle[j] > jump_down_limit)
            {
                types[i].edg[j] = Nr_zero;
            }
        }

        //根据当前点 i 左右两侧的跳变关系，判断当前点是深度跳变边缘点还是wire点 ,如果三个点在同一条直线上，那么夹角180,值为-1,用cos160来判断是否有明显折角
        types[i].intersect = vecs[Prev].dot(vecs[Next]) /  vecs[Prev].norm() / vecs[Next].norm();

        //情况一：左边正常，右边向远处跳变  ， 右边距离大于0.15m  且   大于左边距离的两倍
        if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
        {
            if (types[i].intersect > cos160)
            {
                if (Edge_Jump_judge(pl, types, Prev))
                {
                    types[i].ftype = Edge_Jump;
                }

            }   
            //情况二：左边向远处跳变，右边正常
        }else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] ==Nr_nor && types[i].diata > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
        {
            if (types[i].intersect > cos160)
            {
                if (Edge_Jump_judge(pl, types, i, Next))
                {
                    types[i].ftype = Edge_Jump;
                }
            }
            //情况三：左边正常，右边无穷远/无有效点
        }else if (types[i].edg[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
        {
           if (Edge_Jump_judge(pl, types, i, Prev))
           {
            types[i].ftype = Edge_Jump;
           }
           //情况四：左边无穷远/无有效点，右边正常
        }else if (types[i].edj[pre] == Nr_inf && types[i].edj[Next] == Nr_nor)
        {
            if (Edge_Jump_judge(pl, types, i, Next))
            //情况五；左右两边都异常，标记为 Wire
        }else if (types[i].edj[Prev] > Nr_nor && types[i] > Nr_nor)
        {
            if (types[i].ftype == Nor)
            {
                types[i].ftype = wire;
            }
        }

        //补充识别小平面点,这里用当前点和左右邻居三个点，再做一次简单判断，把它们补成 Real_Plane。
        plsize2 = plsize -1;
        double ratio;
        for (uint i = head + 1; i < plsize2; i++)
        {
            //跳过无效点
            if(types[i].range < blind_sqr || types[i - 1].range < blind_sqr || types[i + 1].range < blind_sqr)
            {
                continue;
            }

            if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
            {
                continue;
            }
        

            //只处理还没有分类的普通点
            if(types[i].ftype == Nor)
            {
                //计算左右距离比例
                if(types[i - 1].dista > types[i].dista  )
                {
                    ratio = types[i -1].dista / types[i].dista;
                }else
                {
                    ratio = types.[i].dista / types[i -1].dista;
                }

                if(types[i].intersect < smallp_intersect && ratio < ratio)
                {
                    if(types[i - 1].ftypes == Nor ){
                        types[i - 1].ftype = real_plane;
                    }
                    if(types[i + 1].ftype == Nor){
                        types[i +1].ftype == real_plane;
                    }
               //连续面点还没达到 point_filter_num 个，就遇到了非平面点
                    types[i].ftype = real_plane;
                }
            }
        }
    }


    //根据前面已经打好的特征标签 ftype，把面点放进 pl_surf，把边缘点放进 pl_corn。
    int  last_surface = -1;
    for( uint j = head; j < plsize; j++)
    {
        if (types[j].ftypes == Poss_plane || types[j].ftype == Real_plane)
        {
            //用来记录一段连续平面点的起始位置。-1代表还没有记录平面点段，j等于n就代表从n开始出现连续面点
            if(last_surface == -1)
            {
                last_surface = j;
            }

            //这里按数量间隔取样，不是普通间隔取羊，减少面点匹配，因为前面已经参与平面判断，
            if( j =uint(last_surface + point_filter_num -1))
            {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.curvature = pl[j].curvature;
                pl_surf.push_babk(ap);

                last_surface = -1;
            }
        }    
        else
        {
            if(types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane )
            {
                pl_corn.push_back(pl[j]);
            }

            //连续面点还没达到 point_filter_num 个，就遇到了非平面点,要做平均
            if( last_surface != -1)
            {
                PointType ap;
                for( uint k = last_surface; k < j; k++)
                {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.curvature += ap.curvature;
                }

                //求平均后的这段平面的几何质点
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                sp.z /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }

            last_surface = -1;
        }
        
    }


    
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
    pl.height = 1;
    pl.witdth = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl,output);
    output.header.frame_id = "livox";
    output.header.stamp = ct;

}

//在一条 LiDAR 扫描线上判断一段点是否足够连续、细长、平滑。通过这种方式找出面点候选。
//return 2 是遇到盲区点，无法判断， 1 是平面有效段， 0 不是平面有效段
void Preprocess::plane_judge(const PointCloudXYZI &pl,  vector<orgtype> &types, uint i_cur, uint i_nex, Eigen::Vector3d current_direct)
{
    //点组中空间跨度阈值设定
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;

    double two_dis;
    vector<double> disarr;   //保存一组距离后面用来判断平面
    disarr.reserve(20);

    for(i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
    {
        if(types[i].range < blind_sqr)
        {
            current_direct.setZero();
            retrun 2;
        }
        disarr.push_babk(types[i_nex].dista)
    }

    for(;;)
    {
        if((i_cur >= pl.size) || (i_nex >= pl.size()))  break;
        
        if(types[i_nex].range < blind_sqr)
        {
            current_direct.setZero();
            return 2;
        };
        //起点到终点向量
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx*vx + vy*vy + vz*vz;

        //循环退出条件
        if(two_dis >= group_dis) {break;}
        
        disarr.push_babk(types[i_nex].dista);
        i_nex++
    }

    double leng_wid = 0;    //记录点组中每个点向量与起始向量的最大偏离距离
    double v1[3], v2[3];    //v1是点组中点与起始点的向量，v2是叉乘

    for (uint j = i_cur + 1; j < i_nex ; j++)
    {
        if((j >= pl.size()) || (i_cur >= pl.size()))  break;

        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        //求该向量与起始向量的叉乘
        v2[0] = v1[1] * vz - vy * v1[2];
        v2[1] = v1[2] * vx - v1[0] * vz;
        v2[2] = v1[0] * vy - vx * v1[1];

        double lw = v2[0] * v2[0] + v[1] * v[1] + v[2] * v[2];  //叉乘的模代表两向量的偏离程度，几何意义是向量面积投影

        if ( lw > leng_wid) { leng_wid = lw};    

    }
   
    if((two_dis * two_dis / leng_wid) < p2l_ratio)   //宽度偏离测量，leng_wid 为|V|平方 * d 的平方,d是向量到起始向量V的垂直距离
    {
        current_direct.setZero();
        return 0;
    }


    //对相邻点距离进行从大到小排序
    uint disarrsize = disarr.size();
    for(uint j = 0; j < disarrsize - 1; j++)
    {
        for (uint k = j +1 ;k < disarrsize; k++)
        {
            if(disarr[i] < disarr[k])
            {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid;
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16)
    {
        current_direct.setZero();
        return 0;
    }

    //用距离比例排除跳变边缘
    if(lidar_type = AVIA)
    {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if(dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
        {
            current_direct.setZero();
            return 0;
        } 
    }
    else {
        double dismax_mid = disarr[0] / disarr[disarrsize -2];
        if(dismax_mid >= limit_maxmid)
        {
            current_direct.setZero();
            return 0;
        }
    }

    current_direct << vx, vy, vz;
    current_direct.normalize();     //向量单位归一化
    return 1;
}
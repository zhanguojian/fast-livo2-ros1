 

#pragma once

#include "common_lib.h"
#include "utils/types.h"
#include <atomic>
#include <exception>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>
#include <stdexcept>
#include <sys/types.h>
#include <vector>

using namespace std;

#deifne IS_VALID(a) (abs(a) > 1e8 ? true : false)

//lidar可以被提取的特征
enum LiDARFeature
{
  Nor = 1,   //普通点
  Poss_plane,   //可能是平面
  real_plane,    //真实平面
  Edge_Jump,   //跳变边缘点
  Edge_Plane,   //边缘平面
  Wire,         //线状特征
  ZeroPoint     //无效点
};

//点的前后周围关系
enum Surround
{
    Prev,     //扫描线上前一个点
    Next,     //扫描线上下一个点
};


//点与相邻点E跳变关系类型
enum Edge_Jump
{
    Nr_nor,    //正常关系
    Nr_zero,    //相邻点是零点或者存在零点  某个方向没有回波被驱动默认设置可能出现零点，一般是自身lidar坐标系原点
    Nr_180,      //几何关系异常，与lidar形成方向几乎相反
    Nr_inf,     //距离突变，有遮挡之类的
    Nr_blind    //点离太近认为盲区点
};


//点的原始几何属性和分类结果
struct orgtype
{
    double range;  //点到lidar的距离
    double dista;    //点和前一个点的距离
    double angle[2];   //点和前一个点的夹角，angle[0]是水平夹角，angle[1]是垂直夹角
    double intersect;   //点和前一个点方向向量的的几何角度指标，0表示与前一个点在同一平面上，1表示与前一个点在不同平面上，局部几何一致性判断

    Edge_Jump edj[2];
    LiDARFeature ftype;

    orgtype()
    {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor ;

        intersect = 2;   //cos之为-1到1,2是不可能的值初始化标识还没有赋值

    }
}

/*

定义各个lidar点云点的类型

*/

//Velodyne 定义 velodyne_ros::Point 一个点的类型

namespace velodyne_ros 
{
    struct EIGEN_ALIGN16 Point    //EIGEN_ALIGN16表示16个字节内存对齐，整个结构体补齐到16的倍数
    {
        PCL_ADD_POINT4D;     //使用PCL提供的添加点的三维坐标，不止添加xyz，还会添加额外的padding使坐标满足16字节对齐
        float intensity;
        std::uint32_t t;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //和EIGEN_ALIGN16配套使用，这个宏给结构体重载 new，保证动态分配这个对象时也满足 Eigen 对齐要求。
    };
}

//pcl注册点云
POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros:point,
    (float,x,x) (float,y,y) (float,z,z) (float,intensity,intensity) float(std::uint32_t,t,t) (std::uint16_t,ring,ring)
)

//Ouster::point
namespace Ouster_ros {
   struct EIGEN_ALIGN16 Point 
   {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      uint8_t ring;
      std::uint16_t ambinent;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   };
}

//pcl注册点云
POINT_CLOUD_REGISTER_POINT_STRUCT(
    Ouster_ros::point,
    (float,x,x) (float,y,y) (float ,z,z) (float ,intensity,intensity) (std::uint32_t ,t,t) (std::uint16_t, reflectivity,reflectivity)
    (std::uint8_t, ring,ring) (std::uint16_t, ambinent,ambinent) (std::uint32_t,range,range) 

)

class Preprocess
{
    Preprocess();
    ~Preprocess();

    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
    void process(const sensor_msg::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128];  //最多128lida扫描线，按线依次扫描点云，存点
    vector<orgtype> typess[128];  //pl_buff上的点一一对应，几何该扫描线上点和属性
    int lidar_type,point_filter_num,N_SCANS;   //lidar类型，点云间隔过滤数，lidar扫描线数

    double blind, blind_sqr;      //lidar盲区数，liadr盲区平方
    bool feature_enable, given_offset_time;  //lidar开关和点偏移时间
    ros::Publisher pub_full, pub_surf, pub_corn;  

    private:
       
       //将不同liadr的数据转换成统一格式
       void avia_handler(const livox_ros_driver::Custom::ConstPtr &msg);  
       void ouster_handler(const livox_ros_driver::Custom::ConstPtr &msg);
       void velodyne_handler(const livox_ros_driver::Custom::ConstPtr &msg);

       //根据一条扫描线上的点云 pl 和它们的几何属性 types，给每个点分类
       void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

       //特征提取
       int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
       bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
       bool Edge_Jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

       //间隔发布
       void pub_func(PointCloudXYZI &pl,const ros::time &ct);


       int group_size;                                       //判断平面连续点的数量
       double disA, disB, inf_bound;                         //判断相邻点距离是否随range变化正常，inf_bound为距离阈值
       double limit_maxmid, limit_midmin, limit_maxmin;      //判断一组点的距离比例关系
       double p2l_ratio;                                     //可能用于判断一组点是否近似共线或近似平面扫描段。
       double jump_up_limit, jump_down_limit;                //判断距离跳变的上下阈值。
       double cos160;                                        //例如如果两个向量夹角大于 160 度，说明它们几乎反向，可能表示一个平滑延展方向，或者用于排除异常边缘。
       double edgea, edgeb;                                  //可能用于根据距离自适应调整边缘阈值：
       double smallp_intersect, smallp_ratio;                //smallp_intersect 可能用于判断前后向量夹角或交会角。smallp_ratio 可能用于判断小平面点间距离比例。
       double vx, vy, vz;                                    //临时变量，可能用于存储方向向量分量。

};

typedef std::shared_ptr<Preprocess> PreprocessPtr;
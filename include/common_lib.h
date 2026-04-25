/*
在这里定义了lidar类型，slam模式，卡尔曼状态更新模式，时间同步的容器，滤波状态和更新结构
*/

#include <cmath>
#include<utils/so3_math.h>
#include<utils/colors.h>
#include<utils/types.h>

#include<opencv2/opencv.hpp>
#include<sensor_msgs/imu.h>
#include<sophus/se3.h>
#include<tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace sophus;

//调试宏，打印文件和行号
#define print_line std::count << __FILE__ << "," << __LINE__ << std::endl;

//grivaty const in guangdong of china
#define G_m_s2 (9.81) 

#define DIM_STATE(19)     //定位的状态19维
#define INIT_COV(0.01)     //初始化协方差0.01
#define SIZE_LARGE (500)    //最大数量500
#define SIZE_SMALL (100)     //最小数量100

#define VEC_FEOM_APPLY(v) v[0],v[1],v[2]
#define MAT_FROM_APPLY(v) v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/" +name))

//lidar类型
enum LID_TYPE
{
    AVIA = 1,
    VELO16 = 2,
    OUST64 = 3,
    L515,
    XT32,
    PANDAR128
};

//slam模式
enum SLAM_MODE
{
    ONLY_LO = 0,
    OMLY_LIO = 1,
    LIVO = 2
};

//卡尔曼状态
enum EKF_STATE
{
    WAIT = 0,
    VIO =1,
    LIO,
    LO
};

//
struct MeasureGroup 
{
    double vio_time;  //图像对应时间
    double lio_time;  //lio对应时间
    deque<sensor_msg::Imu::ConstPtr> imu;
    cv::Mat img;    //当前图像
    //默认构造函数，当差差创建该对象是会执行赋值
    MeasureGroup()
    {
      vio_time = 0.0;
      lio_time = 0.0;
    };
};

//lidar一帧数据的总容器，一帧lidar + 这一帧范围时间内多个measures
struct LidarMeasureGroup
{
    double lidar_frame_beg_time;
    double lidar_frame_end_time;
    double last_lio_update_time;

    PointCloudXYZI::Ptr lidar;  //当前帧点云
    PointCloudXYZI::Ptr pcl_proc_cur;  //当前帧lio更新的点云片段
    PointCloudXYZI::Ptr pcl_proc_nest;  //下一帧点云片段

    deque<struct MeasureGroup> Measures;
    EKF_STATE lio_vio_flg;
    int lidar_scan_index_now;

    LidarMeasureGroup()
    {
        lidar_frame_beg_time = -0.0;
        lidar_frame_end_time = 0.0;
        last_lio_update_time = -1.0;
        lio_vio_flg = WAIT;

        this->lidar.reset(new PointCloudXYZI());
        this->pcl_proc_cur.reset(new PointCloudXYZI());
        this->pcl_proc_nest.reset(new PointCloudXYZI());
        this->Measures.clear();

        lidar_scan_index_now = 0;   //对应图像插入时上下帧分隔位置
        last_lio_update_time = -1.0;
    };

};

//typedef 给 struct pointwithvar 重命名为 pointwithvar
typedef struct pointwithvar
{
   Eigen::Vector3d point_b;    //point in lidar body frame
   Eigen::Vector3d point_i;    //point in imu frame
   Eigen::Vector3d point_w;    //point in world frame

   Eigen::Matrix3d var_nostate;   //不包含状态协方差影响的点协方差，即点测量的不确定性
   Eigen::Matrix3d body_var;     //在body feame下的协方差
   Eigen::Matrix3d var;          //点的总协方差 ， 点的不确定性状态加状态估计不确定性
   Eigen::Matrix3d point_crossmat;   //点的反对称矩阵

   Eigen::Vector3d normal;      //点对应的平面法向量

   pointwithvar()
   {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
   };
}pointwithvar;


//保存滤波器当前 系统状态 + 状态协方差
struct StatesGroup
{

 
    M3D rot_end;
    V3D pos_end;
    V3D vel_end;
    double inv_expo_time;
    V3D bias_g;
    V3D bias_a;
    V3D gravity;
    Matrix<double, DIM_STATE, DIM_STATE> cov;

    StatesGroup()
    {
       this->rot_end = M3D::Identity();   //姿态R
       this->pos_end = V3D::Zero();       //位置P
       this->vel_end = V3D::Zero();       //速度V
       this->bias_g = V3D::Zero();        //陀螺仪偏置
       this->bias_a = V3D::Zero();        //加速度偏置
       this->grivaty = V3D::Zero();       //重力向量
       this->inv_expo_time = 1.0;         //相机逆曝光时间

       this->cov = MD(DIM_STATE ,DIM_STATE)::Identity() * INIT_COV;     //单位矩阵乘 0.01
       this->cov(6,6) = 0.00001;           //将第六维曝光协方差设置0.00001,说明比较信任逆曝光1
       this->cov.block<9,9>(10,10) = MD(9,9)::Identity() * 0.00001;    //将10 - 18维 单位矩阵乘 0.00001

    };

    //拷贝构造参数，从输入状态构造
    StatesGroup(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_a = b.bias_a;
        this->bias_g = b.bias_g;
        this->grivaty = b.grivaty;
        this->inv_expo_time = b.inv_expo_time;

        this->cov = b.cov;
    };

    //重载赋值 等于 = 符号 状态赋予
    StatesGroup &operator=(const Matrix<double, DIM_STATE,1> &statre_add)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_a = b.bias_a;
        this->bias_g = b.bias_g;
        this->grivaty = b.grivaty;
        this->inv_expo_time = b.inv_expo_time;

        this->cov = b.cov;

        return *this;   //是为了可以连续赋值
    };

    //新状态 = 当前状态 ⊞ 状态增量，因为是群所以不能直接加;  生成一个新状态而不修改原来的对象
    StatesGroup &operator+(const Matrix<double, DIM_STATE,1> &statre_add)
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
        a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.inv_expo_time = this->inv_expo_time + statre_add(6,0);
        a.vel_end = this->vel_end + statre_add.block<3,1>(7,0);
        a.bias_g = this->bias_g + statre_add<3,1>(10,0);
        a.bias_a = this->bias_a + statre_add<3,1>(13,0);
        a.grivaty = this->grivaty + state_add<3,1>(16,0);
        
        a.cov  =  this->cov;   //只更新状态值不更新协方差
        return a;
    };

    //直接修改当前状态更新
    StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
    {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
    };



    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
    {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    return a;
    };
 

    void resetpose()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = V3D::Zero();
        this->vel_end = V3D::Zero();
    }

}


//将imu/位姿状态打包成一个pose6D结构体返回
template <typename T>
auto set_pose6d(const double t, const Matrix<T,3,1> &a, const Matrix<T,3,1> &g, const Matrix<T,3,1> &v, const Matrix<T,3,1> &p, const Matrix<T,3,3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for(innt i =0 , i<3 ; i++)
    {
        rot_kp.acc[i]= a(i);
        rot_kp.gyr[i]= g(i);
        rot_kp.vel[i]= v(i);
        rot_kp.pos[i]= p(i);
        for (int j = 0; j < 3; J++)
           rot_kp.rot[i*3 + j] = R(i,j);
    }

    return move(rot_kp);
}
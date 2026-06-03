/*
观测帧
*/

#pragma once

#include "feature.h"
#include<boost/noncopyable.hpp>    //不允许拷贝
#include <vikit/abstract_camera.h>

class VisualPoint;
struct Feature;

typedef list<Feature *> Features;
typedef vector<cv::Mat> Imgpyr;

class Frame : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int frame_counter_;    //frame计数器,统计当前已经创建了多少frame

    int id_;                    //frame的唯一标识符，通常是一个整数，用于区分不同的帧。

    vk::abstract_camera *cam_;   //指向抽象相机对象的指针，表示这个帧使用的相机模型，可以是针孔相机、鱼眼相机等。
    SE3 T_f_w_;                    //frame中world 坐标系到 frame/camera 坐标系的变换。
    SE3 T_f_w_prior_;            //IMU 预测出来的当前帧先验位姿。

    cv::Mat img_;                //frame对应的图像数据，通常是一个OpenCV的Mat对象，存储了图像的像素值。
    Features fts_;                //Feature类表示在图像中检测到的特征点列表，这些特征点与这个帧相关联，fts_表示这个帧中检测到的特征点集合.
 

    Frame(vk::abstract_camera *cam, const cv::mat &img);
    ~Frame();
    

    void initFrame(const cv::Mat &img);    //初始化帧，通常包括图像预处理、特征提取等操作。


/*
这段是 Frame 类里的一组 坐标变换工具函数,核心变量是在SE3 T_f_w_;  从 world 坐标系到当前 frame/camera 坐标系的变换。
*/

    //返回观测到的特征点数量
    inline size_t nObs() const 
    {
        return fts_.size();
    }

    //将世界坐标系下的三维点 xyz_w 投影到图像平面上，返回其像素坐标。
    inline Vector2d w2c(const Vector3d &xyz_w)  const
    {
        return cam_->world2cam(T_f_w_ * xyz_w);
    }

    //将世界坐标系下的三维点 xyz_w 投影到图像平面上，使用先验位姿 T_f_w_prior_ 进行投影，返回其像素坐标。
    inline Vector2d w2c_prior(const Vector3d &xyz_w)  const
    {
        return cam_->world2cam(T_f_w_prior_ * xyz_w);
    }

    //将图像平面上的像素坐标 px 反投影成相机坐标系下的一条单位方向射线：
    inline Vector3d c2f(const Vector2d &px) const
    {
        return cam_->cam2world(px[0], px[1]);
    }
    
    //将图像平面上的像素坐标 (x, y) 反投影成相机坐标系下的一条单位方向射线：
    inline Vector3d c2f(const double x, const double y) const
    {
        return cam_->cam2world(x, y);
    }

    //将世界坐标系下的三维点 xyz_w 转换为frame坐标系下的三维点，返回其三维坐标。
    inline Vector3d w2f(const Vector3d &xyz_w) const
    {
        return T_f_w_ * xyz_w;
    }

    //将frame坐标系下的三维点 f 转换为世界坐标系下的三维点，返回其三维坐标。
    inline Vector3d f2w(const Vector3d &f) const
    {
        return T_f_w_.inverse() * f;
    }

    //将frame坐标系下的三维点 f 转换为图像平面上的像素坐标，返回其像素坐标。
    inline Vector2d f2c(const Vector3d &f) const
    {
        return cam_->world2cam(f);
    }

    //返回frame的position，即frame坐标系原点在世界坐标系下的位置。
    inline Vector3d pos() const
    {
        return T_f_w_.inverse().translation();
    }
};

//
typedef std::unique_ptr<frame> FramePtr;

namespace frame_utils
{

    //创建图像金字塔
    void createImgPyramid(const cv::Mat &img_level_0, int n_levels, Imgpyr &pyr);

}
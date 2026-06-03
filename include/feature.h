/*
特征patch
*/
#pragma once

#include "visual_point.h"
#include <vector>

struct Feature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum FeatureType
    {
        CORNER,    //角点
        EDGELET    //边缘点
    }

    //当前feature的编号
    int id_;

    //特征的类型
    FeatureType type_;

    //跟patch相关的图像
    cv::Mat img;

    //这个特征在金字塔图像层的像素坐标
    vector2d px_;

    //这个像素点对应的相机归一化方向向量，也叫 bearing vector。
    Vector3d f_;

    //图像特征所在的金字塔层数
    int level_;   

    //关联的三维视觉地图点
    VisualPoint *point_;

    //边缘点的主梯度方向，对于 EDGELET 类型特征，图像边缘只有一个方向上的约束更强，所以需要记录梯度方向。
    Vector2d grad_;
    
    //该特征所在的位姿
    SE3 T_f_w_;

    //指向图像 patch 数据的指针。
    float *patch_;

    //这个 patch feature 的分数。
    float score_;

    //patch 的平均灰度值。
    float mean_;

    //该图像帧的逆曝光时间。
    double inv_expo_time_;


    //返回世界坐标点
    inline vector3d pos() const 
    {
        return T_f_w_.inverse().translation();

    }


    //这个构造函数用于创建一个新的 Feature 观测。
    /*
    _point：这个 Feature 对应的三维地图点
    _patch：这个特征周围的图像 patch 数据
    _px：像素坐标
    _f：归一化相机射线方向
    _T_f_w：这个 Feature 所在图像帧的位姿
    _level：图像金字塔层级
    */
    Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3 &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), T_f_w_(_T_f_w), mean_(0), score_(0), level_(_level), patch_(_patch), point_(_point)
      {

      }


      //析构函数
      ~Feature()
      {
        delete[] patch_;
      }


}